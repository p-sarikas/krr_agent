#!/usr/bin/env python3

import math
import os
import time
from threading import Event

import tf_transformations
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from krr_mirte_skills_msgs.srv import GetObjectsInRoom, GetDropLocations
from plansys2_msgs.srv import AddProblem, ClearProblemKnowledge
from std_srvs.srv import Trigger
from std_msgs.msg import String
from typedb.driver import TypeDB, SessionType, TransactionType


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def make_pose_stamped(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

class ScannedObject:
    def __init__(self, entity_id: str, pose, room: str):
        self.entity_id = entity_id
        self.pose = pose
        self.room = room


class TaskManagerBase(Node):

    def __init__(
        self,
        node_name: str,
        enable_nav: bool = True,
        enable_drop_locations: bool = False,
        clear_problem_knowledge: bool = False,
        wait_for_task_status: bool = False,
        task_status_topic: str = '/task_status',
    ):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} started.')

        self.cb_group = ReentrantCallbackGroup()

        self.get_objects_client = self.create_client(
            GetObjectsInRoom, '/get_objects_in_room', callback_group=self.cb_group)

        self.get_drop_locs_client = (
            self.create_client(GetDropLocations, '/get_drop_locations', callback_group=self.cb_group)
            if enable_drop_locations else None
        )

        self.clear_problem_client = (
            self.create_client(
                ClearProblemKnowledge,
                '/problem_expert/clear_problem_knowledge',
                callback_group=self.cb_group,
            )
            if clear_problem_knowledge else None
        )

        self.add_problem_client = self.create_client(
            AddProblem, '/problem_expert/add_problem', callback_group=self.cb_group)

        self.trigger_cpp_client = self.create_client(
            Trigger, '/start_task_planning', callback_group=self.cb_group)

        self.nav_client = (
            ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
            if enable_nav else None
        )

        if wait_for_task_status:
            self.current_plan_status = None
            self.status_event = Event()
            self.sub_cb_group = MutuallyExclusiveCallbackGroup()
            self.status_sub = self.create_subscription(
                String,
                task_status_topic,
                self._status_cb,
                10,
                callback_group=self.sub_cb_group,
            )

        self._wait_for_services()

        self.typedb_address = 'localhost:1729'
        self.database_name = 'task_db'
        self.db_driver = TypeDB.core_driver(self.typedb_address)

        if not self.db_driver.databases.contains(self.database_name):
            raise ValueError(f"Database '{self.database_name}' does not exist.")

        self._load_knowledge_from_db()

    def destroy_node(self):
        if hasattr(self, 'db_driver') and self.db_driver is not None:
            self.db_driver.close()
        super().destroy_node()

    def _wait_for_services(self):
        services = [
            (self.get_objects_client, '/get_objects_in_room'),
            (self.add_problem_client, '/problem_expert/add_problem'),
            (self.trigger_cpp_client, '/start_task_planning'),
        ]

        if self.get_drop_locs_client is not None:
            services.append((self.get_drop_locs_client, '/get_drop_locations'))
        if self.clear_problem_client is not None:
            services.append((self.clear_problem_client, '/problem_expert/clear_problem_knowledge'))

        for client, name in services:
            self.get_logger().info(f'Waiting for {name} ...')
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{name} not available, retrying...')
        self.get_logger().info('All services available.')

    def _status_cb(self, msg: String):
        self.get_logger().info(f'Received task status: {msg.data}')
        self.current_plan_status = msg.data
        self.status_event.set()

    def wait_for_cpp_notification(self, timeout=120.0) -> bool:
        self.get_logger().info('Waiting for Task Controller notification...')
        self.current_plan_status = None
        self.status_event.clear()

        signaled = self.status_event.wait(timeout)
        if not signaled:
            self.get_logger().error('TIMEOUT: No signal received from C++.')
            return False

        if self.current_plan_status == 'SUCCESS':
            self.get_logger().info('Received SUCCESS signal from C++!')
            return True

        self.get_logger().error(f'Received {self.current_plan_status} signal from C++!')
        return False

    def _send_nav_goal_and_wait(self, pose: PoseStamped):
        if self.nav_client is None:
            raise RuntimeError('Navigation client is not enabled for this manager.')

        self.nav_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        self.get_logger().info('Navigation complete.')

    def _load_problem_and_trigger_cpp(self, problem_str: str) -> bool:
        if self.clear_problem_client is not None:
            clear_req = ClearProblemKnowledge.Request()
            f_clear = self.clear_problem_client.call_async(clear_req)
            while not f_clear.done():
                time.sleep(0.05)

        self.get_logger().info('--- Sending new PDDL Problem to PlanSys2 ---')
        req = AddProblem.Request()
        req.problem = problem_str
        f = self.add_problem_client.call_async(req)
        while not f.done():
            time.sleep(0.05)

        if not f.result() or not getattr(f.result(), 'success', True):
            self.get_logger().error('Failed to load PDDL problem.')
            return False

        self.get_logger().info('PDDL problem loaded. Triggering C++ Controller...')
        trigger_req = Trigger.Request()
        trigger_future = self.trigger_cpp_client.call_async(trigger_req)
        while not trigger_future.done():
            time.sleep(0.05)

        if trigger_future.result() and trigger_future.result().success:
            self.get_logger().info('C++ Controller acknowledged and is now executing the plan!')
            return True

        self.get_logger().error('Failed to trigger C++ Controller.')
        return False

    def _execute_phase_pddl(self, problem_str: str) -> bool:
        if self.clear_problem_client is None: # One PDDL case: not need of waiting for completion
            return self._load_problem_and_trigger_cpp(problem_str)

        self.get_logger().info('--- Clearing old PDDL state ---')
        clear_req = ClearProblemKnowledge.Request()
        clear_future = self.clear_problem_client.call_async(clear_req)
        while not clear_future.done():
            time.sleep(0.05)

        self.get_logger().info('--- Sending new PDDL Problem to PlanSys2 ---')
        req = AddProblem.Request()
        req.problem = problem_str
        f = self.add_problem_client.call_async(req)
        while not f.done():
            time.sleep(0.05)
        
        if not f.result() or not f.result().success:
            self.get_logger().error('Failed to load PDDL problem.')
            return False

        self.get_logger().info('Triggering Task Controller...')
        trigger_future = self.trigger_cpp_client.call_async(Trigger.Request())
        while not trigger_future.done():
            time.sleep(0.05)

        if trigger_future.result() and trigger_future.result().success:
            self.get_logger().info('C++ Controller acknowledged and is now executing the plan!')
            success = self.wait_for_cpp_notification(timeout=300.0)

            if success:
                self.get_logger().info('Room tidied successfully! Moving to next phase.')
            return success
        else:
            self.get_logger().error('Failed to trigger C++ Controller.')
            return False

    def _get_objects_in_room(self):
        req = GetObjectsInRoom.Request()
        future = self.get_objects_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        if future.result() is None:
            self.get_logger().error('get_objects_in_room service call failed.')
            return []

        return future.result().room_object_poses

    def _get_obstacles_in_room(self):
        req = GetObjectsInRoom.Request()
        future = self.get_objects_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        if future.result() is None:
            self.get_logger().error('get_objects_in_room service call failed.')
            return []

        return future.result()

    def _get_drop_locations_in_room(self):
        if self.get_drop_locs_client is None:
            raise RuntimeError('Drop-location service client is not enabled.')

        req = GetDropLocations.Request()
        future = self.get_drop_locs_client.call_async(req)
        while not future.done():
            time.sleep(0.05)

        if future.result() is None or not future.result().success:
            self.get_logger().error('get_drop_locations service call failed.')
            return []

        return future.result().drop_locations

    def get_drop_locations_in_room(self, room_name: str) -> dict:
        query = f"""
            match
                $room isa room, has room-name \"{room_name}\";
                (container: $room, contained-pose: $pose) isa spatial-containment;
                $drop isa drop-location, has id $drop_id;
                (located-target: $drop, location: $pose) isa physical-location;
                $pose has pos-x $x, has pos-y $y;
            get $drop_id, $x, $y;
        """
        drops = {}
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                for answer in tx.query.get(query):
                    d_id = answer.get('drop_id').as_attribute().get_value()
                    x = answer.get('x').as_attribute().get_value()
                    y = answer.get('y').as_attribute().get_value()
                    drops[d_id] = (x, y)
        return drops

    def insert_scanned_object(self, obj_id: str, room_name: str, x: float, y: float, handled: bool = False):
        handled_str = "true" if handled else "false"

        query = f"""
            match
                $room isa room, has room-name "{room_name}";
            insert
                $obj isa item, has id "{obj_id}", has handled {handled_str};
                $pose isa pose,
                    has pos-x {x}, has pos-y {y}, has pos-z 0.0,
                    has rot-x 0.0, has rot-y 0.0, has rot-z 0.0, has rot-w 1.0;
                (located-item: $obj, location: $pose) isa physical-location;
                (container: $room, contained-pose: $pose) isa spatial-containment;
        """
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as tx:
                tx.query.insert(query)
                tx.commit()

    def insert_drop_location(self, drop_id: str, room_name: str, x: float, y: float):
        query = f"""
            match
                $room isa room, has room-name \"{room_name}\";
            insert
                $drop isa drop-location, has id \"{drop_id}\";
                $pose isa pose,
                    has pos-x {x}, has pos-y {y}, has pos-z 0.0,
                    has rot-x 0.0, has rot-y 0.0, has rot-z 0.0, has rot-w 1.0;
                (located-target: $drop, location: $pose) isa physical-location;
                (container: $room, contained-pose: $pose) isa spatial-containment;
        """
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as tx:
                tx.query.insert(query)
                tx.commit()

    def get_known_object_at(self, x: float, y: float, threshold: float = 0.4) -> str:
        query = """
            match 
                $obj isa item, has id $id; 
                $p isa pose, has pos-x $px, has pos-y $py; 
                (located-item: $obj, location: $p) isa physical-location; 
            get $id, $px, $py;
        """
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                for answer in tx.query.get(query):
                    px = answer.get('px').as_attribute().get_value()
                    py = answer.get('py').as_attribute().get_value()
                    if euclidean_distance(x, y, px, py) < threshold:
                        return answer.get('id').as_attribute().get_value()
        return None

    def _make_drop_id(self, drop_type: str) -> str:
        mapping = {
            'dishwasher': 'loc_drop_1_dishwasher',
            'tableware': 'loc_drop_2_tableware',
            'livingroom': 'loc_drop_3_livingroom',
            'living_room': 'loc_drop_3_livingroom',
            'toys': 'loc_drop_4_toys',
            'general': 'loc_drop_5_general',
            'bedroom': 'loc_drop_6_bedroom',
            'trash': 'loc_drop_7_trash',
        }
        normalized = drop_type.lower().replace(' ', '_')
        return mapping.get(normalized, f'loc_drop_unknown_{normalized}')
    
    def update_object_poses(self, obj_id: str, x: float, y: float):
        query = f"""
            match
                $obj isa item, has id "{obj_id}";
                $pose isa pose, has pos-x $px, has pos-y $py;
                (located-item: $obj, location: $pose) isa physical-location;
            update
                $pose has pos-x {x}, has pos-y {y};
        """
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as tx:
                tx.query.update(query)
                tx.commit()

        self.get_logger().info(f"Updated TypeDB: {obj_id} moved to ({x:.2f}, {y:.2f})")
        
    def _recover_and_update_poses(self, current_room: str):
        """Scans the room and updates the positions of known objects if they moved."""
        self.get_logger().warn(f"Action failed. Performing recovery scan in {current_room}...")
        
        # 1. Get current physical reality
        new_observations = self._get_objects_in_room()
        
        # 2. Match new observations to known IDs using a wider threshold (e.g., 1.0m)
        for pose in new_observations:
            # We look for a known object near the new observation
            existing_id = self.get_known_object_at(pose.position.x, pose.position.y, threshold=1.0)
            
            if existing_id:
                # Update the database with the more accurate current position
                self.update_object_poses(existing_id, pose.position.x, pose.position.y)
                

    def _load_knowledge_from_db(self):
        raise NotImplementedError('Derived task manager classes must implement _load_knowledge_from_db.')
		
