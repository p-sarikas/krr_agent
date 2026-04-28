#!/usr/bin/env python3

import os
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from typedb.driver import SessionType, TransactionType

# Import your base class and shared utilities
# Ensure task_manager_base.py is in your PYTHONPATH or the same directory
from task_manager_base import TaskManagerBase, euclidean_distance, make_pose_stamped, ScannedObject

# ---------------------------------------------------------------------------
# CONFIGURATION & PATHS
# ---------------------------------------------------------------------------

START_ROOM = 'kitchen'
pkg_share_dir = get_package_share_directory('krr_agent')
PROBLEM_TEMPLATE_PATH = os.path.join(pkg_share_dir, 'pddl', 'problem_t1_template.pddl')
PROBLEM_FILE_PATH = '/tmp/problem_t1_generated.pddl'

# ---------------------------------------------------------------------------
# PDDL BUILDER (Task 1 Specific)
# ---------------------------------------------------------------------------

def build_pddl_problem(
        scanned_objects, all_drop_locations, object_to_closest_drop, 
        rooms, scan_locations, drop_to_room, start_room, next_room_scan_loc):
    """
    Constructs a PDDL problem for a specific room tidying phase.
    """
    # Item and Location declarations
    item_lines = '\n'.join(f'    {o.entity_id} - item' for o in scanned_objects)
    
    loc_parts = [f'    {scan_locations[r]} - location' for r in rooms if r in scan_locations]
    loc_parts += [f'    loc_{o.entity_id} - location' for o in scanned_objects]
    loc_parts += [f'    {d_id} - location' for d_id in all_drop_locations.keys()]
    location_lines = '\n'.join(loc_parts)

    room_lines = '\n'.join(f'    {r} - room' for r in rooms)

    # Predicates
    loc_in_room_parts = [f'    (location_in_room {scan_locations[r]} {r})' for r in rooms if r in scan_locations]
    loc_in_room_parts += [f'    (location_in_room loc_{o.entity_id} {o.room})' for o in scanned_objects]
    loc_in_room_parts += [f'    (location_in_room {d_id} {drop_to_room.get(d_id, start_room)})' 
                          for d_id in all_drop_locations.keys()]
    
    # Simple Adjacency for the phase
    next_room = [r for r in rooms if r != start_room][0]
    adjacency_lines = f'    (adjacent {start_room} {next_room})\n    (adjacent {next_room} {start_room})'

    # Goals
    goal_lines = '\n'.join(f'    (on_drop_loc {o.entity_id} {object_to_closest_drop[o.entity_id]})' for o in scanned_objects)
    tidy_goal_lines = f'    (tidy {start_room})\n    (agent_at {next_room_scan_loc})'

    with open(PROBLEM_TEMPLATE_PATH, 'r') as f:
        content = f.read()

    replacements = {
        'START_LOCATION': scan_locations[start_room],
        '; PLACEHOLDER: item declarations': item_lines,
        '; PLACEHOLDER: location declarations': location_lines,
        '; PLACEHOLDER: room declarations': room_lines,
        '; PLACEHOLDER: tidying predicates': f'    (tidying {start_room})',
        '; PLACEHOLDER: untidy predicates': '\n'.join(f'    (untidy {r})' for r in rooms),
        '; PLACEHOLDER: scan_loc predicates': '\n'.join(f'    (scan_loc {scan_locations[r]})' for r in rooms if r in scan_locations),
        '; PLACEHOLDER: location_in_room predicates': '\n'.join(loc_in_room_parts),
        '; PLACEHOLDER: drop_loc predicates': '\n'.join(f'    (drop_loc {d})' for d in all_drop_locations.keys()),
        '; PLACEHOLDER: adjacent predicates': adjacency_lines,
        '; PLACEHOLDER: item_at predicates': '\n'.join(f'    (item_at {o.entity_id} loc_{o.entity_id})' for o in scanned_objects),
        '; PLACEHOLDER: on_drop_loc goals': goal_lines,
        '; PLACEHOLDER: tidy goals': tidy_goal_lines
    }

    for k, v in replacements.items():
        content = content.replace(k, v)

    with open(PROBLEM_FILE_PATH, 'w') as f:
        f.write(content)
    
    return content

# ---------------------------------------------------------------------------
# MAIN NODE
# ---------------------------------------------------------------------------

class TaskManagerNode(TaskManagerBase):

    def __init__(self):
        super().__init__(
            node_name='task_manager_node',
            enable_nav=True,
            enable_drop_locations=True,
            clear_problem_knowledge=True,
            wait_for_task_status=True,
        )
        self.get_logger().info('Task 1 Manager (Optimized) started.')
        self._task_started = False
        self.task_thread = threading.Thread(target=self._start_task)
        self.task_thread.start()

    def _load_knowledge_from_db(self):
        """Loads rooms and scan locations from TypeDB."""
        self.room_waypoints, self.scan_locations, self.drop_to_room = {}, {}, {}

        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                # Load Scan Locations
                for answer in tx.query.get("match $rm isa room, has room-name $rn; $scan isa scan-location; $p isa pose, has pos-x $x, has pos-y $y; (container: $rm, contained-pose: $p) isa spatial-containment; (located-target: $scan, location: $p) isa physical-location; get $rn, $x, $y;"):
                    r = answer.get("rn").as_attribute().get_value()
                    self.room_waypoints[r] = (answer.get("x").as_attribute().get_value(), answer.get("y").as_attribute().get_value(), 0.0)
                    self.scan_locations[r] = f"scan_{r}"

    def _start_task(self):
        if self._task_started: return
        self._task_started = True
        self.run_task()

    def run_task(self):
        """Greedy scanning and tidy-up execution loop."""
        unvisited = list(self.room_waypoints.keys())
        ordered_rooms = []
        current_loc = self.room_waypoints.get(START_ROOM, (0.0, 0.0, 0.0))
        
        if START_ROOM in unvisited:
            ordered_rooms.append(START_ROOM)
            unvisited.remove(START_ROOM)

        while unvisited:
            next_room = min(unvisited, key=lambda r: euclidean_distance(current_loc[0], current_loc[1], self.room_waypoints[r][0], self.room_waypoints[r][1]))
            ordered_rooms.append(next_room)
            unvisited.remove(next_room)
            current_loc = self.room_waypoints[next_room]

        for i, room_name in enumerate(ordered_rooms):
            # Navigate using parent method
            self._send_nav_goal_and_wait(make_pose_stamped(*self.room_waypoints[room_name]))

            # Scan and get closest drops
            object_poses = self._get_objects_in_room()


            scanned_drops = self._get_drop_locations_in_room() 
            room_drops = {}
            
            # Knowledge Update: Process Drop Locations
            for drop in scanned_drops:
                drop_type = drop.type.data.strip()   # e.g. "dishwasher", "tableware", ...
                drop_pose = drop.drop_pose
                drop_id = self._make_drop_id(drop_type)
                
                x = drop_pose.position.x
                y = drop_pose.position.y
                
                room_drops[drop_id] = (x, y)
                self.drop_to_room[drop_id] = room_name
                
                # Update TypeDB to anchor the physical reality into symbolic knowledge
                self.insert_drop_location(drop_id, room_name, x, y)
                self.get_logger().info(f"Registered drop location {drop_id} in {room_name}")

            
            all_scanned_objects, object_to_closest_drop = [], {}

            for idx, pose in enumerate(object_poses):
                obj_id = f'obj_{idx}_{room_name}'
                self.insert_scanned_object(obj_id, room_name, pose.position.x, pose.position.y)
                all_scanned_objects.append(ScannedObject(obj_id, pose, room_name))

                closest_drop = min(room_drops.keys(), key=lambda d: euclidean_distance(pose.position.x, pose.position.y, room_drops[d][0], room_drops[d][1]))
                object_to_closest_drop[obj_id] = closest_drop

            if not all_scanned_objects:
                continue

            # Planning Phase
            next_idx = i + 1 if i + 1 < len(ordered_rooms) else 0
            next_scan_loc = self.scan_locations[ordered_rooms[next_idx]]

            problem_str = build_pddl_problem(all_scanned_objects, room_drops, object_to_closest_drop, 
                                             [room_name, ordered_rooms[next_idx]], self.scan_locations, 
                                             self.drop_to_room, room_name, next_scan_loc)
            
            # Execute PDDL using parent method
            self._execute_phase_pddl(problem_str)

        self.get_logger().info('Task 1 Complete.')

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()