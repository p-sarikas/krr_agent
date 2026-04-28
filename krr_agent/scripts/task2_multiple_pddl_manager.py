#!/usr/bin/env python3

import os
import time
import threading
import rclpy
from ament_index_python.packages import get_package_share_directory
from typedb.driver import SessionType, TransactionType
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

# Assuming task_manager_base.py is in the same directory or PYTHONPATH
from task_manager_base import TaskManagerBase, euclidean_distance, make_pose_stamped, ScannedObject

# ---------------------------------------------------------------------------
# CONSTANTS & HELPERS
# ---------------------------------------------------------------------------

START_ROOM = 'kitchen'
pkg_share_dir = get_package_share_directory('krr_agent')
PROBLEM_TEMPLATE_PATH = os.path.join(pkg_share_dir, 'pddl', 'problem_t2_template.pddl')
PROBLEM_FILE_PATH = '/tmp/problem_t2_multiple_pddl_generated.pddl'


def build_pddl_problem(
        scanned_objects: list,
        rooms: list,        # only current room and next room 
        scan_locations: dict, # only the next room scan location
        start_room: str,
        next_room_scan_loc: str) -> str: # current room-
    
    global ROOM_ADJACENCY

    # -----------------------
    # DECLARATIONS
    # -----------------------

    ## Item declarations 
    item_lines = '\n'.join(f'    {o.entity_id} - item' for o in scanned_objects)
    
    
    ## Location declarations
    location_parts = []
    # Scan locations
    for r in rooms:
        if r in scan_locations:
            location_parts.append(f'    {scan_locations[r]} - location')

    # Object locations
    for o in scanned_objects:
        location_parts.append(f'    loc_{o.entity_id} - location')
    
    location_parts.append('\n    loc_drop_dummy - location')
    location_lines = '\n'.join(location_parts)
    
    ## Room declarations 
    room_lines = '\n'.join(f'    {r} - room' for r in rooms)


    # -----------------------
    # PREDICATES
    # -----------------------
    current_agent_loc = scan_locations[start_room]

    # Tidying predicate (start_room only at start)
    tidying_lines = f'    (tidying {start_room})'

    # Next room untidy (so it can execute next_room action)
    untidy_parts = []
    for r in rooms:
        untidy_parts.append(f'    (untidy {r})')
    untidy_lines = '\n'.join(untidy_parts)

    tidy_lines = '' # Handled dynamically by next_room action

    ## --- Scan locations ---
    scan_loc_parts = []
    for r in rooms:
        if r in scan_locations:
            scan_loc_parts.append(f'    (scan_loc {scan_locations[r]})')
    scan_loc_lines = '\n'.join(scan_loc_parts)

    ## --- Location in room ---
    loc_in_room_parts = []
    for r in rooms:
        if r in scan_locations:
            loc_in_room_parts.append(f'    (location_in_room {scan_locations[r]} {r})')

    for o in scanned_objects:
        loc_in_room_parts.append(f'    (location_in_room loc_{o.entity_id} {o.room})')
    loc_in_room_lines = '\n'.join(loc_in_room_parts)

    # --- Item at ---
    item_at_parts = []
    for o in scanned_objects:
        item_at_parts.append(f'    (item_at {o.entity_id} loc_{o.entity_id})')
    item_at_lines = '\n'.join(item_at_parts)

    current_room = start_room
    next_room = [r for r in rooms if r != start_room][0]
    adjacency_lines = '\n'.join([
        f'    (adjacent {current_room} {next_room})',
        f'    (adjacent {next_room} {current_room})'])

    # -----------------------
    # GOALS
    # -----------------------
    goal_parts = []
    goal_parts.append(f'    (tidy {start_room})\n')
    
    for o in scanned_objects:
        goal_parts.append(f'    (on_drop_loc {o.entity_id})')
        
    goal_parts.append(f'\n    (agent_at {next_room_scan_loc})')
    goal_lines = '\n'.join(goal_parts)


    # -----------------------
    # FILL IN THE TEMPLATE
    # -----------------------
    with open(PROBLEM_TEMPLATE_PATH, 'r') as f:
        content = f.read()

    content = content.replace('START_LOCATION', current_agent_loc)
    content = content.replace('    ; PLACEHOLDER: item declarations',        item_lines)
    content = content.replace('    ; PLACEHOLDER: location declarations',    location_lines)
    content = content.replace('    ; PLACEHOLDER: room declarations',        room_lines)
    content = content.replace('    ; PLACEHOLDER: untidy predicates',        untidy_lines)
    content = content.replace('    ; PLACEHOLDER: tidying predicates',       tidying_lines)
    content = content.replace('    ; PLACEHOLDER: tidy predicates',          tidy_lines)
    content = content.replace('    ; PLACEHOLDER: scan_loc predicates',      scan_loc_lines)
    content = content.replace('    ; PLACEHOLDER: location_in_room predicates', loc_in_room_lines)
    content = content.replace('    ; PLACEHOLDER: adjacent predicates',      adjacency_lines)
    content = content.replace('    ; PLACEHOLDER: item_at predicates',       item_at_lines)
    content = content.replace('    ; PLACEHOLDER: on_drop_loc goals',        goal_lines)

    # --- Write to disk ---
    try:
        os.makedirs(os.path.dirname(os.path.abspath(PROBLEM_FILE_PATH)), exist_ok=True)
        with open(PROBLEM_FILE_PATH, 'w') as f:
            f.write(content)
        print(f'[INFO] problem_t2.pddl written with {len(scanned_objects)} object(s).')
    except OSError as e:
        print(f'[WARN] Could not write problem.pddl: {e}')

    return content


# ---------------------------------------------------------------------------
# MAIN NODE
# ---------------------------------------------------------------------------

class TaskManagerNode(TaskManagerBase):
    def __init__(self):
        # We pass all configuration to the base class constructor
        super().__init__(
            node_name='task2_manager_node',
            enable_nav=True,
            enable_drop_locations=False, 
            clear_problem_knowledge=True,
            wait_for_task_status=True,
        )
        
        self.get_logger().info('Task 2 manager node started.')
        self._task_started = False

        # Background thread for the state machine
        self.task_thread = threading.Thread(target=self._start_task)
        self.task_thread.start()

    def _load_knowledge_from_db(self):
        """
        Implementation of the abstract method from TaskManagerBase.
        Loads waypoints and scan locations specifically for Task 2.
        """
        self.room_waypoints = {}
        self.scan_locations = {}
        self.drop_to_room = {}

        self.get_logger().info("Loading environment knowledge from TypeDB...")

        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                # Query drop locations
                query_drops = """
                    match
                        $rm isa room, has room-name $r_name;
                        $d isa drop-location, has id $d_id;
                        $p isa pose;
                        (located-target: $d, location: $p) isa physical-location;
                        (container: $rm, contained-pose: $p) isa spatial-containment;
                    get $r_name, $d_id;
                """
                for answer in tx.query.get(query_drops):
                    room = answer.get("r_name").as_attribute().get_value()
                    drop_id = answer.get("d_id").as_attribute().get_value()
                    self.drop_to_room[drop_id] = room

                # Query scan locations
                query_scans = """
                    match
                        $rm isa room, has room-name $r_name;
                        $scan isa scan-location;
                        $p isa pose, has pos-x $x, has pos-y $y;
                        (located-target: $scan, location: $p) isa physical-location;
                        (container: $rm, contained-pose: $p) isa spatial-containment;
                    get $r_name, $x, $y;
                """
                for answer in tx.query.get(query_scans):
                    room = answer.get("r_name").as_attribute().get_value()
                    x = answer.get("x").as_attribute().get_value()
                    y = answer.get("y").as_attribute().get_value()
                    self.room_waypoints[room] = (x, y, 0.0) 
                    self.scan_locations[room] = f"scan_{room}"

    def _start_task(self):
        if self._task_started:
            return
        self._task_started = True
        self.run_task()

    def run_task(self):
        """
        Main logic loop for Task 2.
        """
        unvisited_rooms = list(self.room_waypoints.keys())
        ordered_rooms = []

        # Simple greedy pathing
        if START_ROOM in unvisited_rooms:
            ordered_rooms.append(START_ROOM)
            unvisited_rooms.remove(START_ROOM)
            current_loc = self.room_waypoints[START_ROOM]
        else:
            current_loc = (0.0, 0.0, 0.0)

        # Greedy search for room scanning order
        while unvisited_rooms:
            next_room = min(
                unvisited_rooms, 
                key=lambda r: euclidean_distance(
                    current_loc[0], current_loc[1], 
                    self.room_waypoints[r][0], self.room_waypoints[r][1]
                )
            )
            ordered_rooms.append(next_room)
            unvisited_rooms.remove(next_room)
            current_loc = self.room_waypoints[next_room]


        for i, room_name in enumerate(ordered_rooms):
            all_scanned_objects = []
            all_drop_locations = {}

            wx, wy, wyaw = self.room_waypoints[room_name]

            # The PDDL does it, but just in case
            self.get_logger().info(f'Navigating to {room_name}...')
            self._send_nav_goal_and_wait(make_pose_stamped(wx, wy, wyaw))
            
            self.get_logger().info(f'Scanning {room_name}...')
            object_poses = self._get_objects_in_room()

            room_drops = self.get_drop_locations_in_room(room_name)
            all_drop_locations.update(room_drops)
            self.get_logger().info(
                f'Found {len(object_poses)} object(s) and '
                f'{len(room_drops)} drop location(s) in {room_name}.')

            # Empty room navigation
            if not object_poses:
                self.get_logger().info(f'No objects in {room_name}. Moving to next waypoint...')
                if i + 1 < len(ordered_rooms):
                    next_room = ordered_rooms[i+1]
                    nx, ny, nyaw = self.room_waypoints[next_room]
                    self._send_nav_goal_and_wait(make_pose_stamped(nx, ny, nyaw))
                continue


            for obj_idx, pose in enumerate(object_poses):
                obj_id = f'obj_{obj_idx}_{room_name}'

                # Filtering out objects that are already on drops in this room (if any)
                is_on_drop = False
                for drop_id, (dx, dy) in room_drops.items():
                    if euclidean_distance(pose.position.x, pose.position.y, dx, dy) < 0.5:
                        self.get_logger().info(
                            f'  Skipping {obj_id} at ({pose.position.x:.2f}, '
                            f'{pose.position.y:.2f}) because it is on drop {drop_id}.')
                        is_on_drop = True
                        break

                # Insert object as a generic item
                self.insert_scanned_object(
                    obj_id, room_name, pose.position.x, pose.position.y, handled=is_on_drop)

                if not is_on_drop:
                    obj = ScannedObject(obj_id, pose, room_name)
                    all_scanned_objects.append(obj)

                    self.get_logger().info(
                        f'  Found {obj_id} at ({pose.position.x:.2f}, '
                        f'{pose.position.y:.2f}). Target: [UNKNOWN PENDING PERCEPTION]')

                
            if not all_scanned_objects:
                self.get_logger().info(f'No objects found in {room_name}. Skipping to next room.')                
                continue
        
            self.get_logger().info(
                f'Building PDDL problem with {len(all_scanned_objects)} '
                f'total object(s)...')
            
            
            # Choosing next room:
            pass_rooms = []
            current_index = ordered_rooms.index(room_name)

            if current_index == len(ordered_rooms) - 1:
                pass_rooms.append(room_name)
                pass_rooms.append(START_ROOM)
                next_room_scan_loc = self.scan_locations[START_ROOM]
            else:
                next_room_r = ordered_rooms[current_index + 1]
                pass_rooms.append(room_name)
                pass_rooms.append(next_room_r)
                next_room_scan_loc = self.scan_locations[next_room_r]


            # Generar problema simplificado
            problem_str = build_pddl_problem(
                all_scanned_objects, 
                pass_rooms,
                self.scan_locations,
                room_name,
                next_room_scan_loc)
        
            self.get_logger().info(f'Starting execution for {room_name}...')
            self._execute_phase_pddl(problem_str)

        self.get_logger().info('All rooms scanned. Returning to start room...')
        wx, wy, wyaw = self.room_waypoints.get('kitchen', (0.0, 0.0, 0.0))
        self._send_nav_goal_and_wait(make_pose_stamped(wx, wy, wyaw))

        self.get_logger().info('Task 2 Complete.')

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()