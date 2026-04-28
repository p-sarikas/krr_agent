#!/usr/bin/env python3

import os
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from typedb.driver import SessionType, TransactionType

# Import your base class and shared utilities
from task_manager_base import TaskManagerBase, euclidean_distance, make_pose_stamped, ScannedObject

# ---------------------------------------------------------------------------
# CONFIGURATION & PATHS
# ---------------------------------------------------------------------------

START_ROOM = 'kitchen'
START_LOCATION = 'scan_kitchen'
ROOM_ADJACENCY = []
pkg_share_dir = get_package_share_directory('krr_agent')
PROBLEM_TEMPLATE_PATH = os.path.join(pkg_share_dir, 'pddl', 'problem_t2_template.pddl')
PROBLEM_FILE_PATH = '/tmp/problem_t2_generated.pddl'

# ---------------------------------------------------------------------------
# PDDL HELPERS
# ---------------------------------------------------------------------------

def build_pddl_problem(
        scanned_objects: list,
        rooms: list,
        scan_locations: dict,
        start_room: str) -> str:

    global ROOM_ADJACENCY

    # -----------------------
    # DECLARATIONS
    # -----------------------

    ## --- Item declarations ---
    item_lines = '\n'.join(
        f'    {o.entity_id} - item' for o in scanned_objects)
    
    ## --- Location declarations ---
    location_parts = []

    # Scan locations
    for scan_loc in scan_locations.values():
        location_parts.append(f'    {scan_loc} - location')

    # Object locations
    for o in scanned_objects:
        location_parts.append(f'    loc_{o.entity_id} - location')
    location_lines = '\n'.join(location_parts)
    
    ## --- Room declarations ---
    room_lines = '\n'.join(f'    {r} - room' for r in rooms)

    # Tidying predicate (start_room only at start)
    tidying_lines = f'    (tidying {start_room})'

    # Untidy for rooms with items 
    rooms_with_items = {o.room for o in scanned_objects}
    empty_rooms = set(rooms) - rooms_with_items

    untidy_lines = '\n'.join(
        f'    (untidy {r})' for r in rooms_with_items)
    
    # Tidy for empty rooms except start_room (which is tidying)
    tidy_lines = '\n'.join(
        f'    (tidy {r})' for r in empty_rooms if r != start_room)

    # -----------------------
    # PREDICATES
    # -----------------------

    ## --- Location predicates ---
    scan_loc_lines = '\n'.join(f'    (scan_loc {loc})' for loc in scan_locations.values())

    loc_in_room_parts = []
    for room, scan_loc in scan_locations.items(): 
        loc_in_room_parts.append(f'    (location_in_room {scan_loc} {room})')
        
    for o in scanned_objects:
        loc_in_room_parts.append(f'    (location_in_room loc_{o.entity_id} {o.room})')
    loc_in_room_lines = '\n'.join(loc_in_room_parts)

    # adjacency_lines = '\n'.join(
    #     f'    (adjacent {r1} {r2})' for (r1, r2) in ROOM_ADJACENCY)
    
    adj_pairs = set(ROOM_ADJACENCY)
    adj_pairs.update((r2, r1) for (r1, r2) in ROOM_ADJACENCY)
    adjacency_lines = '\n'.join(
        f'    (adjacent {r1} {r2})' for (r1, r2) in sorted(adj_pairs))

    item_at_lines = '\n'.join(
        f'    (item_at {o.entity_id} loc_{o.entity_id})'
        for o in scanned_objects)

    # ---- Goals predicates ---
    goal_lines = '\n'.join(
        f'    (on_drop_loc {o.entity_id})'
        for o in scanned_objects)

    tidy_goal_lines = '\n'.join(
        f'    (tidy {r})' for r in rooms)

    # -----------------------
    # FILL IN THE TEMPLATE
    # -----------------------
    with open(PROBLEM_TEMPLATE_PATH, 'r') as f:
        content = f.read()

    content = content.replace('START_LOCATION', START_LOCATION)
    content = content.replace('    ; PLACEHOLDER: item declarations',        item_lines)
    content = content.replace('    ; PLACEHOLDER: location declarations',    location_lines)
    content = content.replace('    ; PLACEHOLDER: room declarations',        room_lines)
    content = content.replace('    ; PLACEHOLDER: tidying predicates',       tidying_lines)
    content = content.replace('    ; PLACEHOLDER: untidy predicates',        untidy_lines)
    content = content.replace('    ; PLACEHOLDER: tidy predicates',          tidy_lines)
    content = content.replace('    ; PLACEHOLDER: scan_loc predicates',      scan_loc_lines)
    content = content.replace('    ; PLACEHOLDER: location_in_room predicates', loc_in_room_lines)
    content = content.replace('    ; PLACEHOLDER: adjacent predicates',      adjacency_lines)
    content = content.replace('    ; PLACEHOLDER: item_at predicates',       item_at_lines)
    content = content.replace('    ; PLACEHOLDER: on_drop_loc goals',        goal_lines)
    content = content.replace('    ; PLACEHOLDER: tidy goals',               tidy_goal_lines)
    
    # --- Write to disk ---
    try:
        os.makedirs(os.path.dirname(os.path.abspath(PROBLEM_FILE_PATH)),
                    exist_ok=True)
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
        super().__init__(
            node_name='task2_manager_node',
            enable_nav=True,
            enable_drop_locations=True,
            clear_problem_knowledge=False,
            wait_for_task_status=False,
        )
        self.get_logger().info('Task 2 Manager (Inherited) started.')
        self._task_started = False
        
        # Start main thread
        self.task_thread = threading.Thread(target=self._start_task)
        self.task_thread.start()

    def _load_knowledge_from_db(self):
        """Loads static mappings from TypeDB to replace hardcoded Python dictionaries."""
        global ROOM_ADJACENCY
        self.room_waypoints = {}
        self.scan_locations = {}
        self.drop_to_room = {}  # populated at runtime via perception
        self.room_adjacency = []

        self.get_logger().info("Loading environment knowledge from TypeDB...")

        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                
                # 1. Fetch adjacency predicates
                query_adjacency = """
                    match
                        $r1 isa room, has room-name $name1;
                        $r2 isa room, has room-name $name2;
                        (room-a: $r1, room-b: $r2) isa adjacent;
                    get $name1, $name2;
                """
                for answer in tx.query.get(query_adjacency):
                    room1 = answer.get("name1").as_attribute().get_value()
                    room2 = answer.get("name2").as_attribute().get_value()
                    self.room_adjacency.append((room1, room2))

                # 2. Fetch Scan Waypoints
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

        ROOM_ADJACENCY = list(self.room_adjacency)
        self.get_logger().info(
            f"Loaded {len(self.room_waypoints)} scan locations and "
            f"{len(self.room_adjacency)} adjacency relations. "
            f"Drop locations will be discovered via perception.")


    def _start_task(self):
        if self._task_started: return
        self._task_started = True
        self.run_task()

    def run_task(self):
        all_scanned_objects = []

        unvisited_rooms = list(self.room_waypoints.keys())
        ordered_rooms = []

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

        self.get_logger().info(f'Optimized scanning sequence: {" -> ".join(ordered_rooms)}')

        for room_name in ordered_rooms:
            wx, wy, wyaw = self.room_waypoints[room_name]
            
            self.get_logger().info(f'Navigating to {room_name}...')
            self._send_nav_goal_and_wait(make_pose_stamped(wx, wy, wyaw))

            # Discover obj via perception, then insert to TypeDB
            self.get_logger().info(f'Scanning {room_name}...')
            object_poses = self._get_objects_in_room()

            for i, pose in enumerate(object_poses):
                obj_id = f'obj_{i}_{room_name}'
                obj = ScannedObject(obj_id, pose, room_name)
                all_scanned_objects.append(obj)

                self.insert_scanned_object(obj_id, room_name, pose.position.x, pose.position.y)
                self.get_logger().info(
                    f'  Found {obj_id} at ({pose.position.x:.2f}, 'f'{pose.position.y:.2f}). Target: [UNKNOWN PENDING PERCEPTION]')
                
            # Discover drop locations via perception, then insert to TypeDB
            perceived_drops = self._get_drop_locations_in_room()

            for drop in perceived_drops:
                drop_type = drop.type.data.strip()   # e.g. "dishwasher", "tableware", ...
                drop_pose = drop.drop_pose
                # Better to preserve semantic IDs instead of inventing generic drop_0_room ids
                drop_id = self._make_drop_id(drop_type)

                # Insert drop location
                self.insert_drop_location(drop_id, room_name, drop_pose.position.x, drop_pose.position.y)

                self.get_logger().info(
                    f'  Found {drop_id} at ({drop_pose.position.x:.2f}, 'f'{drop_pose.position.y:.2f})')


        self.get_logger().info('All rooms scanned. Returning to start room...')
        wx, wy, wyaw = self.room_waypoints.get('kitchen', (0.0, 0.0, 0.0))
        self._send_nav_goal_and_wait(make_pose_stamped(wx, wy, wyaw))

        if not all_scanned_objects:
            self.get_logger().info('No objects found anywhere. Nothing to tidy.')
            return

        self.get_logger().info(
            f'Building PDDL problem with {len(all_scanned_objects)} '
            f'total object(s)...')
        
        # Generar problema simplificado
        problem_str = build_pddl_problem(
            all_scanned_objects, 
            list(self.room_waypoints.keys()),
            self.scan_locations,
            START_ROOM)
        
        self._load_problem_and_trigger_cpp(problem_str)
        self.get_logger().info('Task handed off to executor.')


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