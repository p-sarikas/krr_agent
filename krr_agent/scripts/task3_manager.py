#!/usr/bin/env python3

import os
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from typedb.driver import SessionType, TransactionType
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

from typedb.driver import TypeDBOptions

# Assuming task_manager_base.py is in the same directory or PYTHONPATH
from task_manager_base import TaskManagerBase, euclidean_distance, make_pose_stamped

# ---------------------------------------------------------------------------
# CONSTANTS & CONFIG
# ---------------------------------------------------------------------------

START_ROOM = 'kitchen'
TARGET_ITEM_TYPE = 'book' 
pkg_share_dir = get_package_share_directory('krr_agent')
EXPLORATION_TEMPLATE_PATH = os.path.join(pkg_share_dir, 'pddl', 'problem_exploration_template.pddl')

class ScannedObject:
    def __init__(self, entity_id: str, pose, room: str, is_obstacle: bool = False):
        self.entity_id = entity_id
        self.pose = pose
        self.room = room
        self.is_obstacle = is_obstacle

# ---------------------------------------------------------------------------
# MAIN NODE
# ---------------------------------------------------------------------------

class TaskManagerNode(TaskManagerBase):

    def __init__(self):
        # Pass configuration to parent; parent handles all service/action clients
        super().__init__(
            node_name='task_manager_node',
            enable_nav=True,
            enable_drop_locations=False,
            clear_problem_knowledge=True,
            wait_for_task_status=True,
        )
        self.get_logger().info('[INIT] Task 3 Cognitive Manager started.')
        self._task_started = False
        
        # Trigger the mission loop
        self.create_timer(2.0, self._start_mission, callback_group=self.cb_group)

    # ------------------------------------------------------------------
    # TYPEDB REASONING (Task 3 Specific)
    # ------------------------------------------------------------------

    def _load_knowledge_from_db(self):
        """Loads rooms, adjacencies, and door statuses."""
        self.room_waypoints = {}
        self.scan_locations = {}
        self.room_map = {}
        self.door_status = {} 

        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                # 1. Waypoints
                query_scans = "match $rm isa room, has room-name $r_name; $scan isa scan-location; $p isa pose, has pos-x $x, has pos-y $y; (located-target: $scan, location: $p) isa physical-location; (container: $rm, contained-pose: $p) isa spatial-containment; get $r_name, $x, $y;"
                for answer in tx.query.get(query_scans):
                    room = answer.get("r_name").as_attribute().get_value()
                    self.room_waypoints[room] = (answer.get("x").as_attribute().get_value(), answer.get("y").as_attribute().get_value(), 0.0)
                    self.scan_locations[room] = f"scan_{room}"

                # 2. Adjacency
                query_adj = "match $ra isa room, has room-name $name_a; $rb isa room, has room-name $name_b; (room-a: $ra, room-b: $rb) isa adjacent; get $name_a, $name_b;"
                for answer in tx.query.get(query_adj):
                    a = answer.get("name_a").as_attribute().get_value()
                    b = answer.get("name_b").as_attribute().get_value()
                    self.room_map.setdefault(a, []).append(b)
                    self.room_map.setdefault(b, []).append(a)
                    self.door_status[(a, b)] = 'blocked'

    def check_if_book_found(self):
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """match $obj isa item, has id $id; 
                            $pose isa pose, has pos-x $x, has pos-y $y; 
                            (located-item: $obj, location: $pose) isa physical-location; 
                            get $obj, $id, $x, $y;"""
                for answer in tx.query.get(query):
                    obj_type = answer.get("obj").get_type().get_label().name
                    obj_id = answer.get("id").as_attribute().get_value()
                    if obj_type == TARGET_ITEM_TYPE:
                        return True, obj_id, (answer.get("x").as_attribute().get_value(), answer.get("y").as_attribute().get_value())
        return False,None, None

    
    def objects_to_tidy(self) -> list:
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = """match $obj isa item, has id $id, has handled $h; 
                            $pose isa pose, has pos-x $x, has pos-y $y; 
                            (located-item: $obj, location: $pose) isa physical-location; 
                            get $obj, $id, $x, $y, $h;"""
                items_found = []
                for answer in tx.query.get(query):
                    obj_type = answer.get("obj").get_type().get_label().name
                    obj_id = answer.get("id").as_attribute().get_value()

                    # Exclude by type 
                    if obj_type == TARGET_ITEM_TYPE:
                        continue

                    # Exclude if handled (i.e. already tidied)
                    handled = answer.get("h").as_attribute().get_value()
                    if handled:
                        self.get_logger().info(f"Excluding {obj_id} from tidying list because it is already handled.")
                        continue

                    room_name = "unknown"
                    for r in self.room_map.keys():
                        if r in obj_id:
                            room_name = r
                            break

                    items_found.append(ScannedObject(obj_id, None, room_name))
        return items_found


    def likely_room(self, unvisited_rooms: list) -> str:
        """Uses book-clue relation to prioritize rooms."""
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as tx:
                query = f'match $b isa book, has id "obj_4_book"; $r isa room, has room-name $r_name; (target-book: $b, likely-room: $r) isa book-clue; get $r_name;'
                for answer in tx.query.get(query):
                    room_name = answer.get("r_name").as_attribute().get_value()
                    if room_name in unvisited_rooms:
                        return room_name, True
        return 'office' if 'office' in unvisited_rooms else unvisited_rooms[0], False


    # We query TypeDB for the room where the target drop location for a specific item is
    def drop_loc_target(self, item_id: str) -> str:
        room_name = "unknown"

        options = TypeDBOptions()
        options.infer = True
        
        with self.db_driver.session(self.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ, options) as tx:
                query = f"""
                    match
                        $obj isa item, has id "{item_id}";
                        (dropped-item: $obj, target-location: $d) isa correct-drop;
                        (located-target: $d, location: $p) isa physical-location;
                        (container: $room, contained-pose: $p) isa spatial-containment;
                        $room has room-name $r_name;
                    get $r_name;
                """
                for answer in tx.query.get(query):
                    room_name = answer.get("r_name").as_attribute().get_value()
                    break  # Found the room, no need to keep looping
                    
        return room_name

    def _process_doorway_obstacles(self, perception_result, current_room: str, next_room: str) -> list:
        room_obstacles = []

        # 1. Prepare a dictionary with the actual adjacent rooms to the current one
        adjacent_rooms = self.room_map.get(current_room, [])
        obstacles_by_door = {adj: [] for adj in adjacent_rooms}

        if hasattr(perception_result, 'doorway_object_poses'):
            doorway_groups = perception_result.doorway_object_poses
            global_obs_idx = 0 

            for dg in doorway_groups:
                doorway_str = dg.which_doorway.data  # e.g., 'kitchen_to_living'

                # 2. SMART MATCHING (Ignoring '_room')
                target_room = None
                for adj in adjacent_rooms:
                    adj_base = adj.replace('_room', '')
                    curr_base = current_room.replace('_room', '')
                    
                    if adj_base in doorway_str and curr_base in doorway_str:
                        target_room = adj
                        break

                if not target_room:
                    self.get_logger().warn(f"Could not match doorway '{doorway_str}' to any adjacent room!")
                    continue

                # 3. Process the obstacles for THIS specific door
                for pose in dg.objects_in_doorway:
                    # Check if an object already exists at these coordinates
                    existing_id = self.get_known_object_at(pose.position.x, pose.position.y)
                    
                    if existing_id:
                        obs_id = existing_id
                        self.get_logger().info(f"Vision: Reusing known obstacle ID: {obs_id}")
                    else:
                        # Only create a new ID and save to TypeDB if it's genuinely new
                        obs_id = f"obs_door_{global_obs_idx}_{current_room}"
                        global_obs_idx += 1 
                        self.insert_scanned_object(obs_id, current_room, pose.position.x, pose.position.y)
                    
                    obs_obj = ScannedObject(obs_id, pose, current_room, is_obstacle=True)
                    obstacles_by_door[target_room].append(obs_obj)

        # 4. APPLY NEGATIVE PERCEPTION AND EXTRACT OBSTACLES
        for adj_room, obs_list in obstacles_by_door.items():
            if len(obs_list) == 0:
                # No obstacles in the Gazebo list! The corridor is genuinely safe.
                self.door_status[(current_room, adj_room)] = 'clear'
                self.get_logger().info(f"Vision verified: Door from {current_room} to {adj_room} is CLEAR.")
            else:
                self.get_logger().warn(f"Vision verified: {len(obs_list)} obstacle(s) blocking door to {adj_room}.")
                # Only return to Phase 1 the ones blocking our immediate path
                if adj_room == next_room:
                    room_obstacles.extend(obs_list)

        return room_obstacles
    # ------------------------------------------------------------------
    # MISSION EXECUTION
    # ------------------------------------------------------------------

    def _start_mission(self):
        if self._task_started: return
        self._task_started = True
        
        # ==========================================
        # PHASE 1: FIND AND TIDY THE BOOK
        # ==========================================
        # Unvisited : all except start room
        unvisited = [r for r in self.room_waypoints.keys() if r != START_ROOM]
            
        current_room = START_ROOM
        all_objects_to_tidy = []
        book_found = False
        book_returned = False
        finish_phase_1 = False

        while not finish_phase_1:
            self.get_logger().info(f"[PHASE 1] Investigating: {current_room}")
            self.get_logger().info(f'Scanning {current_room}...')

            object_poses = self._get_objects_in_room()

            self.get_logger().info(f'Found {len(object_poses)} object(s).')
            
            current_room_objects = []

            room_drops = self.get_drop_locations_in_room(current_room)
            
            for i, pose in enumerate(object_poses):
                obj_id = f'obj_{i}_{current_room}'

                # Check if this object is already known in TypeDB (e.g., from a previous scan or because it's on a drop location)
                existing_id = self.get_known_object_at(pose.position.x, pose.position.y)
                if existing_id:
                    continue  # Skip processing this object since it's already known

                # Filtering out objects that are already on drops in this room (if any): book!
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
                    obj_id, current_room, pose.position.x, pose.position.y, handled=is_on_drop)

                if not is_on_drop:
                    obj = ScannedObject(obj_id, pose, current_room)
                    current_room_objects.append(obj)
                    self.get_logger().info(
                        f'  Found {obj_id} at ({pose.position.x:.2f}, '
                        f'{pose.position.y:.2f}). Target: [UNKNOWN PENDING PERCEPTION]')
                    
            # 2. CHECK OBJECTS (Inspect) if there are any, otherwise just move on
            if current_room_objects and not book_found:
                self.get_logger().info(f'Building PDDL problem for exploration of {current_room}...')
                problem_str = self.build_exploration_pddl(current_room, current_room_objects) 
                self.get_logger().info('Executing PDDL problem...')
                
                if self._execute_phase_pddl(problem_str):
                    self.get_logger().info(f"{current_room} inspected successfully.")
                    # Check in typeDB if any of the found objects is the target item 
                    book_found, book_id, book_pose = self.check_if_book_found()
                else:
                    self.get_logger().error(f"Failed to inspect {current_room}. Stopping mission.")
                    break
            else:
                self.get_logger().info(f"No objects found in {current_room}. Moving on.")

            
            # If book found, build a problem to return it
            if book_found and not book_returned:
                self.get_logger().info(f"Book found at ({book_pose[0]:.2f}, {book_pose[1]:.2f}).")
                self.get_logger().info(f'Building PDDL problem for returning the book...')
                problem_str = self.build_task2_pddl_problem([ScannedObject(book_id, None, current_room)], current_room, True)
                
                if self._execute_phase_pddl(problem_str):
                    self.get_logger().info(f"Book returned successfully. Part 1 complete!")

                    book_returned = True

                    # Where is my robot? We query for the drop position of the target
                    agent_room = self.drop_loc_target(book_id)

                    if not unvisited: 
                        self.get_logger().info(f"All rooms visited, book found = {book_found}. Ending Phase 1.")
                        finish_phase_1 = True
                        next_room = START_ROOM # to clear the last doorway if needed
                    else:
                        next_room, clues = self.likely_room(unvisited)

                    if agent_room == next_room:
                        self.get_logger().info(f"Book dropped in {agent_room}, which is our next target! Staying here.")
                        
                        # Re-orient in the center of the room
                        nav_pose = PoseStamped()
                        nav_pose.header.frame_id = "map"
                        nav_pose.pose.position.x = self.room_waypoints[agent_room][0]
                        nav_pose.pose.position.y = self.room_waypoints[agent_room][1]
                        nav_pose.pose.position.z = self.room_waypoints[agent_room][2]
                        nav_pose.pose.orientation.w = 1.0
                        self._send_nav_goal_and_wait(nav_pose)
 
                        current_room = next_room
                        
                        # Mark it as visited so we don't scan it twice
                        if current_room in unvisited:
                            unvisited.remove(current_room)
                            
                        continue # Skip the rest of the loop and start processing this new room immediately
                    
                    elif agent_room == "unknown":
                        # If there was an error and the room is unkown: go back to office with no PDDL
                        self.get_logger().info(f"Book in unknown room! Going back to the office")
                        nav_pose = PoseStamped()
                        nav_pose.header.frame_id = "map"
                        nav_pose.pose.position.x = self.room_waypoints[current_room][0]
                        nav_pose.pose.position.y = self.room_waypoints[current_room][1]
                        nav_pose.pose.position.z = self.room_waypoints[current_room][2]
                        nav_pose.pose.orientation.w = 1.0
                        self._send_nav_goal_and_wait(nav_pose)

                    else:
                        # 4. The condition is the other way around. Go back to where we found the object!
                        self.get_logger().info(f"Book dropped in {agent_room}. Navigating back to {current_room} to resume search pattern...")
                        
                        # A. Check for obstacles on the way back
                        obs = self._get_obstacles_in_room()
                        return_obstacles = self._process_doorway_obstacles(obs, agent_room, current_room)

                        # B. Build the PDDL move problem
                        problem_str = self.build_new_room_pddl(agent_room, current_room, return_obstacles)
                        
                        # C. Execute the move
                        if self._execute_phase_pddl(problem_str):
                            self.get_logger().info(f"Successfully returned to {current_room}.")
                            self.door_status[(agent_room, current_room)] = 'clear'
                        else:
                            self.get_logger().error(f"Failed to return to {current_room}. Stopping mission.")
                            break


                    # If it corresponds with the next unvisited room: we stay there
                else:
                    self.get_logger().error(f"Failed to tidy book.")


            # 3. CHOOSE NEXT NEIGHBOR
            if not unvisited: 
                self.get_logger().info(f"All rooms visited, book found = {book_found}. Ending Phase 1.")
                finish_phase_1 = True
                next_room = START_ROOM # to clear the last doorway if needed
            else:
                next_room, clues = self.likely_room(unvisited)
                unvisited.remove(next_room)

                # If the room is not adjacent and we are only going there because of lack of clues:
                if next_room not in self.room_map[current_room] and not clues:
                    routed = False
                    for intermediate in self.room_map[current_room]:
                        if next_room in self.room_map[intermediate] and intermediate in unvisited:
                            self.get_logger().info(
                                f"{current_room} → {next_room} not direct. Routing via {intermediate}."
                            )
                            unvisited.append(next_room)   # ← put original target back
                            unvisited.remove(intermediate) # ← only consume the intermediate
                            next_room = intermediate
                            routed = True
                            break
                    if not routed:
                        # No unvisited intermediate available — go direct anyway
                        self.get_logger().warn(
                            f"No unvisited intermediate to reach {next_room}. Going direct."
                        )


            # 4. HANDLE OBSTACLES FOR THIS TRANSITION
            obs = self._get_obstacles_in_room()
            room_obstacles = self._process_doorway_obstacles(obs, current_room, next_room)

            # 5. EXECUTE MOVE
            self.get_logger().info(f"[PHASE 1] Moving: {current_room} -> {next_room}")
            problem_str = self.build_new_room_pddl(current_room, next_room, room_obstacles)
            
            if self._execute_phase_pddl(problem_str):
                self.get_logger().info(f"Moved to {next_room} successfully.")
                # That door is clear now
                self.door_status[(current_room, next_room)] = 'clear'
                current_room = next_room
            else:
                self.get_logger().error(f"Failed to move to {next_room}. Stopping mission.")
                break


        # ==========================================
        # PHASE 2: TIDY UP
        # ==========================================

        # Query TypeDB for all discovered objects that are still untidy (i.e. exist in a room but not on a drop location)
        self.get_logger().info(f"[PHASE 2] Excluding known book IDs")
        tries = 3
        
        while True:
            all_objects_to_tidy = self.objects_to_tidy()

            if not all_objects_to_tidy:
                self.get_logger().info("No objects left to tidy. Mission complete!")
                break

            self.get_logger().info(f"[PHASE 2] Items remaining: {len(all_objects_to_tidy)}")

            # Build and execute plan for remaining items
            problem_str = self.build_task2_pddl_problem(all_objects_to_tidy, current_room, False)
            success = self._execute_phase_pddl(problem_str)

            if success:
                self.get_logger().info("Plan step successful.")
                # We don't break; we loop to double-check if anything else appeared or was missed
            else:
                if tries <= 0:
                    self.get_logger().error("Max retries reached. Some items could not be tidied.")
                    break
                
                # RECOVERY STEP: If pick failed, rescan the room and update DB
                self._recover_and_update_poses(current_room)
                tries -= 1
                self.get_logger().info(f"Retrying Phase 2... ({tries} retries left)")


    # ------------------------------------------------------------------
    # PDDL BUILDERS
    # ------------------------------------------------------------------

    def build_exploration_pddl(self, current_room: str, objects_to_inspect: list) -> str:
        item_decls, loc_decls, preds, goals = "", "", "", ""
        preds += f"    (scan_loc scan_{current_room})\n    (location_in_room scan_{current_room} {current_room})\n    (tidying {current_room})\n"

        for ob in objects_to_inspect:
            item_decls += f"    {ob.entity_id} - item\n"
            loc_decls += f"    loc_{ob.entity_id} - location\n"
            preds += f"    (location_in_room loc_{ob.entity_id} {current_room})\n"
            preds += f"    (item_at {ob.entity_id} loc_{ob.entity_id})\n"
            preds += f"    (standard_item {ob.entity_id})\n"
            goals += f"    (inspected {ob.entity_id})\n"
        
        goals += f"    (free_gripper)\n"

        with open(EXPLORATION_TEMPLATE_PATH, 'r') as f: content = f.read()
        content = content.replace('START_LOCATION', f'scan_{current_room}')
        content = content.replace('; PLACEHOLDER: item declarations', item_decls)
        content = content.replace('; PLACEHOLDER: location declarations', loc_decls)
        content = content.replace('; PLACEHOLDER: predicates', preds)
        content = content.replace('; PLACEHOLDER: goals', goals)
        return content
    
    def build_new_room_pddl(self, current_room: str, next_room: str, obstacles: list) -> str:
        item_decls, loc_decls, preds = "", "", ""
        preds += f"    (scan_loc scan_{current_room})\n    (location_in_room scan_{current_room} {current_room})\n    (tidying {current_room})\n"
        preds += f"    (scan_loc scan_{next_room})\n    (location_in_room scan_{next_room} {next_room})\n    (untidy {next_room})\n"
        
        if not obstacles:
            preds += f"    (door_clear {current_room} {next_room})\n    (door_clear {next_room} {current_room})\n"
        else:
            for obs in obstacles:
                item_decls += f"    {obs.entity_id} - item\n"
                loc_decls += f"    loc_{obs.entity_id} - location\n"
                preds += f"    (location_in_room loc_{obs.entity_id} {current_room})\n"
                preds += f"    (item_at {obs.entity_id} loc_{obs.entity_id})\n"
                preds += f"    (obstacle {obs.entity_id})\n"
                preds += f"    (door_blocked_by {obs.entity_id} {current_room} {next_room})\n"
            
        with open(EXPLORATION_TEMPLATE_PATH, 'r') as f: content = f.read()
        content = content.replace('START_LOCATION', f'scan_{current_room}')
        content = content.replace('; PLACEHOLDER: item declarations', item_decls)
        content = content.replace('; PLACEHOLDER: location declarations', loc_decls)
        content = content.replace('; PLACEHOLDER: predicates', preds)
        content = content.replace('; PLACEHOLDER: goals', f"(agent_at scan_{next_room})")
        return content

    def build_task2_pddl_problem(self, objects: list, start_room: str, tidying_book: bool) -> str:
        item_decls = '\n'.join(f'    {o.entity_id} - item' for o in objects)
        loc_decls = '\n'.join(f'    loc_{o.entity_id} - location' for o in objects)
        for r, sl in self.scan_locations.items(): loc_decls += f"\n    {sl} - location"
        
        preds = f"    (tidying {start_room})\n"
        preds += '\n'.join(f'    (scan_loc {sl})' for sl in self.scan_locations.values())
        preds += '\n'.join(f'    (location_in_room {sl} {r})' for r, sl in self.scan_locations.items())
        
        # Add door predicates: (door_clear ?r1 ?r2 - room)
        clear_adj = {r: set() for r in self.room_map.keys()}
        for key, status in self.door_status.items():
            if status == 'clear':
                r1, r2 = key
                clear_adj[r1].add(r2)
                clear_adj[r2].add(r1)
        
        # We insert door_clear predicates for all the known clear doors:
        for r1, neighbors in clear_adj.items():
            for r2 in neighbors:
                preds += f"    (door_clear {r1} {r2})\n"
                preds += f"    (door_clear {r2} {r1})\n"

        if tidying_book:
            # All rooms are untidy
            for r in self.room_map.keys():
                preds += f"    (untidy {r})\n"
        else:
            # Tidy rooms if there are no objects in them
            for r in self.room_map.keys():
                if r == start_room: 
                    preds += f"    (untidy {r})\n"
                elif not any(o.room == r for o in objects):
                    preds += f"    (tidy {r})\n"
                else:
                    preds += f"    (untidy {r})\n"
                
        for o in objects:
            preds += f"    (location_in_room loc_{o.entity_id} {o.room})\n"
            preds += f"    (item_at {o.entity_id} loc_{o.entity_id})\n"
            preds += f"    (standard_item {o.entity_id})\n"
        
        goals = '\n'.join(f'    (on_drop_loc {o.entity_id})' for o in objects)

        # if not tidying_book:
        #     goals += '\n'.join(f'\n    (tidy {r})' for r in self.room_map.keys())

        with open(EXPLORATION_TEMPLATE_PATH, 'r') as f: content = f.read()
        content = content.replace('START_LOCATION', f'scan_{start_room}')
        content = content.replace('; PLACEHOLDER: item declarations', item_decls)
        content = content.replace('; PLACEHOLDER: location declarations', loc_decls)
        content = content.replace('; PLACEHOLDER: predicates', preds)
        content = content.replace('; PLACEHOLDER: goals', goals)
        return content


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

if __name__ == '__main__': main()