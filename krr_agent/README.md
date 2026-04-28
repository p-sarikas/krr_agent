# krr_agent
Solution to the KRR final project — RoboCup@Home-inspired Tasks

**Group 5** · TU Delft — Carlota Alvear Llorente · Núria Seguí Vidal · Panagiotis Sarikas

- [Container Images](#container-images)
- [Workspace Setup](#workspace-setup)
- [Running the Tasks](#running-the-tasks)
- [krr_agent package](#krr_agent-1)
- [Repository Structure](#repository-structure)
- [References](#references)

---

## Container Images

| Description | Image | Default Command |
| --- | --- | --- |
| ROS 2 Humble + Gazebo + Nav2 + PlanSys2 + TypeDB environment | `ro47014_humble_<vX>.sif` | `singularity shell` |

> Replace `<vX>` with the version of the image you have (e.g., `v3`).

---

## Workspace Setup

Create a ROS 2 workspace named **`krr_ws`** and clone this package inside it:

```bash
mkdir -p ~/krr_ws/src
cd ~/krr_ws/src
git clone git@gitlab.tudelft.nl:cor/ro47014/students-2526/group_05/krr_agent.git
cd ~/krr_ws
colcon build
```

---

## Running the Tasks

All tasks implementations require **two terminals**, inside the Singularity container.

Firstly, a terminal to lauch the typeDB server is needed.

```bash
# Replace <PATH> with the path to your .sif image and <vX> with the version (e.g., v3)
singularity shell -B $XAUTHORITY:$XAUTHORITY -p <PATH>/ro47014_humble_<vX>.sif

# Source the course base underlay
source /krr/krr_base_ws/install/setup.bash

# Source your workspace overlay (replace <your_ws> with your workspace name, e.g. krr_ws)
source ~/<your_ws>/install/setup.bash

# Launch the TypeDB server
typedb server --storage.data ~/<your_ws>/src/krr_agent/typedb_data

```

And a second terminal to launch the implementation (*always launch this second*).
```bash
# Replace <PATH> with the path to your .sif image and <vX> with the version (e.g., v3)
singularity shell -B $XAUTHORITY:$XAUTHORITY -p <PATH>/ro47014_humble_<vX>.sif

# Source the course base underlay
source /krr/krr_base_ws/install/setup.bash

# Source your workspace overlay (replace <your_ws> with your workspace name, e.g. krr_ws)
source ~/<your_ws>/install/setup.bash

# Launch the desired task
ros2 launch krr_agent task<Y>.launch.xml   # Y = 1, 2, or 3

```



#### Task 1 — Simple Tidy Up
Places each object in the nearest drop location (Euclidean distance) within the same room.
```bash
ros2 launch krr_agent task1.launch.xml
```

#### Task 2 — Semantic Tidy Up · Multiple PDDL *(default)*
Room-by-room planning; drop locations resolved via TypeDB semantic rules. Faster and more scalable.
```bash
ros2 launch krr_agent task2.launch.xml
```

#### Task 2 — Semantic Tidy Up · Single PDDL
Full apartment-level world model built before planning. Use when drop location positions cannot be assumed known a priori.
```bash
ros2 launch krr_agent task2.launch.xml manager_script:=task2_manager.py
```

#### Task 3 — Find and Bring
Locates a target book via TypeDB contextual inference (`book-clue` relation), clears doorway obstacles, then semantically tidies remaining objects.
```bash
ros2 launch krr_agent task3.launch.xml
```

---

## `krr_agent package`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/task_status` | `std_msgs/String` | Task Manager receives plan execution result (`SUCCESS` / `FAILED`) from the Task Controller |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/task_status` | `std_msgs/String` | Task Controller publishes plan execution result after each planning cycle |
| `/cmd_vel` | `geometry_msgs/Twist` | `PlaceBack` action publishes rotation commands to rotate the robot 90° away from a blocked doorway before releasing the obstacle |

### Services

| Service | Type | Description |
| --- | --- | --- |
| `/start_task_planning` | `std_srvs/Trigger` | Task Manager calls this to trigger plan execution in the Task Controller |
| `/get_objects_in_room` | `krr_mirte_skills_msgs/GetObjectsInRoom` | Scans the current room and returns detected object IDs and poses |
| `/get_drop_locations` | `krr_mirte_skills_msgs/GetDropLocations` | Returns known drop location IDs and poses (Tasks 1 & 2 Single PDDL) |
| `/get_object_info` | `krr_mirte_skills_msgs/GetObjectInfo` | Identifies the type and attributes of the currently held object (used in `pick` action for knowledge refinement) |
| `/pick_object` | `krr_mirte_skills_msgs/PickObject` | Executes the physical grasp via Mirte skills |
| `/place_object` | `krr_mirte_skills_msgs/PlaceObject` | Executes the physical place via Mirte skills |
| `/ros_typedb/query` | `ros_typedb_msgs/Query` | Read / write queries to the TypeDB knowledge base |
| `/problem_expert/add_problem` | `plansys2_msgs/AddProblem` | Loads a PDDL problem into PlanSys2 |
| `/problem_expert/clear_problem_knowledge` | `plansys2_msgs/ClearProblemKnowledge` | Clears the current PDDL problem state before loading a new one |

### Actions

| Action | Type | Description |
| --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Sends navigation goals to Nav2; used by all move action nodes and the Task Manager |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `plan_solver_plugins` | `string[]` | `["POPF"]` | Active planner plugin(s) for PlanSys2 |
| `plan_solver_timeout` | `double` | `400.0` | Maximum time (seconds) allowed for the planner to find a solution |
| `database_name` | `string` | `task_db` | Name of the TypeDB database |
| `address` | `string` | `localhost:1729` | TypeDB server address |
| `action_name` | `string` | *(per node)* | PDDL action name mapped to each C++ action executor node |
| `infer` | `bool` | `true` | Enables TypeDB rule inference (e.g., `correct-drop`, `book-clue` relations) |

---

## Repository Structure

```
krr_agent/
├── config/
│   └── plansys2_params.yaml
├── include/krr_agent/
│   ├── action_move_to_drop_location_t1.hpp
│   ├── action_move_to_drop_location_t2.hpp
│   ├── action_move_to_object.hpp
│   ├── action_next_room.hpp
│   ├── action_pick.hpp
│   ├── action_place_back.hpp
│   ├── action_place.hpp
│   ├── krr_agent.hpp
│   └── task_controller.hpp
├── launch/
│   ├── task1.launch.py
│   ├── task1.launch.xml
│   ├── task2.launch.py
│   ├── task2.launch.xml
│   ├── task3.launch.py
│   └── task3.launch.xml
├── pddl/
│   ├── domain_t1.pddl
│   ├── domain_t2.pddl
│   ├── domain_t3.pddl
│   ├── problem_dummy.pddl
│   ├── problem_dummy_t3.pddl
│   ├── problem_exploration_template.pddl
│   ├── problem_t1_template.pddl
│   ├── problem_t2_template.pddl
│   ├── problem_t3_template.pddl
├── scripts/
│   ├── data.tql
│   ├── schema.tql
│   ├── setup_database.py
│   ├── task_manager_base.py
│   ├── task1_manager.py
│   ├── task2_manager.py
│   ├── task2_multiple_pddl_manager.py
│   └── task3_manager.py
├── src/
│   ├── action_move_to_drop_location_t1.cpp
│   ├── action_move_to_drop_location_t2.cpp
│   ├── action_move_to_object.cpp
│   ├── action_next_room.cpp
│   ├── action_pick.cpp
│   ├── action_place_back.cpp
│   ├── action_place.cpp
│   ├── krr_agent.cpp
│   ├── set_initial_pose.cpp
│   └── task_controller.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## References

1. F. Martín et al., "PlanSys2: A Planning System Framework for ROS2," *IROS 2021*.
2. A. Lindsay, "On Using Action Inheritance and Modularity in PDDL Domain Modelling," *ICAPS 2023*.
3. Y. Carreno et al., "Towards Long-Term Autonomy Based on Temporal Planning," *TAROS 2019*.
4. S. Macenski et al., "The Marathon 2: A Navigation System," *IROS 2020*.
5. S. Izquierdo-Badiola et al., "Planning for Human-Robot Collaboration Scenarios with Heterogeneous Costs and Durations," *ECAI 2024*.