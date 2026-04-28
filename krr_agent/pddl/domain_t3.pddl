(define (domain task3)
    (:requirements 
        :strips
        :typing
        :adl
        :negative-preconditions
        :durative-actions
    )
    
    (:types 
        room
        location
        item
    )

    (:predicates
        ;; States of the rooms
        (tidy ?r - room)
        (untidy ?r - room)
        (tidying ?r - room)

        ;; States of the items and gripper
        (on_drop_loc ?i - item)
        (in_gripper ?i - item)
        (free_gripper)
        (busy_gripper)
        (ready_to_place ?i - item)
        (just_picked ?i - item) ;; For inspection purposes, to know which item was just picked up

        ;; Geographical predicates
        (location_in_room ?l - location ?r - room)
        (agent_at ?l - location)
        (scan_loc ?l - location)
        (item_at ?i - item ?l - location)

        ;; ==========================================
        ;; NEW PREDICATES FOR TASK 3
        ;; ==========================================
        (inspected ?i - item)                         ;; True when the robot has picked and identified the item
        (obstacle ?i - item)                          ;; True if the item is an obstacle blocking a doorway
        (standard_item ?i - item)                     ;; True if the item is a regular object that can be inspected but doesn't block doors
        (door_clear ?r1 ?r2 - room)                   ;; True if the passage between r1 and r2 is free of obstacles
        (door_blocked_by ?i - item ?r1 ?r2 - room)    ;; Links an obstacle item to a specific doorway
    )

    ;; 1. Navigate to an object within the current room
    (:durative-action move_to_object
        :parameters (?i - item ?l1 ?l2 - location ?r - room)
        :duration (= ?duration 5)
        :condition (and 
            (at start  (agent_at ?l1))
            (over all  (location_in_room ?l2 ?r))
            (over all  (item_at ?i ?l2))
            (over all  (free_gripper))
            (over all  (tidying ?r))
        )
        :effect (and 
            (at start  (not (agent_at ?l1)))
            (at end    (agent_at ?l2))
        )
    )

    ;; 2. Move to the drop location 
    (:durative-action move_to_drop_location
        :parameters (?i - item ?l_current - location ?r - room)
        :duration (= ?duration 5)
        :condition (and 
            (at start  (agent_at ?l_current))
            (over all  (location_in_room ?l_current ?r))
            (over all  (in_gripper ?i))
            (over all  (busy_gripper))
        )
        :effect (and 
            (at end    (ready_to_place ?i))
            (at start (not (just_picked ?i))) ;; Reset the just_picked state after moving to the drop location
        )
    )

    ;; 3. Pick up a standard object (Acts as an inspection tool!)
    (:durative-action pick
        :parameters (?i - item ?l - location ?r - room)
        :duration (= ?duration 2) ;; Slightly longer to account for perception delay
        :condition (and
            (at start  (free_gripper))
            (at start  (agent_at ?l))
            (over all  (agent_at ?l))
            (at start  (item_at ?i ?l))
            (over all  (location_in_room ?l ?r))
            (over all  (tidying ?r))
            (over all  (standard_item ?i))
        )
        :effect (and
            (at start  (not (free_gripper)))
            (at end    (busy_gripper))
            (at end    (in_gripper ?i))
            (at end    (not (item_at ?i ?l)))
            (at end    (just_picked ?i))
            ;; TASK 3 MAGIC: Picking up the item fulfills the 'inspected' goal
            (at end    (inspected ?i)) 
        )
    )

    ;; 4. Pick up an obstacle blocking a door
    (:durative-action pick_obstacle
        :parameters (?i - item ?l - location ?r1 ?r2 - room)
        :duration (= ?duration 2)
        :condition (and
            (at start  (free_gripper))
            (at start  (agent_at ?l))
            (over all  (agent_at ?l))
            (at start  (door_blocked_by ?i ?r1 ?r2))
            (at start  (item_at ?i ?l))
        )
        :effect (and
            (at start  (not (free_gripper)))
            (at end    (busy_gripper))
            (at end    (in_gripper ?i))
            (at end    (not (item_at ?i ?l)))
            
            ;; Clear the blockage logically (Bidirectional)
            (at end    (not (door_blocked_by ?i ?r1 ?r2)))
            (at end    (door_clear ?r1 ?r2)) 
            (at end    (door_clear ?r2 ?r1)) 
            
            ;; The obstacle is also inspected while being held
            (at end    (inspected ?i))
            (at end (obstacle ?i)) ;; To choose the correct place action
        )
    )

    ;; 5. Place an object back on the ground (to continue exploring)
    (:durative-action place_back
        :parameters (?i - item ?l - location)
        :duration (= ?duration 3)
        :condition (and
            (at start  (in_gripper ?i))
            (at start  (inspected ?i)) ;; Only place back if we actually inspected it
            (over all  (agent_at ?l))
            (at start (obstacle ?i)) ;; Only place back if it's an obstacle (to unblock the door logically
        )
        :effect (and
            (at end    (not (in_gripper ?i)))
            (at end    (free_gripper))
            (at end    (not (busy_gripper)))
            (at end    (item_at ?i ?l)) ;; Item returns to the floor logically

        )
    )

   
    ;; 6. Place the held item in the same place (for inspection, no movement)
    (:durative-action place_inspection
        :parameters (?i - item ?l - location)
        :duration (= ?duration 1)
        :condition (and
            (at start  (in_gripper ?i))
            (at start (just_picked ?i)) ;; Only place if we just picked it up (to prevent re-inspecting the same item multiple times)
            (at start  (inspected ?i)) ;; Only place back if we actually inspected it
            (over all  (agent_at ?l))
            (over all (standard_item ?i)) ;; Standard items can be placed anywhere
        )
        :effect (and
            (at end    (not (in_gripper ?i)))
            (at end    (free_gripper))
            (at end    (not (busy_gripper)))
            (at end    (item_at ?i ?l))
            (at end    (not (just_picked ?i))) ;; Reset the just_picked state after placing
        )
    )

    ;; 7. Place the object in the drop  location

    (:durative-action place
        :parameters (?i - item ?l_current - location)
        :duration (= ?duration 1)
        :condition (and
            (at start  (ready_to_place ?i))
            (at start  (agent_at ?l_current))
            (at start  (in_gripper ?i))
            (over all  (busy_gripper))
        )
        :effect (and
            (at end    (not (in_gripper ?i)))
            (at end    (free_gripper))
            (at end    (not (busy_gripper)))
            (at end    (on_drop_loc ?i))
            (at end    (not (ready_to_place ?i)))
        )
    )

    ;; 8. Switch from one active room to the next (Topology Navigation)
    (:durative-action next_room
        :parameters (?r1 ?r2 - room ?l1 ?l2 - location)
        :duration (= ?duration 10)
        :condition (and 
            (at start  (tidying ?r1))
            (at start  (agent_at ?l1))
            (over all  (free_gripper))
            (over all  (location_in_room ?l1 ?r1))
            (over all  (location_in_room ?l2 ?r2))
            (over all  (scan_loc ?l2))
            ; (at start  (untidy ?r2))
            
            ;; TASK 3 REQUIREMENT: The door must not be blocked!
            (over all  (door_clear ?r1 ?r2))
        )
        :effect (and 
            (at start  (not (agent_at ?l1)))
            (at end    (not (tidying ?r1)))
            (at end    (tidy ?r1))
            (at end    (tidying ?r2))
            (at end    (agent_at ?l2))
            ; (at end    (not (untidy ?r2)))
        )
    )
)