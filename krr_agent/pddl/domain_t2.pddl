(define (domain task2)
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
        (tidy ?r - room)
        (untidy ?r - room)
        (tidying ?r - room)

        (on_drop_loc ?i - item)
        (in_gripper ?i - item)
        (free_gripper)
        (busy_gripper)
        
        ;; NUEVO: Puente lógico entre moverse y soltar
        (ready_to_place ?i - item)

        (location_in_room ?l - location ?r - room)
        (agent_at ?l - location)

        (scan_loc ?l - location)
        (item_at ?i - item ?l - location)

        (adjacent ?r1 ?r2 - room)
    )

    ;; Navigate to an item
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

    ;; Pick up an item
    (:durative-action pick
        :parameters (?i - item ?l - location ?r - room)
        :duration (= ?duration 1)
        :condition (and
            (at start  (free_gripper))
            (at start  (agent_at ?l))
            (over all  (agent_at ?l))
            (at start  (item_at ?i ?l))
            (over all  (item_at ?i ?l))
            (over all  (location_in_room ?l ?r))
            (over all  (tidying ?r))
        )
        :effect (and
            (at start  (not (free_gripper)))
            (at end    (busy_gripper))
            (at end    (in_gripper ?i))
            (at end    (not (item_at ?i ?l)))
        )
    )

    ;; TASK 2 CHANGE: Move to drop location logically
    ;; El C++ se encarga de todo el viaje. Lógicamente no cambiamos agent_at.
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
            ;; We’ve completed the journey and we’re ready to let go
            (at end    (ready_to_place ?i))
        )
    )

    ;; Place the held item
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

    ;; switch from one active room to the next
    (:durative-action next_room
        :parameters (?r1 ?r2 - room ?l1 ?l2 - location)
        :duration (= ?duration 10)
        :condition (and 
            (at start  (tidying ?r1))
            (at start  (agent_at ?l1))
            (at start  (adjacent ?r1 ?r2))
            (over all  (free_gripper))
            (over all  (location_in_room ?l1 ?r1))
            (over all  (location_in_room ?l2 ?r2))
            (over all  (scan_loc ?l2))
            ; (at start  (untidy ?r2))
        )
        :effect (and 
            (at start  (not (agent_at ?l1)))
            (at end    (not (tidying ?r1)))
            (at end    (tidy ?r1))
            (at end    (tidying ?r2))
            (at end    (agent_at ?l2))
            (at end    (not (untidy ?r2)))
        )
    )
)