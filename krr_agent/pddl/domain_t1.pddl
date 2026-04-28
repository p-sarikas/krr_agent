(define (domain task1)
    (:requirements 
        :strips
        :typing
        :adl
        :negative-preconditions
        :durative-actions
    )
    
    ;; multiple type of object in this domain
    (:types 
        room
        location
        item ; the object type already exists in PDDL
    )

    (:predicates
        ;; States of the rooms

        (tidy ?r - room)
        (untidy ?r - room)
        (tidying ?r - room)
   
        (on_drop_loc ?i - item ?l - location)
        (in_gripper ?i - item)

        (free_gripper)
        (busy_gripper)

        ;; Geographical predicates
        (location_in_room ?l - location ?r - room)

        (agent_at ?l - location) ;; This is for the navigation actions, we can have a location type for that

        (scan_loc ?l - location) ;; This is to identify the scan locations in the domain, we can have a location type for that
        (drop_loc ?l - location) ;; This is to identify the drop locations in the domain, we can have a location type for that
        (item_at ?i - item ?l - location) ;; This is to identify the locations of the items in the domain, we can have a location type for that
        (adjacent ?r1 ?r2 - room)
    )


    ;; NavigateToPose: move the robot to a target pose
    (:durative-action move_to_object
        :parameters (?i - item ?l1 ?l2 - location ?r - room)
        :duration (= ?duration 5)
        :condition (and 
            (at start (agent_at ?l1))
            (over all (location_in_room ?l2  ?r))
            (over all (item_at ?i ?l2))
            (over all (free_gripper))
            (over all (tidying ?r))
        )
        :effect (and 
            (at start (not (agent_at ?l1)))
            (at end (agent_at ?l2))
        )
    )
    

    (:durative-action move_to_drop_location
        :parameters (?i - item ?l1 ?l2 - location ?r - room)
        :duration (= ?duration 5)
        :condition (and 
            (at start (agent_at ?l1))

            (over all (location_in_room ?l2  ?r))
            (over all (drop_loc ?l2))
            (over all (in_gripper ?i))
            (over all (tidying ?r))
            (over all (busy_gripper))
        )
        :effect (and 
            (at start (not (agent_at ?l1)))
            (at end (agent_at ?l2))
        )
    )


    (:durative-action next_room
    :parameters (?r1 ?r2 - room ?l1 ?l2 - location)
    :duration (= ?duration 20)
    :condition (and 
            (at start (tidying ?r1))
            (at start (agent_at ?l1))
            (at start (adjacent ?r1 ?r2))
            (over all (free_gripper))

            (over all (location_in_room ?l1 ?r1))
            (over all (location_in_room ?l2 ?r2))

            (over all (scan_loc ?l2))
            (at start (untidy ?r2))
        )
    :effect (and 
            (at start (not (agent_at ?l1)))

            (at end (not (tidying ?r1)))
            (at end (tidy ?r1))

            (at end (tidying ?r2))
            (at end (agent_at ?l2))
            (at end (not (untidy ?r2)))
        )
    )
    

    (:durative-action pick
        :parameters (?i - item ?l - location ?r - room)
        :duration ( = ?duration 1)
        :condition (and
            (at start (free_gripper))
            (at start (agent_at ?l))
            (at end (agent_at ?l))
            (over all (agent_at ?l))
            (at start (item_at ?i ?l))
            (over all (item_at ?i ?l))
            (over all (location_in_room ?l ?r))
            (over all (tidying ?r))
        )
        :effect (and
            (at start (not (free_gripper)))
            (at end (busy_gripper))
            (at end (in_gripper ?i))
            (at end (not (item_at ?i ?l)))
        )
    )


    (:durative-action place
        :parameters (?i - item ?l - location ?r - room)
        :duration ( = ?duration 1)
        :condition (and
            (at start (in_gripper ?i))
            (over all (in_gripper ?i))

            (over all (location_in_room ?l ?r))
            (over all (tidying ?r))

            (at start (drop_loc ?l))
            (over all (drop_loc ?l))
            (at end (drop_loc ?l))
            
            (at start (busy_gripper))
            (over all (busy_gripper))

            (at start (agent_at ?l))
            (at end (agent_at ?l))
            (over all (agent_at ?l))
        )
        :effect (and
            (at end (not (in_gripper ?i)))
            (at end (free_gripper))
            (at end (not (busy_gripper)))
            (at end (on_drop_loc ?i ?l))
        )
    )

)