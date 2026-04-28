(define (problem find-book-task3)
  (:domain task3)

  (:objects
    ; PLACEHOLDER: item declarations       (obj_4_book - item  +  any doorway blockers)
    ; PLACEHOLDER: location declarations   (scan_locs + door_locs + safe_locs + delivery_loc)
    ; PLACEHOLDER: room declarations
  )

  (:init
    (free_gripper)
    (agent_at START_LOCATION)

    ; PLACEHOLDER: room_visited for start room   e.g. (room_visited kitchen)
    ; PLACEHOLDER: room_unlocked for rooms reachable without clearing
    ; PLACEHOLDER: room_locked for rooms whose doorway is blocked
    ; PLACEHOLDER: scan_loc predicates
    ; PLACEHOLDER: door_loc predicates
    ; PLACEHOLDER: safe_loc predicates
    ; PLACEHOLDER: delivery_loc predicates
    ; PLACEHOLDER: location_in_room predicates
    ; PLACEHOLDER: item_at predicates       (book + blockers)
    ; PLACEHOLDER: is_book predicate        (is_book obj_4_book)
  )

  (:goal (and
    (book_delivered)
  ))
)
