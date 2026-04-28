(define (problem exploration-t3)
  (:domain task3)

  (:objects
    ;; Static rooms and scan locations
    kitchen bedroom living_room office - room
    scan_kitchen scan_bedroom scan_living_room scan_office - location
    
    ;; Dynamic objects injected by Python
    ; PLACEHOLDER: item declarations
    ; PLACEHOLDER: location declarations
  )

  (:init
    ;; Standard initial state
    (free_gripper)
    (agent_at START_LOCATION)

    ;; Dynamic state injected by Python (door status, item positions, etc.)
    ; PLACEHOLDER: predicates
  )

  (:goal (and
    ;; Tactical goal injected by Python (reach the next room)
    ; PLACEHOLDER: goals
  ))
)