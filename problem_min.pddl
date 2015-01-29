(define (problem network001)
  (:domain network)
  (:objects
      modemdriver-Task
  )
  (:init
      	 (task modemdriver-Task)
;;         (is-running modemdriver-Task)
  )
  (:goal
;;    (valid-running-state)
;;    (not (valid-running-state))
    (valid-running-state task modemdriver-Task)
;;    (task modemdriver-Task)
  )
)
