(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      lights - instance_req
  )
  (:init
      	 (composition root)
      	 (is-running lights)
      	 (depends root root)
      	 (task lights)
  )
  (:goal
  (and

    (is-running root)
    
    (forall (?t - instance_req)
    (exists (?r - instance_req)
            (imply
            (is-running ?t)
            (requests ?r ?t)
            )
            )
            )
  )
  )
)

;call
; pddl_planner_bin -t 600 -p FDAUTOTUNE2 domain.pddl diss-esamples/problem_stop.pddl
;erg:
;Planner FDAUTOTUNE2:
;0   [start_root][stop lights][unit cost)]

