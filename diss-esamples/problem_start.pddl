(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      lights - instance_req
  )
  (:init
      	 (composition root)
      	 (depends root root)
      	 (task lights)
         (depends root lights)
  )
  (:goal
  (and

    (is-running root)
    
    (forall (?t - instance_req)
    (exists (?r - instance_req)
            (imply
            (is-running ?t)
            (requests ?r ?t)
            ;(depends ?r ?t)
            )
            )
            )

  )
  )
)

;call:
;goldhoorn@mgoldhoorn:~/dev-avalon-21/bundles/diss$ pddl_planner_bin -t 600 -p FDAUTOTUNE2 domain.pddl diss-esamples/problem_start.pddl 
/home/goldhoorn/dev-avalon-21/install/bin/fast_downward-planner
;result:
Planner FDAUTOTUNE2:
0   [startt root lights][start_root][unit cost)]


