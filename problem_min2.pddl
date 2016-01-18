(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      lights - instance_req
  )
  (:init
      	 (composition root)
;      	 (is-running root)
;      	 (is-running lights)
      	 (depends root root)
      	 (task lights)
;         (depeneds root lights)
         (depends root lights)
  )
  (:goal
  (and

    (is-running root)
    
;    (forall (?t - instance_req)
;    (exists (?r - instance_req)
;            (imply
;            (is-running ?t)
;            (requests ?r ?t)
;            ;(depends ?r ?t)
;            )
;            )
;            )

;     	 (is-running lights)
    
  )
  )
)
