(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      lights-Lights - instance_req
      TestCmp - instance_req
      mars-IMU - instance_req
      mars-Task - instance_req
  )
  (:init
      (is-running root)
      (depends root root)
      (task lights-Lights)
      (task mars-Task)
      (task mars-IMU)
      (composition TestCmp)
      (requires TestCmp mars-IMU)
      (requires TestCmp mars-Task)
  )
  (:goal (and
      
    (forall (?t - instance_req)
        (imply (is-running ?t)
            (exists (?t2 - instance_req) (depends ?t2 ?t))
        )
    )
      	 (is-running root)
      	 (depends root lights-Lights)
      	 (depends root TestCmp)
  ))
)
