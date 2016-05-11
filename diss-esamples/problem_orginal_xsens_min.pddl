(define (problem network001)
  (:domain network)
  (:objects
        root - instance_req
        XsensImu::Task - instance_req
  )
  (:init
        (composition  root)
        (task  XsensImu::Task)
; Fix encoded knoeledge
            (is-root root)
            (depends root XsensImu::Task)

; Begin requirements

  )
  (:goal (and


    (forall (?t - instance_req)
    (exists (?r - instance_req)
            (imply
            (is-running ?t)
            (requests ?r ?t)
            )
            )
            )
    
    (is-running root)
  
  ))
)
