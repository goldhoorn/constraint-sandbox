(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      shouldStopped - instance_req
      shouldRunning - instance_req
      cmpChild21 - instance_req
      cmpChild22 - instance_req
      TestCmp2 - instance_req
      cmpChild1 - instance_req
      cmpChild2 - instance_req
      TestCmp - instance_req
  )
  (:init
      	 (is-running shouldStopped)
      	 (is-running root)
      	 (depends root root)
      	 (requests root root)
      	 (requests root shouldRunning)

      	 (task shouldStopped)
      	 (task shouldRunning)
      	 (task cmpChild1)
      	 (task cmpChild2)
        (composition TestCmp)
      (depends TestCmp cmpChild1)
      (depends TestCmp cmpChild2)
      (depends TestCmp TestCmp2)
      	 (task cmpChild21)
      	 (task cmpChild22)
        (composition TestCmp2)
      (depends TestCmp2 cmpChild21)
      (depends TestCmp2 cmpChild22)


;;Testfall composition läft aber keiner da
;      (is-running TestCmp)
;      (is-running cmpChild1)
;      (requests TestCmp cmpChild1)
;;Testfall cmp starten
      (requests root TestCmp)
;      (requests root TestCmp2)
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
  
    (forall (?t - instance_req)
    (forall (?r - instance_req)
            (imply
            (requests ?r ?t)
            (is-running ?t)
            )
            )
            )

;    (forall (?t - instance_req)
;        (imply (is-running ?t)
;            (depends ?x ?t)
;            (is-running ?t)
;        )
;    )
;      	 (is-running root)
;      	 (depends root shouldRunning)
;        (requests root TestCmp)
;        (depends root TestCmp)
;     	 (requests root TestCmp2)
  ))
)
