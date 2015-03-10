(define (domain network)
  (:requirements :strips :equality :typing :conditional-effects)
  (:types input_port output_port instance_req)
  (:predicates
	 (has-output ?x - instance_req ?y - output_port)
	 (has-input ?x - instance_req ?y - input_port)
	 (is-connected ?x - output_port ?y - input_port)
	 (should-connected ?x - instance_req ?y - input_port ?z - output_port)
	 (requests ?x - instance_req ?y - instance_req)
	 (fullfills ?x - instance_req ?y - instance_req)
	 (depends ?x - instance_req ?y - instance_req)
	 (is-running ?x - instance_req)
	 (data-service ?x - instance_req)
	 (task ?x - instance_req)
	 (composition ?x - instance_req)
)

(:action startt :parameters (?r - instance_req ?t - instance_req)
 :precondition (and
    (task ?t) 
    (not (is-running ?t)) ;;Unsure ob das rein soll
    (or (depends ?r ?t) (fullfills ?t ?r))
  )
 :effect (and  (is-running ?t) (requests ?r ?t) )
)

;(:action startds :parameters (?r - instance_req ?ds - instance_req)
;:precondition (and
;;    (not (is-running ?ds)) ;;Unsure ob das rein soll
;;    (data-service ?ds) 
;;    (requests ?r ?ds)
;;    (exists (?t - task) (and (fullfills ?t ?ds) ) ) ;(is-running ?t) ) )
;)
;:effect (and (is-running ?ds) (depends ?r ?ds) )
;)

(:action startds :parameters (?r - instance_req ?ds - instance_req)
:precondition (and
    (not (is-running ?ds)) ;;Unsure ob das rein soll
    (data-service ?ds) 
    (depends ?r ?ds)
    (exists (?t - instance_req) (and (fullfills ?t ?ds) (is-running ?t) (requests ?ds ?t) ) ) 
    ;(exists (?t - instance_req) (and (fullfills ?t ?ds) (is-running ?t) ) ) 
)
:effect (and (is-running ?ds) (depends ?r ?ds) (requests ?r ?ds))
)



(:action stop :parameters (?t - instance_req)
 :precondition (and
    (or (data-service ?t) (task ?t) (composition ?t) )
    (is-running ?t)
    (not (exists (?r - instance_req) (requests ?r ?t)))
    )
 :effect (and  
    (not (is-running ?t))
    (forall (?dep - instance_req)
        ;(when (depends ?t ?dep) (not (depends ?t ?dep) ) )
        (not (requests ?dep ?t) ) 
    )
  )
)

(:action start_root 
 :precondition (and
   (not (is-running root))
   ;(forall (?dep - instance_req) (imply (depends root ?dep) (or (and  (is-running ?dep) (requests root ?dep)) (depends root ?dep) ) ) ) 
   (forall (?dep - instance_req) (imply (depends root ?dep) 
        (or     
                (and (is-running ?dep) (requests root ?dep) )
                (depends ?dep root)
        ) )  
    )
  )
 :effect (is-running root) 
)

(:action startc :parameters (?r - instance_req ?c - instance_req)
 :precondition (and
   (not (is-running ?c))
   (or (depends ?r ?c) (fullfills ?c ?r))
   (composition ?c)
   (forall (?dep - instance_req) 
    (imply (depends?c ?dep) (and (is-running ?dep) (requests ?c ?dep) ) ) )
  )
 :effect (and (is-running ?c) (requests ?r ?c) ) ;(depends ?r ?c) (forall (?dep - instance_req) (when (depends ?c ?dep) (requests ?c ?dep ))))
)

;;Needed interrim step to start tasks, make sure one of the previous elemtns is required before adding this constraint
;;Todo should be checked to match recursive constraints
;(:action addRequirement :parameters (?r2 - instance_req ?r - instance_req ?c - instance_req)
;    :precondition (and
;        (requests ?r2 ?r)
;        (depends ?r ?c)
;    )
;    :effect (requests ?r ?c)
;)

)
