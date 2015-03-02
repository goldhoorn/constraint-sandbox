(define (domain network)
  (:requirements :strips :equality :typing :conditional-effects)
  (:types input_port output_port instance_req)
  (:predicates
	 (has-output ?x - instance_req ?y - output_port)
	 (has-input ?x - instance_req ?y - input_port)
	 (is-connected ?x - output_port ?y - input_port)
	 (should-connected ?x - instance_req ?y - input_port ?z - output_port)
	 (requests ?x - instance_req ?y - instance_req)
	 (depends ?x - instance_req ?y - instance_req)
	 (is-running ?x - instance_req)
	 (task ?x - instance_req)
	 (composition ?x - instance_req)
)

(:action startt :parameters (?r - instance_req ?t - instance_req)
 :precondition (and
    (not (is-running ?t)) ;;Unsure ob das rein soll
    (task ?t) 
    (requests ?r ?t)
  )
 :effect (and   (is-running ?t) (depends ?r ?t) )
)

(:action stop :parameters (?t - instance_req)
 :precondition (and
    (not (exists (?r - instance_req) (requests ?r ?t)))
    (is-running ?t)
    )
 :effect (and  
    (not (is-running ?t))
    (forall (?dep - instance_req)
        ;(when (depends ?t ?dep) (not (depends ?t ?dep) ) )
        (not (depends ?dep ?t) ) 
    )
  )
)

(:action startc :parameters (?r - instance_req ?c - instance_req)
 :precondition (and
   (not (is-running ?c))
   (requests ?r ?c)
   (composition ?c)
   (forall (?dep - instance_req) 
    (imply (depends ?c ?dep) (is-running ?dep)) 
   )
  )
 :effect (and (is-running ?c) (depends ?r ?c) (forall (?dep - instance_req) (when (depends ?c ?dep) (requests ?c ?dep ))))
)

;;Needed interrim step to start tasks, make sure one of the previous elemtns is required before adding this constraint
;;Todo should be checked to match recursive constraints
(:action addRequirement :parameters (?r2 - instance_req ?r - instance_req ?c - instance_req)
    :precondition (and
        (requests ?r2 ?r)
        (depends ?r ?c)
    )
    :effect (requests ?r ?c)
)

)
