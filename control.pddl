(define (control lognetwork)
    (:domain network)

(:defpredicate unused
    :parameters (?task)
;;    :body (not (exists (?parent) (and (depends ?parent ?task) ) ) )
    :body (task modemdriver-Task)
)

(:defpredicate valid-running-state
    :parameters (?task)
    ;; :body (not (exists (?task) (and (is-running ?task) (task ?task) ) ) )
    :body (task ?task)
)

)
