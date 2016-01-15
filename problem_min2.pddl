(define (problem network001)
  (:domain network)
  (:objects
      root
      lights-Lights
  )
  (:init
      	 (task root)
      	 (is-running root)
      	 (depends root root)
      	 (task lights-Lights)
  )
  (:goal
  (and
      	 (is-running root)
         (depends root lights-Lights)
  )
  )
)
