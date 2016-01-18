(define (problem network001)
  (:domain network)
  (:objects
      root - instance_req
      mars-Altimeter - instance_req
;      mars-AuvController - instance_req
;      mars-AuvMotion - instance_req
;      mars-CameraPlugin - instance_req
;      mars-DepthCamera - instance_req
;      mars-ForceApplier - instance_req
;      mars-ForceTorque6DOF - instance_req
;      mars-HighResRangeFinder - instance_req
;      mars-Joints - instance_req
;      mars-LaserRangeFinder - instance_req
;      mars-Plugin - instance_req
;      mars-RotatingLaserRangeFinder - instance_req
;      mars-Sonar - instance_req
;      mars-Trigger - instance_req
      mars-Camera - instance_req
      mars-IMU - instance_req
      mars-Task - instance_req
      TestCmp - instance_req
  )
  (:init
      	 (is-running mars-Altimeter)
      	 (is-running root)
      	 (depends root root)
      	 (requests root root)

      	 (requests root mars-Camera)
      	 (task mars-Altimeter)
;      	 (task mars-AuvController)
;      	 (task mars-AuvMotion)
      	 (task mars-Camera)
;      	 (task mars-CameraPlugin)
;      	 (task mars-DepthCamera)
;      	 (task mars-ForceApplier)
;      	 (task mars-ForceTorque6DOF)
;      	 (task mars-HighResRangeFinder)
      	 (task mars-IMU)
;      	 (task mars-Joints)
;      	 (task mars-LaserRangeFinder)
;      	 (task mars-Plugin)
;      	 (task mars-RotatingLaserRangeFinder)
;      	 (task mars-Sonar)
      	 (task mars-Task)
;      	 (task mars-Trigger)
      (composition TestCmp)
      (depends TestCmp mars-IMU)
      (depends TestCmp mars-Task)

;;Testfall composition läft aber keiner da
;      (is-running TestCmp)
;      (is-running mars-IMU)
;      (requests TestCmp mars-IMU)
;;Testfall cmp starten
      (requests root TestCmp)
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
;            (depends ?x_ ?t)
;;            (is-running ?t)
;        )
;    )
      	 (is-running root)
;      	 (depends root mars-Camera)
;      	 (depends root TestCmp)
  ))
)
