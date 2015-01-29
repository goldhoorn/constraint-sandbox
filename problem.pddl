(define (problem network001)
  (:domain network)
  (:objects
      auv-rel-pos-controller-Task
      pipeline-inspection-ColorFilter
      pipeline-inspection-Inspection
      pipeline-inspection-LaserSimulation
      uw-particle-localization-FastFusion
      uw-particle-localization-MotionModel
      uw-particle-localization-OrientationCorrection
      uw-particle-localization-Task
      gps-BaseTask
      gps-GPSDTask
      gps-MB500Task
      line-scanner-Task
      message-producer-Task
      buoy-Detector
      buoy-Detector2
      buoy-ServoingOnWall
      buoy-Survey
      mars-Altimeter
      mars-AuvController
      mars-AuvMotion
      mars-Camera
      mars-CameraPlugin
      mars-DepthCamera
      mars-ForceApplier
      mars-ForceTorque6DOF
      mars-HighResRangeFinder
      mars-IMU
      mars-Joints
      mars-LaserRangeFinder
      mars-Plugin
      mars-RotatingLaserRangeFinder
      mars-Sonar
      mars-Task
      mars-Trigger
      sonar-feature-estimator-Task
      modem-can-Task
      taskmon-Task
      depth-reader-Task
      raw-control-command-converter-Movement
      raw-control-command-converter-Position
      sonar-feature-detector-Task
      frame-demultiplexer-Task
      sonar-tritech-Echosounder
      sonar-tritech-Micron
      sonar-tritech-Profiling
      battery-watcher-Task
      orientation-estimator-BaseEstimator
      orientation-estimator-IKF
      orientation-estimator-IKFEstimator
      orientation-estimator-UKFEstimator
      hsv-mosaicing-Task
      sonar-wall-hough-Task
      pose-estimation-BaseTask
      pose-estimation-UWPoseEstimator
      pose-estimation-VehiclePoseEstimator
      auv-waypoint-navigator-Task
      camera-aravis-Task
      gps-helper-GPSFaker
      gps-helper-MapToGPS
      gps-helper-WaypointNavigation
      avalon-simulation-AsvNavigation
      avalon-simulation-LineLaser
      avalon-simulation-Modem
      avalon-simulation-WhiteLight
      avalon-control-FakeWriter
      avalon-control-MotionControlTask
      avalon-control-MotionFeedbackTask
      avalon-control-PositionControlTask
      avalon-control-RelFakeWriter
      avalon-control-TrajectoryFollower
      hbridge-CommandWriter
      hbridge-SensorReader
      sonar-structure-servoing-Task
      depth-map-LaserToPoints
      depth-map-Task
      depth-map-frame-export
      depth-map-sonar-ground-distance
      wall-servoing-DualSonarServoing
      wall-servoing-SingleSonarServoing
      wall-servoing-SonarServoing
      wall-servoing-WallDetector
      wall-servoing-WallServoing
      camera-unicap-CameraTask
      task-scheduler-Task
      sonar-blueview-Task
      auv-helper-depth-and-orientation-fusion
      sysmon-Task
      logger-Logger
      fog-kvh-Dsp3000Task
      rear-sonar-distance-estimator-Task
      transformer-Task
      camera-base-Task
      low-level-driver-LowLevelTask
      wall-orientation-correction-OrientationInMap
      wall-orientation-correction-Task
      structure-servoing-Alignment
      structure-servoing-Task
      image-preprocessing-BaseTask
      image-preprocessing-DepthImage2Pointcloud
      image-preprocessing-HSVSegmentationAndBlur
      image-preprocessing-MonoTask
      image-preprocessing-StereoTask
      offshore-pipeline-detector-SonarDetector
      offshore-pipeline-detector-Task
      interfaces-ActuatorCommandWriter
      interfaces-ActuatorSensorReader
      interfaces-IMU
      interfaces-LaserRangeFinder
      interfaces-Servo
      canbus-InterfaceTask
      canbus-Task
      structured-light-Calibration
      structured-light-Task
      xsens-imu-Task
      camera-prosilica-Task
      dynamixel-Task
      video-streamer-vlc-Capturer
      video-streamer-vlc-Streamer
      controldev-GenericRawToMotion2D
      controldev-GenericTask
      controldev-JoyPadTask
      controldev-JoystickTask
      controldev-Mouse3DTask
      controldev-RawJoystickToMotion2D
      controldev-RawWheelToMotion2D
      controldev-Remote
      controldev-SliderboxTask
      controldev-SteeringWheelTask
      structure-reconstruction-Task
      lights-Lights
      auv-control-AccelerationController
      auv-control-AlignedToBody
      auv-control-Base
      auv-control-BasePIDController
      auv-control-ConstantCommand
      auv-control-ConstantCommandGroundAvoidance
      auv-control-ConstantCommandGroundFollower
      auv-control-MotionCommand2DConverter
      auv-control-OptimalHeadingController
      auv-control-PIDController
      auv-control-WaypointNavigator
      auv-control-WorldToAligned
      modemdriver-Modem
      modemdriver-ModemCanbus
      modemdriver-ModemSerial
  )
  (:init
      	 (task auv-rel-pos-controller-Task)
      	 (task pipeline-inspection-ColorFilter)
      	 (task pipeline-inspection-Inspection)
      	 (task pipeline-inspection-LaserSimulation)
      	 (task uw-particle-localization-FastFusion)
      	 (task uw-particle-localization-MotionModel)
      	 (task uw-particle-localization-OrientationCorrection)
      	 (task uw-particle-localization-Task)
      	 (task gps-BaseTask)
      	 (task gps-GPSDTask)
      	 (task gps-MB500Task)
      	 (task line-scanner-Task)
      	 (task message-producer-Task)
      	 (task buoy-Detector)
      	 (task buoy-Detector2)
      	 (task buoy-ServoingOnWall)
      	 (task buoy-Survey)
      	 (task mars-Altimeter)
      	 (task mars-AuvController)
      	 (task mars-AuvMotion)
      	 (task mars-Camera)
      	 (task mars-CameraPlugin)
      	 (task mars-DepthCamera)
      	 (task mars-ForceApplier)
      	 (task mars-ForceTorque6DOF)
      	 (task mars-HighResRangeFinder)
      	 (task mars-IMU)
      	 (task mars-Joints)
      	 (task mars-LaserRangeFinder)
      	 (task mars-Plugin)
      	 (task mars-RotatingLaserRangeFinder)
      	 (task mars-Sonar)
      	 (task mars-Task)
      	 (task mars-Trigger)
      	 (task sonar-feature-estimator-Task)
      	 (task modem-can-Task)
      	 (task taskmon-Task)
      	 (task depth-reader-Task)
      	 (task raw-control-command-converter-Movement)
      	 (task raw-control-command-converter-Position)
      	 (task sonar-feature-detector-Task)
      	 (task frame-demultiplexer-Task)
      	 (task sonar-tritech-Echosounder)
      	 (task sonar-tritech-Micron)
      	 (task sonar-tritech-Profiling)
      	 (task battery-watcher-Task)
      	 (task orientation-estimator-BaseEstimator)
      	 (task orientation-estimator-IKF)
      	 (task orientation-estimator-IKFEstimator)
      	 (task orientation-estimator-UKFEstimator)
      	 (task hsv-mosaicing-Task)
      	 (task sonar-wall-hough-Task)
      	 (task pose-estimation-BaseTask)
      	 (task pose-estimation-UWPoseEstimator)
      	 (task pose-estimation-VehiclePoseEstimator)
      	 (task auv-waypoint-navigator-Task)
      	 (task camera-aravis-Task)
      	 (task gps-helper-GPSFaker)
      	 (task gps-helper-MapToGPS)
      	 (task gps-helper-WaypointNavigation)
      	 (task avalon-simulation-AsvNavigation)
      	 (task avalon-simulation-LineLaser)
      	 (task avalon-simulation-Modem)
      	 (task avalon-simulation-WhiteLight)
      	 (task avalon-control-FakeWriter)
      	 (task avalon-control-MotionControlTask)
      	 (task avalon-control-MotionFeedbackTask)
      	 (task avalon-control-PositionControlTask)
      	 (task avalon-control-RelFakeWriter)
      	 (task avalon-control-TrajectoryFollower)
      	 (task hbridge-CommandWriter)
      	 (task hbridge-SensorReader)
      	 (task sonar-structure-servoing-Task)
      	 (task depth-map-LaserToPoints)
      	 (task depth-map-Task)
      	 (task depth-map-frame-export)
      	 (task depth-map-sonar-ground-distance)
      	 (task wall-servoing-DualSonarServoing)
      	 (task wall-servoing-SingleSonarServoing)
      	 (task wall-servoing-SonarServoing)
      	 (task wall-servoing-WallDetector)
      	 (task wall-servoing-WallServoing)
      	 (task camera-unicap-CameraTask)
      	 (task task-scheduler-Task)
      	 (task sonar-blueview-Task)
      	 (task auv-helper-depth-and-orientation-fusion)
      	 (task sysmon-Task)
      	 (task logger-Logger)
      	 (task fog-kvh-Dsp3000Task)
      	 (task rear-sonar-distance-estimator-Task)
      	 (task transformer-Task)
      	 (task camera-base-Task)
      	 (task low-level-driver-LowLevelTask)
      	 (task wall-orientation-correction-OrientationInMap)
      	 (task wall-orientation-correction-Task)
      	 (task structure-servoing-Alignment)
      	 (task structure-servoing-Task)
      	 (task image-preprocessing-BaseTask)
      	 (task image-preprocessing-DepthImage2Pointcloud)
      	 (task image-preprocessing-HSVSegmentationAndBlur)
      	 (task image-preprocessing-MonoTask)
      	 (task image-preprocessing-StereoTask)
      	 (task offshore-pipeline-detector-SonarDetector)
      	 (task offshore-pipeline-detector-Task)
      	 (task interfaces-ActuatorCommandWriter)
      	 (task interfaces-ActuatorSensorReader)
      	 (task interfaces-IMU)
      	 (task interfaces-LaserRangeFinder)
      	 (task interfaces-Servo)
      	 (task canbus-InterfaceTask)
      	 (task canbus-Task)
      	 (task structured-light-Calibration)
      	 (task structured-light-Task)
      	 (task xsens-imu-Task)
      	 (task camera-prosilica-Task)
      	 (task dynamixel-Task)
      	 (task video-streamer-vlc-Capturer)
      	 (task video-streamer-vlc-Streamer)
      	 (task controldev-GenericRawToMotion2D)
      	 (task controldev-GenericTask)
      	 (task controldev-JoyPadTask)
      	 (task controldev-JoystickTask)
      	 (task controldev-Mouse3DTask)
      	 (task controldev-RawJoystickToMotion2D)
      	 (task controldev-RawWheelToMotion2D)
      	 (task controldev-Remote)
      	 (task controldev-SliderboxTask)
      	 (task controldev-SteeringWheelTask)
      	 (task structure-reconstruction-Task)
      	 (task lights-Lights)
      	 (task auv-control-AccelerationController)
      	 (task auv-control-AlignedToBody)
      	 (task auv-control-Base)
      	 (task auv-control-BasePIDController)
      	 (task auv-control-ConstantCommand)
      	 (task auv-control-ConstantCommandGroundAvoidance)
      	 (task auv-control-ConstantCommandGroundFollower)
      	 (task auv-control-MotionCommand2DConverter)
      	 (task auv-control-OptimalHeadingController)
      	 (task auv-control-PIDController)
      	 (task auv-control-WaypointNavigator)
      	 (task auv-control-WorldToAligned)
      	 (task modemdriver-Modem)
      	 (task modemdriver-ModemCanbus)
      	 (task modemdriver-ModemSerial)
  )
  (:goal
        (and( task(?x - task) (is-running ?x)))
  )
)
