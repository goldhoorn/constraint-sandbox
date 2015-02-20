(define (problem network001)
  (:domain network)
  (:objects
      root
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
      dynamixel-Task
      pddl-planner-Task
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
      auv-rel-pos-controller-Task.motion-command
      auv-rel-pos-controller-Task.state
      auv-rel-pos-controller-Task.position-command
      auv-rel-pos-controller-Task.position-sample
      pipeline-inspection-ColorFilter.frame-out
      pipeline-inspection-ColorFilter.green-frame
      pipeline-inspection-ColorFilter.diff-frame
      pipeline-inspection-ColorFilter.state
      pipeline-inspection-ColorFilter.frame-in
      pipeline-inspection-Inspection.inspectionStatus
      pipeline-inspection-Inspection.pipePoints
      pipeline-inspection-Inspection.debugFrame
      pipeline-inspection-Inspection.pipeMap
      pipeline-inspection-Inspection.state
      pipeline-inspection-Inspection.laserSamples
      pipeline-inspection-Inspection.laserPoints
      pipeline-inspection-Inspection.laserPointCloud
      pipeline-inspection-Inspection.pipeline
      pipeline-inspection-Inspection.dead-reckoning
      pipeline-inspection-LaserSimulation.laserPoints
      pipeline-inspection-LaserSimulation.laserPointCloud
      pipeline-inspection-LaserSimulation.vehiclePos
      pipeline-inspection-LaserSimulation.state
      uw-particle-localization-FastFusion.pose-samples
      uw-particle-localization-FastFusion.state
      uw-particle-localization-FastFusion.position-samples
      uw-particle-localization-FastFusion.depth-samples
      uw-particle-localization-FastFusion.orientation-samples
      uw-particle-localization-FastFusion.velocity-samples
      uw-particle-localization-MotionModel.pose-samples
      uw-particle-localization-MotionModel.stream-aligner-status
      uw-particle-localization-MotionModel.state
      uw-particle-localization-MotionModel.thruster-samples
      uw-particle-localization-MotionModel.orientation-samples
      uw-particle-localization-OrientationCorrection.orientation-output
      uw-particle-localization-OrientationCorrection.orientation-offset-corrected
      uw-particle-localization-OrientationCorrection.state
      uw-particle-localization-OrientationCorrection.orientation-input
      uw-particle-localization-OrientationCorrection.orientation-offset
      uw-particle-localization-Task.pose-samples
      uw-particle-localization-Task.environment
      uw-particle-localization-Task.dead-reckoning-samples
      uw-particle-localization-Task.full-dead-reckoning
      uw-particle-localization-Task.particles
      uw-particle-localization-Task.debug-sonar-beam
      uw-particle-localization-Task.stats
      uw-particle-localization-Task.depth-grid
      uw-particle-localization-Task.grid-map
      uw-particle-localization-Task.debug-filtered-obstacles
      uw-particle-localization-Task.stream-aligner-status
      uw-particle-localization-Task.state
      uw-particle-localization-Task.laser-samples
      uw-particle-localization-Task.speed-samples
      uw-particle-localization-Task.pipeline-samples
      uw-particle-localization-Task.pose-update
      uw-particle-localization-Task.gps-pose-samples
      uw-particle-localization-Task.buoy-samples-orange
      uw-particle-localization-Task.buoy-samples-white
      uw-particle-localization-Task.thruster-samples
      uw-particle-localization-Task.orientation-samples
      uw-particle-localization-Task.echosounder-samples
      uw-particle-localization-Task.obstacle-samples
      uw-particle-localization-Task.structur-samples
      gps-BaseTask.solution
      gps-BaseTask.position-samples
      gps-BaseTask.state
      gps-MB500Task.constellation
      gps-MB500Task.time
      line-scanner-Task.state
      line-scanner-Task.pointcloud
      line-scanner-Task.debug
      line-scanner-Task.frame
      message-producer-Task.messages
      message-producer-Task.state
      buoy-Detector.state
      buoy-Detector.buoy
      buoy-Detector.light
      buoy-Detector.h-image
      buoy-Detector.s-image
      buoy-Detector.v-image
      buoy-Detector.binary-debug-image
      buoy-Detector.gray-debug-image
      buoy-Detector.hough-debug-image
      buoy-Detector.other-buoys
      buoy-Detector.debug-image
      buoy-Detector.frame
      buoy-Detector2.state
      buoy-Detector2.buoy
      buoy-Detector2.light
      buoy-Detector2.h-image
      buoy-Detector2.s-image
      buoy-Detector2.v-image
      buoy-Detector2.binary-debug-image
      buoy-Detector2.gray-debug-image
      buoy-Detector2.hough-debug-image
      buoy-Detector2.other-buoys
      buoy-Detector2.debug-image
      buoy-Detector2.frame
      buoy-ServoingOnWall.world-cmd
      buoy-ServoingOnWall.aligned-position-cmd
      buoy-ServoingOnWall.state
      buoy-ServoingOnWall.buoy-samples
      buoy-ServoingOnWall.wall-samples
      buoy-ServoingOnWall.orientation-samples
      buoy-Survey.state
      buoy-Survey.strafed-angle
      buoy-Survey.relative-position
      buoy-Survey.position
      buoy-Survey.orientation-samples
      buoy-Survey.force-cutting
      buoy-Survey.input-buoy
      buoy-Survey.motion-command
      buoy-Survey.light
      buoy-Survey.target-angle-input
      mars-Altimeter.ground-distance
      mars-Camera.frame
      mars-DepthCamera.distance-image
      mars-ForceApplier.status
      mars-ForceApplier.command
      mars-ForceTorque6DOF.wrenches
      mars-HighResRangeFinder.pointcloud
      mars-IMU.orientation-samples
      mars-IMU.calibrated-sensors
      mars-IMU.pose-samples
      mars-Joints.status-samples
      mars-Joints.current-values
      mars-Joints.transforms
      mars-Joints.command
      mars-LaserRangeFinder.scans
      mars-Plugin.state
      mars-RotatingLaserRangeFinder.pointcloud
      mars-Sonar.sonar-beam
      mars-Task.time
      mars-Task.simulated-time
      mars-Task.state
      mars-Task.control-action
      sonar-feature-estimator-Task.new-feature
      sonar-feature-estimator-Task.features-out
      sonar-feature-estimator-Task.debug-output
      sonar-feature-estimator-Task.2d-debug-output
      sonar-feature-estimator-Task.state
      sonar-feature-estimator-Task.sonar-input
      sonar-feature-estimator-Task.orientation-sample
      modem-can-Task.modem-out
      modem-can-Task.canOut
      modem-can-Task.motion-command
      modem-can-Task.state
      modem-can-Task.modem-in
      modem-can-Task.canModem
      modem-can-Task.light-value
      modem-can-Task.position-samples
      taskmon-Task.stats
      taskmon-Task.state
      depth-reader-Task.depthOut
      depth-reader-Task.depth-samples
      depth-reader-Task.canOut
      depth-reader-Task.state
      depth-reader-Task.canIn
      raw-control-command-converter-Movement.motion-command
      raw-control-command-converter-Movement.world-command
      raw-control-command-converter-Movement.world-command-depth
      raw-control-command-converter-Movement.aligned-velocity-command
      raw-control-command-converter-Movement.state
      raw-control-command-converter-Movement.raw-command
      raw-control-command-converter-Movement.orientation-readings
      raw-control-command-converter-Movement.ground-distance
      raw-control-command-converter-Position.position-command
      raw-control-command-converter-Position.world-command
      raw-control-command-converter-Position.state
      raw-control-command-converter-Position.raw-command
      raw-control-command-converter-Position.pose-samples
      sonar-feature-detector-Task.features
      sonar-feature-detector-Task.next-target
      sonar-feature-detector-Task.next-target-feature
      sonar-feature-detector-Task.next-target-command
      sonar-feature-detector-Task.state
      sonar-feature-detector-Task.grid-maps
      sonar-feature-detector-Task.pose-samples
      frame-demultiplexer-Task.oframe-pair
      frame-demultiplexer-Task.oframe
      frame-demultiplexer-Task.state
      frame-demultiplexer-Task.iframe
      sonar-tritech-Echosounder.ground-distance
      sonar-tritech-Echosounder.state
      sonar-tritech-Micron.ground-distance
      sonar-tritech-Micron.sonar-beam
      sonar-tritech-Micron.state
      sonar-tritech-Profiling.profiling-scan
      sonar-tritech-Profiling.state
      battery-watcher-Task.battery-info
      orientation-estimator-BaseEstimator.stream-aligner-status
      orientation-estimator-BaseEstimator.attitude-b-g
      orientation-estimator-BaseEstimator.state
      orientation-estimator-BaseEstimator.imu-orientation
      orientation-estimator-BaseEstimator.fog-samples
      orientation-estimator-BaseEstimator.heading-correction
      orientation-estimator-IKF.transformer-stream-aligner-status
      orientation-estimator-IKF.transformer-status
      orientation-estimator-IKF.attitude-b-g
      orientation-estimator-IKF.state
      orientation-estimator-IKF.imu-samples
      orientation-estimator-IKF.fog-samples
      orientation-estimator-IKF.initial-orientation
      orientation-estimator-IKF.dynamic-transformations
      orientation-estimator-IKFEstimator.stream-aligner-status
      orientation-estimator-IKFEstimator.attitude-b-g
      orientation-estimator-IKFEstimator.inputs-backup
      orientation-estimator-IKFEstimator.state
      orientation-estimator-IKFEstimator.imu-orientation
      orientation-estimator-IKFEstimator.fog-samples
      orientation-estimator-IKFEstimator.imu-samples
      orientation-estimator-UKFEstimator.stream-aligner-status
      orientation-estimator-UKFEstimator.attitude-b-g
      orientation-estimator-UKFEstimator.state
      orientation-estimator-UKFEstimator.imu-orientation
      orientation-estimator-UKFEstimator.fog-samples
      orientation-estimator-UKFEstimator.imu-samples
      hsv-mosaicing-Task.result
      hsv-mosaicing-Task.state
      hsv-mosaicing-Task.frame
      sonar-wall-hough-Task.lines
      sonar-wall-hough-Task.peaks
      sonar-wall-hough-Task.houghspace
      sonar-wall-hough-Task.position
      sonar-wall-hough-Task.position-quality
      sonar-wall-hough-Task.state
      sonar-wall-hough-Task.sonar-samples
      sonar-wall-hough-Task.reset
      sonar-wall-hough-Task.orientation-samples
      sonar-wall-hough-Task.pose-samples
      pose-estimation-BaseTask.pose-samples
      pose-estimation-BaseTask.state
      pose-estimation-UWPoseEstimator.transformer-stream-aligner-status
      pose-estimation-UWPoseEstimator.transformer-status
      pose-estimation-UWPoseEstimator.orientation-samples
      pose-estimation-UWPoseEstimator.depth-samples
      pose-estimation-UWPoseEstimator.dvl-velocity-samples
      pose-estimation-UWPoseEstimator.model-velocity-samples
      pose-estimation-UWPoseEstimator.lbl-position-samples
      pose-estimation-UWPoseEstimator.xy-position-samples
      pose-estimation-UWPoseEstimator.dynamic-transformations
      pose-estimation-VehiclePoseEstimator.transformer-stream-aligner-status
      pose-estimation-VehiclePoseEstimator.transformer-status
      pose-estimation-VehiclePoseEstimator.orientation-samples
      pose-estimation-VehiclePoseEstimator.velocity-samples
      pose-estimation-VehiclePoseEstimator.position-samples
      pose-estimation-VehiclePoseEstimator.dynamic-transformations
      auv-waypoint-navigator-Task.relative-position-command
      auv-waypoint-navigator-Task.current-delta
      auv-waypoint-navigator-Task.current-waypoint
      auv-waypoint-navigator-Task.queue-size
      auv-waypoint-navigator-Task.state
      auv-waypoint-navigator-Task.trajectory
      auv-waypoint-navigator-Task.pose-samples
      gps-helper-GPSFaker.position-samples
      gps-helper-GPSFaker.state
      gps-helper-MapToGPS.gps-position
      gps-helper-MapToGPS.transformer-stream-aligner-status
      gps-helper-MapToGPS.transformer-status
      gps-helper-MapToGPS.state
      gps-helper-MapToGPS.position-samples
      gps-helper-MapToGPS.dynamic-transformations
      gps-helper-WaypointNavigation.target-waypoint
      gps-helper-WaypointNavigation.heading-offset
      gps-helper-WaypointNavigation.distance-delta
      gps-helper-WaypointNavigation.state
      gps-helper-WaypointNavigation.gps-position-samples
      gps-helper-WaypointNavigation.pose-samples
      avalon-simulation-AsvNavigation.state
      avalon-simulation-Modem.motion-command
      avalon-simulation-Modem.state
      avalon-simulation-Modem.light-value
      avalon-simulation-Modem.position-samples
      avalon-simulation-WhiteLight.state
      avalon-control-FakeWriter.motion-commands
      avalon-control-FakeWriter.state
      avalon-control-MotionControlTask.hbridge-commands
      avalon-control-MotionControlTask.joint-commands
      avalon-control-MotionControlTask.debug
      avalon-control-MotionControlTask.estimated-ground-pos
      avalon-control-MotionControlTask.state
      avalon-control-MotionControlTask.dummy-feedback
      avalon-control-MotionControlTask.ground-distance
      avalon-control-MotionControlTask.pose-samples
      avalon-control-MotionControlTask.motion-commands
      avalon-control-MotionControlTask.joints-in
      avalon-control-MotionFeedbackTask.hbridge-status
      avalon-control-MotionFeedbackTask.state
      avalon-control-MotionFeedbackTask.hbridge-feedback
      avalon-control-PositionControlTask.motion-commands
      avalon-control-PositionControlTask.state
      avalon-control-PositionControlTask.pose-samples
      avalon-control-PositionControlTask.position-commands
      avalon-control-RelFakeWriter.position-command
      avalon-control-RelFakeWriter.state
      avalon-control-TrajectoryFollower.next-position
      avalon-control-TrajectoryFollower.position-command
      avalon-control-TrajectoryFollower.next-pos-on-spline
      avalon-control-TrajectoryFollower.last-pos-on-spline
      avalon-control-TrajectoryFollower.segment-dist
      avalon-control-TrajectoryFollower.world-command
      avalon-control-TrajectoryFollower.state
      avalon-control-TrajectoryFollower.pose-samples
      hbridge-CommandWriter.state
      hbridge-CommandWriter.can-out
      hbridge-CommandWriter.speedCtrlDebug
      hbridge-CommandWriter.fakeReader
      hbridge-CommandWriter.can-in
      hbridge-CommandWriter.command
      hbridge-SensorReader.state
      hbridge-SensorReader.can-out
      hbridge-SensorReader.status-samples
      hbridge-SensorReader.can-in
      sonar-structure-servoing-Task.position-command
      sonar-structure-servoing-Task.aligned-position-command
      sonar-structure-servoing-Task.world-command
      sonar-structure-servoing-Task.debug-data
      sonar-structure-servoing-Task.transformer-stream-aligner-status
      sonar-structure-servoing-Task.transformer-status
      sonar-structure-servoing-Task.state
      sonar-structure-servoing-Task.sonarbeam-feature
      sonar-structure-servoing-Task.odometry-samples
      sonar-structure-servoing-Task.dynamic-transformations
      depth-map-LaserToPoints.pointcloud
      depth-map-LaserToPoints.state
      depth-map-LaserToPoints.laser-scan
      depth-map-LaserToPoints.rbs
      depth-map-Task.output
      depth-map-Task.state
      depth-map-Task.input
      depth-map-frame-export.frames
      depth-map-frame-export.minZ
      depth-map-frame-export.maxZ
      depth-map-frame-export.state
      depth-map-frame-export.pointcloud
      depth-map-sonar-ground-distance.new-feature
      depth-map-sonar-ground-distance.state
      depth-map-sonar-ground-distance.sonar-input
      wall-servoing-DualSonarServoing.position-command
      wall-servoing-DualSonarServoing.aligned-command
      wall-servoing-DualSonarServoing.wall-servoing-debug
      wall-servoing-DualSonarServoing.sonarbeam-feature-front
      wall-servoing-DualSonarServoing.sonarbeam-feature-rear
      wall-servoing-DualSonarServoing.orientation-sample
      wall-servoing-SingleSonarServoing.position-command
      wall-servoing-SingleSonarServoing.aligned-position-command
      wall-servoing-SingleSonarServoing.world-command
      wall-servoing-SingleSonarServoing.wall-servoing-debug
      wall-servoing-SingleSonarServoing.wall
      wall-servoing-SingleSonarServoing.sonarbeam-feature
      wall-servoing-SingleSonarServoing.orientation-sample
      wall-servoing-SingleSonarServoing.position-sample
      wall-servoing-SonarServoing.state
      wall-servoing-WallDetector.point-cloud
      wall-servoing-WallDetector.wall
      wall-servoing-WallDetector.state
      wall-servoing-WallDetector.sonarbeam-feature
      wall-servoing-WallDetector.orientation-sample
      wall-servoing-WallDetector.position-sample
      wall-servoing-WallServoing.motion-command
      wall-servoing-WallServoing.world-command
      wall-servoing-WallServoing.aligned-velocity-command
      wall-servoing-WallServoing.state
      wall-servoing-WallServoing.orientation-sample
      wall-servoing-WallServoing.servoing-wall
      wall-servoing-WallServoing.obstacle-wall
      camera-unicap-CameraTask.frame
      camera-unicap-CameraTask.state
      task-scheduler-Task.state
      task-scheduler-Task.trigger
      sonar-blueview-Task.frame
      sonar-blueview-Task.state
      auv-helper-depth-and-orientation-fusion.pose-samples
      auv-helper-depth-and-orientation-fusion.stream-aligner-status
      auv-helper-depth-and-orientation-fusion.state
      auv-helper-depth-and-orientation-fusion.orientation-samples
      auv-helper-depth-and-orientation-fusion.depth-samples
      auv-helper-depth-and-orientation-fusion.ground-distance
      sysmon-Task.ocu-markers
      sysmon-Task.annotations
      sysmon-Task.system-status
      sysmon-Task.state
      sysmon-Task.can-in-system-status
      sysmon-Task.can-in-experiment-markers
      sysmon-Task.in-experiment-markers
      sysmon-Task.in-modem-substates
      logger-Logger.state
      fog-kvh-Dsp3000Task.rotation
      fog-kvh-Dsp3000Task.orientation-samples
      fog-kvh-Dsp3000Task.timestamp-estimator-status
      fog-kvh-Dsp3000Task.state
      fog-kvh-Dsp3000Task.config
      rear-sonar-distance-estimator-Task.ground-distance
      rear-sonar-distance-estimator-Task.state
      rear-sonar-distance-estimator-Task.BaseScan
      rear-sonar-distance-estimator-Task.depth-samples
      transformer-Task.configuration-state
      transformer-Task.state
      camera-base-Task.frame
      camera-base-Task.frame-raw
      camera-base-Task.state
      low-level-driver-LowLevelTask.state
      low-level-driver-LowLevelTask.depth-samples
      low-level-driver-LowLevelTask.ShortExposure
      low-level-driver-LowLevelTask.LongExposure
      low-level-driver-LowLevelTask.LightValue
      low-level-driver-LowLevelTask.DebugLED
      low-level-driver-LowLevelTask.LaserRate
      wall-orientation-correction-OrientationInMap.orientation-in-map
      wall-orientation-correction-OrientationInMap.transformer-stream-aligner-status
      wall-orientation-correction-OrientationInMap.transformer-status
      wall-orientation-correction-OrientationInMap.state
      wall-orientation-correction-OrientationInMap.orientation-in-world
      wall-orientation-correction-OrientationInMap.dynamic-transformations
      wall-orientation-correction-Task.orientation-in-world
      wall-orientation-correction-Task.angle-in-world
      wall-orientation-correction-Task.debug-data
      wall-orientation-correction-Task.transformer-stream-aligner-status
      wall-orientation-correction-Task.transformer-status
      wall-orientation-correction-Task.state
      wall-orientation-correction-Task.sonarbeam-feature
      wall-orientation-correction-Task.orientation-samples
      wall-orientation-correction-Task.dynamic-transformations
      structure-servoing-Alignment.world-command
      structure-servoing-Alignment.aligned-speed-command
      structure-servoing-Alignment.left
      structure-servoing-Alignment.right
      structure-servoing-Alignment.top
      structure-servoing-Alignment.bottom
      structure-servoing-Alignment.size
      structure-servoing-Alignment.x
      structure-servoing-Alignment.y
      structure-servoing-Alignment.state
      structure-servoing-Alignment.input
      structure-servoing-Task.servoed-angle
      structure-servoing-Task.angle-speed
      structure-servoing-Task.world-command
      structure-servoing-Task.aligned-speed-command
      structure-servoing-Task.found-structure
      structure-servoing-Task.state
      structure-servoing-Task.left
      structure-servoing-Task.right
      structure-servoing-Task.top
      structure-servoing-Task.bottom
      structure-servoing-Task.size
      structure-servoing-Task.heading
      structure-servoing-Task.cnt-left
      structure-servoing-Task.cnt-right
      structure-servoing-Task.cnt-top
      structure-servoing-Task.cnt-bottom
      structure-servoing-Task.rbs
      structure-servoing-Task.input
      image-preprocessing-BaseTask.state
      image-preprocessing-DepthImage2Pointcloud.pointcloud
      image-preprocessing-DepthImage2Pointcloud.stream-aligner-status
      image-preprocessing-DepthImage2Pointcloud.state
      image-preprocessing-DepthImage2Pointcloud.color-frame
      image-preprocessing-DepthImage2Pointcloud.frame
      image-preprocessing-HSVSegmentationAndBlur.oframe
      image-preprocessing-HSVSegmentationAndBlur.binary-result
      image-preprocessing-HSVSegmentationAndBlur.hDebug
      image-preprocessing-HSVSegmentationAndBlur.hDebugGray
      image-preprocessing-HSVSegmentationAndBlur.vDebug
      image-preprocessing-HSVSegmentationAndBlur.vDebugGray
      image-preprocessing-HSVSegmentationAndBlur.sDebug
      image-preprocessing-HSVSegmentationAndBlur.sDebugGray
      image-preprocessing-HSVSegmentationAndBlur.hsv-v-frame
      image-preprocessing-HSVSegmentationAndBlur.state
      image-preprocessing-HSVSegmentationAndBlur.frame
      image-preprocessing-MonoTask.oframe
      image-preprocessing-MonoTask.frame
      image-preprocessing-StereoTask.oframe-pair
      image-preprocessing-StereoTask.frame-left
      image-preprocessing-StereoTask.frame-right
      offshore-pipeline-detector-SonarDetector.frame
      offshore-pipeline-detector-SonarDetector.state
      offshore-pipeline-detector-SonarDetector.sonar-beam
      offshore-pipeline-detector-Task.state
      offshore-pipeline-detector-Task.pipeline
      offshore-pipeline-detector-Task.world-command
      offshore-pipeline-detector-Task.aligned-position-command
      offshore-pipeline-detector-Task.position-command
      offshore-pipeline-detector-Task.debug
      offshore-pipeline-detector-Task.debug-frame
      offshore-pipeline-detector-Task.frame
      offshore-pipeline-detector-Task.orientation-sample
      offshore-pipeline-detector-Task.altitude-samples
      interfaces-ActuatorCommandWriter.state
      interfaces-ActuatorSensorReader.state
      interfaces-IMU.orientation-samples
      interfaces-IMU.calibrated-sensors
      interfaces-IMU.state
      interfaces-LaserRangeFinder.scans
      interfaces-LaserRangeFinder.state
      interfaces-Servo.upper2lower
      interfaces-Servo.angle
      interfaces-Servo.state
      interfaces-Servo.cmd-angle
      canbus-InterfaceTask.can-out
      canbus-InterfaceTask.state
      canbus-InterfaceTask.can-in
      canbus-Task.stats
      canbus-Task.can-status
      canbus-Task.log-message
      canbus-Task.state
      canbus-Task.in
      structured-light-Calibration.stream-aligner-status
      structured-light-Calibration.state
      structured-light-Calibration.laser-scan
      structured-light-Calibration.calibration
      structured-light-Task.laser-scan
      structured-light-Task.candidates
      structured-light-Task.debug-frame
      structured-light-Task.state
      structured-light-Task.frame-pair
      structured-light-Task.frame
      xsens-imu-Task.orientation-samples
      xsens-imu-Task.calibrated-sensors
      xsens-imu-Task.timestamp-estimator-status
      xsens-imu-Task.state
      xsens-imu-Task.hard-timestamps
      dynamixel-Task.lowerDynamixel2UpperDynamixel
      pddl-planner-Task.state
      video-streamer-vlc-Capturer.state
      video-streamer-vlc-Streamer.state
      controldev-GenericRawToMotion2D.motion-command
      controldev-GenericRawToMotion2D.state
      controldev-GenericRawToMotion2D.raw-command
      controldev-GenericTask.raw-command
      controldev-GenericTask.state
      controldev-RawJoystickToMotion2D.motion-command
      controldev-RawJoystickToMotion2D.state
      controldev-RawJoystickToMotion2D.raw-command
      controldev-RawWheelToMotion2D.motion-command
      controldev-RawWheelToMotion2D.state
      controldev-RawWheelToMotion2D.raw-command
      controldev-Remote.canInputDevice
      structure-reconstruction-Task.transformer-stream-aligner-status
      structure-reconstruction-Task.transformer-status
      structure-reconstruction-Task.state
      structure-reconstruction-Task.front-camera
      structure-reconstruction-Task.bottom-camera
      structure-reconstruction-Task.dynamic-transformations
      lights-Lights.light-value
      lights-Lights.state
      lights-Lights.int-in
      lights-Lights.can-in
      auv-control-AccelerationController.cmd-out
      auv-control-AlignedToBody.cmd-out
      auv-control-AlignedToBody.orientation-samples
      auv-control-Base.state
      auv-control-Base.cmd-in
      auv-control-Base.cmd-cascade
      auv-control-BasePIDController.cmd-out
      auv-control-BasePIDController.pid-state
      auv-control-BasePIDController.pose-samples
      auv-control-ConstantCommand.cmd-out
      auv-control-ConstantCommand.state
      auv-control-ConstantCommandGroundFollower.floor-position
      auv-control-ConstantCommandGroundFollower.state
      auv-control-ConstantCommandGroundFollower.cmd-out
      auv-control-ConstantCommandGroundFollower.altimeter
      auv-control-ConstantCommandGroundFollower.depth
      auv-control-ConstantCommandGroundFollower.cmd-in
      auv-control-MotionCommand2DConverter.cmd-out
      auv-control-MotionCommand2DConverter.state
      auv-control-MotionCommand2DConverter.cmd-in
      auv-control-OptimalHeadingController.cmd-out
      auv-control-OptimalHeadingController.orientation-samples
      auv-control-WaypointNavigator.cmd-out
      auv-control-WaypointNavigator.waypoint-info
      auv-control-WaypointNavigator.state
      auv-control-WaypointNavigator.trajectory
      auv-control-WaypointNavigator.pose-sample
      auv-control-WorldToAligned.cmd-out
      auv-control-WorldToAligned.pose-samples
      modemdriver-Modem.data-out
      modemdriver-Modem.distance
      modemdriver-Modem.out-modem-substates
      modemdriver-Modem.state
      modemdriver-Modem.data-in
      modemdriver-ModemCanbus.can-out
      modemdriver-ModemCanbus.stats
      modemdriver-ModemCanbus.can-in
  )
  (:init
      	 (task root)
      	 (is-running root)
      	 (depends root root)
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
      	 (task dynamixel-Task)
      	 (task pddl-planner-Task)
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
      	 (output-port auv-rel-pos-controller-Task.motion-command)
      	 (has-output auv-rel-pos-controller-Task auv-rel-pos-controller-Task.motion-command)
      	 (output-port auv-rel-pos-controller-Task.state)
      	 (has-output auv-rel-pos-controller-Task auv-rel-pos-controller-Task.state)
      	 (output-port pipeline-inspection-ColorFilter.frame-out)
      	 (has-output pipeline-inspection-ColorFilter pipeline-inspection-ColorFilter.frame-out)
      	 (output-port pipeline-inspection-ColorFilter.green-frame)
      	 (has-output pipeline-inspection-ColorFilter pipeline-inspection-ColorFilter.green-frame)
      	 (output-port pipeline-inspection-ColorFilter.diff-frame)
      	 (has-output pipeline-inspection-ColorFilter pipeline-inspection-ColorFilter.diff-frame)
      	 (output-port pipeline-inspection-ColorFilter.state)
      	 (has-output pipeline-inspection-ColorFilter pipeline-inspection-ColorFilter.state)
      	 (output-port pipeline-inspection-Inspection.inspectionStatus)
      	 (has-output pipeline-inspection-Inspection pipeline-inspection-Inspection.inspectionStatus)
      	 (output-port pipeline-inspection-Inspection.pipePoints)
      	 (has-output pipeline-inspection-Inspection pipeline-inspection-Inspection.pipePoints)
      	 (output-port pipeline-inspection-Inspection.debugFrame)
      	 (has-output pipeline-inspection-Inspection pipeline-inspection-Inspection.debugFrame)
      	 (output-port pipeline-inspection-Inspection.pipeMap)
      	 (has-output pipeline-inspection-Inspection pipeline-inspection-Inspection.pipeMap)
      	 (output-port pipeline-inspection-Inspection.state)
      	 (has-output pipeline-inspection-Inspection pipeline-inspection-Inspection.state)
      	 (output-port pipeline-inspection-LaserSimulation.laserPoints)
      	 (has-output pipeline-inspection-LaserSimulation pipeline-inspection-LaserSimulation.laserPoints)
      	 (output-port pipeline-inspection-LaserSimulation.laserPointCloud)
      	 (has-output pipeline-inspection-LaserSimulation pipeline-inspection-LaserSimulation.laserPointCloud)
      	 (output-port pipeline-inspection-LaserSimulation.vehiclePos)
      	 (has-output pipeline-inspection-LaserSimulation pipeline-inspection-LaserSimulation.vehiclePos)
      	 (output-port pipeline-inspection-LaserSimulation.state)
      	 (has-output pipeline-inspection-LaserSimulation pipeline-inspection-LaserSimulation.state)
      	 (output-port uw-particle-localization-FastFusion.pose-samples)
      	 (has-output uw-particle-localization-FastFusion uw-particle-localization-FastFusion.pose-samples)
      	 (output-port uw-particle-localization-FastFusion.state)
      	 (has-output uw-particle-localization-FastFusion uw-particle-localization-FastFusion.state)
      	 (output-port uw-particle-localization-MotionModel.pose-samples)
      	 (has-output uw-particle-localization-MotionModel uw-particle-localization-MotionModel.pose-samples)
      	 (output-port uw-particle-localization-MotionModel.stream-aligner-status)
      	 (has-output uw-particle-localization-MotionModel uw-particle-localization-MotionModel.stream-aligner-status)
      	 (output-port uw-particle-localization-MotionModel.state)
      	 (has-output uw-particle-localization-MotionModel uw-particle-localization-MotionModel.state)
      	 (output-port uw-particle-localization-OrientationCorrection.orientation-output)
      	 (has-output uw-particle-localization-OrientationCorrection uw-particle-localization-OrientationCorrection.orientation-output)
      	 (output-port uw-particle-localization-OrientationCorrection.orientation-offset-corrected)
      	 (has-output uw-particle-localization-OrientationCorrection uw-particle-localization-OrientationCorrection.orientation-offset-corrected)
      	 (output-port uw-particle-localization-OrientationCorrection.state)
      	 (has-output uw-particle-localization-OrientationCorrection uw-particle-localization-OrientationCorrection.state)
      	 (output-port uw-particle-localization-Task.pose-samples)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.pose-samples)
      	 (output-port uw-particle-localization-Task.environment)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.environment)
      	 (output-port uw-particle-localization-Task.dead-reckoning-samples)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.dead-reckoning-samples)
      	 (output-port uw-particle-localization-Task.full-dead-reckoning)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.full-dead-reckoning)
      	 (output-port uw-particle-localization-Task.particles)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.particles)
      	 (output-port uw-particle-localization-Task.debug-sonar-beam)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.debug-sonar-beam)
      	 (output-port uw-particle-localization-Task.stats)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.stats)
      	 (output-port uw-particle-localization-Task.depth-grid)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.depth-grid)
      	 (output-port uw-particle-localization-Task.grid-map)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.grid-map)
      	 (output-port uw-particle-localization-Task.debug-filtered-obstacles)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.debug-filtered-obstacles)
      	 (output-port uw-particle-localization-Task.stream-aligner-status)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.stream-aligner-status)
      	 (output-port uw-particle-localization-Task.state)
      	 (has-output uw-particle-localization-Task uw-particle-localization-Task.state)
      	 (output-port gps-BaseTask.solution)
      	 (has-output gps-BaseTask gps-BaseTask.solution)
      	 (output-port gps-BaseTask.position-samples)
      	 (has-output gps-BaseTask gps-BaseTask.position-samples)
      	 (output-port gps-BaseTask.state)
      	 (has-output gps-BaseTask gps-BaseTask.state)
      	 (output-port gps-MB500Task.constellation)
      	 (has-output gps-MB500Task gps-MB500Task.constellation)
      	 (output-port gps-MB500Task.time)
      	 (has-output gps-MB500Task gps-MB500Task.time)
      	 (output-port line-scanner-Task.state)
      	 (has-output line-scanner-Task line-scanner-Task.state)
      	 (output-port line-scanner-Task.pointcloud)
      	 (has-output line-scanner-Task line-scanner-Task.pointcloud)
      	 (output-port line-scanner-Task.debug)
      	 (has-output line-scanner-Task line-scanner-Task.debug)
      	 (output-port message-producer-Task.messages)
      	 (has-output message-producer-Task message-producer-Task.messages)
      	 (output-port message-producer-Task.state)
      	 (has-output message-producer-Task message-producer-Task.state)
      	 (output-port buoy-Detector.state)
      	 (has-output buoy-Detector buoy-Detector.state)
      	 (output-port buoy-Detector.buoy)
      	 (has-output buoy-Detector buoy-Detector.buoy)
      	 (output-port buoy-Detector.light)
      	 (has-output buoy-Detector buoy-Detector.light)
      	 (output-port buoy-Detector.h-image)
      	 (has-output buoy-Detector buoy-Detector.h-image)
      	 (output-port buoy-Detector.s-image)
      	 (has-output buoy-Detector buoy-Detector.s-image)
      	 (output-port buoy-Detector.v-image)
      	 (has-output buoy-Detector buoy-Detector.v-image)
      	 (output-port buoy-Detector.binary-debug-image)
      	 (has-output buoy-Detector buoy-Detector.binary-debug-image)
      	 (output-port buoy-Detector.gray-debug-image)
      	 (has-output buoy-Detector buoy-Detector.gray-debug-image)
      	 (output-port buoy-Detector.hough-debug-image)
      	 (has-output buoy-Detector buoy-Detector.hough-debug-image)
      	 (output-port buoy-Detector.other-buoys)
      	 (has-output buoy-Detector buoy-Detector.other-buoys)
      	 (output-port buoy-Detector.debug-image)
      	 (has-output buoy-Detector buoy-Detector.debug-image)
      	 (output-port buoy-Detector2.state)
      	 (has-output buoy-Detector2 buoy-Detector2.state)
      	 (output-port buoy-Detector2.buoy)
      	 (has-output buoy-Detector2 buoy-Detector2.buoy)
      	 (output-port buoy-Detector2.light)
      	 (has-output buoy-Detector2 buoy-Detector2.light)
      	 (output-port buoy-Detector2.h-image)
      	 (has-output buoy-Detector2 buoy-Detector2.h-image)
      	 (output-port buoy-Detector2.s-image)
      	 (has-output buoy-Detector2 buoy-Detector2.s-image)
      	 (output-port buoy-Detector2.v-image)
      	 (has-output buoy-Detector2 buoy-Detector2.v-image)
      	 (output-port buoy-Detector2.binary-debug-image)
      	 (has-output buoy-Detector2 buoy-Detector2.binary-debug-image)
      	 (output-port buoy-Detector2.gray-debug-image)
      	 (has-output buoy-Detector2 buoy-Detector2.gray-debug-image)
      	 (output-port buoy-Detector2.hough-debug-image)
      	 (has-output buoy-Detector2 buoy-Detector2.hough-debug-image)
      	 (output-port buoy-Detector2.other-buoys)
      	 (has-output buoy-Detector2 buoy-Detector2.other-buoys)
      	 (output-port buoy-Detector2.debug-image)
      	 (has-output buoy-Detector2 buoy-Detector2.debug-image)
      	 (output-port buoy-ServoingOnWall.world-cmd)
      	 (has-output buoy-ServoingOnWall buoy-ServoingOnWall.world-cmd)
      	 (output-port buoy-ServoingOnWall.aligned-position-cmd)
      	 (has-output buoy-ServoingOnWall buoy-ServoingOnWall.aligned-position-cmd)
      	 (output-port buoy-ServoingOnWall.state)
      	 (has-output buoy-ServoingOnWall buoy-ServoingOnWall.state)
      	 (output-port buoy-Survey.state)
      	 (has-output buoy-Survey buoy-Survey.state)
      	 (output-port buoy-Survey.strafed-angle)
      	 (has-output buoy-Survey buoy-Survey.strafed-angle)
      	 (output-port buoy-Survey.relative-position)
      	 (has-output buoy-Survey buoy-Survey.relative-position)
      	 (output-port buoy-Survey.position)
      	 (has-output buoy-Survey buoy-Survey.position)
      	 (output-port mars-Altimeter.ground-distance)
      	 (has-output mars-Altimeter mars-Altimeter.ground-distance)
      	 (output-port mars-Camera.frame)
      	 (has-output mars-Camera mars-Camera.frame)
      	 (output-port mars-DepthCamera.distance-image)
      	 (has-output mars-DepthCamera mars-DepthCamera.distance-image)
      	 (output-port mars-ForceApplier.status)
      	 (has-output mars-ForceApplier mars-ForceApplier.status)
      	 (output-port mars-ForceTorque6DOF.wrenches)
      	 (has-output mars-ForceTorque6DOF mars-ForceTorque6DOF.wrenches)
      	 (output-port mars-HighResRangeFinder.pointcloud)
      	 (has-output mars-HighResRangeFinder mars-HighResRangeFinder.pointcloud)
      	 (output-port mars-IMU.orientation-samples)
      	 (has-output mars-IMU mars-IMU.orientation-samples)
      	 (output-port mars-IMU.calibrated-sensors)
      	 (has-output mars-IMU mars-IMU.calibrated-sensors)
      	 (output-port mars-IMU.pose-samples)
      	 (has-output mars-IMU mars-IMU.pose-samples)
      	 (output-port mars-Joints.status-samples)
      	 (has-output mars-Joints mars-Joints.status-samples)
      	 (output-port mars-Joints.current-values)
      	 (has-output mars-Joints mars-Joints.current-values)
      	 (output-port mars-Joints.transforms)
      	 (has-output mars-Joints mars-Joints.transforms)
      	 (output-port mars-LaserRangeFinder.scans)
      	 (has-output mars-LaserRangeFinder mars-LaserRangeFinder.scans)
      	 (output-port mars-Plugin.state)
      	 (has-output mars-Plugin mars-Plugin.state)
      	 (output-port mars-RotatingLaserRangeFinder.pointcloud)
      	 (has-output mars-RotatingLaserRangeFinder mars-RotatingLaserRangeFinder.pointcloud)
      	 (output-port mars-Sonar.sonar-beam)
      	 (has-output mars-Sonar mars-Sonar.sonar-beam)
      	 (output-port mars-Task.time)
      	 (has-output mars-Task mars-Task.time)
      	 (output-port mars-Task.simulated-time)
      	 (has-output mars-Task mars-Task.simulated-time)
      	 (output-port mars-Task.state)
      	 (has-output mars-Task mars-Task.state)
      	 (output-port sonar-feature-estimator-Task.new-feature)
      	 (has-output sonar-feature-estimator-Task sonar-feature-estimator-Task.new-feature)
      	 (output-port sonar-feature-estimator-Task.features-out)
      	 (has-output sonar-feature-estimator-Task sonar-feature-estimator-Task.features-out)
      	 (output-port sonar-feature-estimator-Task.debug-output)
      	 (has-output sonar-feature-estimator-Task sonar-feature-estimator-Task.debug-output)
      	 (output-port sonar-feature-estimator-Task.2d-debug-output)
      	 (has-output sonar-feature-estimator-Task sonar-feature-estimator-Task.2d-debug-output)
      	 (output-port sonar-feature-estimator-Task.state)
      	 (has-output sonar-feature-estimator-Task sonar-feature-estimator-Task.state)
      	 (output-port modem-can-Task.modem-out)
      	 (has-output modem-can-Task modem-can-Task.modem-out)
      	 (output-port modem-can-Task.canOut)
      	 (has-output modem-can-Task modem-can-Task.canOut)
      	 (output-port modem-can-Task.motion-command)
      	 (has-output modem-can-Task modem-can-Task.motion-command)
      	 (output-port modem-can-Task.state)
      	 (has-output modem-can-Task modem-can-Task.state)
      	 (output-port taskmon-Task.stats)
      	 (has-output taskmon-Task taskmon-Task.stats)
      	 (output-port taskmon-Task.state)
      	 (has-output taskmon-Task taskmon-Task.state)
      	 (output-port depth-reader-Task.depthOut)
      	 (has-output depth-reader-Task depth-reader-Task.depthOut)
      	 (output-port depth-reader-Task.depth-samples)
      	 (has-output depth-reader-Task depth-reader-Task.depth-samples)
      	 (output-port depth-reader-Task.canOut)
      	 (has-output depth-reader-Task depth-reader-Task.canOut)
      	 (output-port depth-reader-Task.state)
      	 (has-output depth-reader-Task depth-reader-Task.state)
      	 (output-port raw-control-command-converter-Movement.motion-command)
      	 (has-output raw-control-command-converter-Movement raw-control-command-converter-Movement.motion-command)
      	 (output-port raw-control-command-converter-Movement.world-command)
      	 (has-output raw-control-command-converter-Movement raw-control-command-converter-Movement.world-command)
      	 (output-port raw-control-command-converter-Movement.world-command-depth)
      	 (has-output raw-control-command-converter-Movement raw-control-command-converter-Movement.world-command-depth)
      	 (output-port raw-control-command-converter-Movement.aligned-velocity-command)
      	 (has-output raw-control-command-converter-Movement raw-control-command-converter-Movement.aligned-velocity-command)
      	 (output-port raw-control-command-converter-Movement.state)
      	 (has-output raw-control-command-converter-Movement raw-control-command-converter-Movement.state)
      	 (output-port raw-control-command-converter-Position.position-command)
      	 (has-output raw-control-command-converter-Position raw-control-command-converter-Position.position-command)
      	 (output-port raw-control-command-converter-Position.world-command)
      	 (has-output raw-control-command-converter-Position raw-control-command-converter-Position.world-command)
      	 (output-port raw-control-command-converter-Position.state)
      	 (has-output raw-control-command-converter-Position raw-control-command-converter-Position.state)
      	 (output-port sonar-feature-detector-Task.features)
      	 (has-output sonar-feature-detector-Task sonar-feature-detector-Task.features)
      	 (output-port sonar-feature-detector-Task.next-target)
      	 (has-output sonar-feature-detector-Task sonar-feature-detector-Task.next-target)
      	 (output-port sonar-feature-detector-Task.next-target-feature)
      	 (has-output sonar-feature-detector-Task sonar-feature-detector-Task.next-target-feature)
      	 (output-port sonar-feature-detector-Task.next-target-command)
      	 (has-output sonar-feature-detector-Task sonar-feature-detector-Task.next-target-command)
      	 (output-port sonar-feature-detector-Task.state)
      	 (has-output sonar-feature-detector-Task sonar-feature-detector-Task.state)
      	 (output-port frame-demultiplexer-Task.oframe-pair)
      	 (has-output frame-demultiplexer-Task frame-demultiplexer-Task.oframe-pair)
      	 (output-port frame-demultiplexer-Task.oframe)
      	 (has-output frame-demultiplexer-Task frame-demultiplexer-Task.oframe)
      	 (output-port frame-demultiplexer-Task.state)
      	 (has-output frame-demultiplexer-Task frame-demultiplexer-Task.state)
      	 (output-port sonar-tritech-Echosounder.ground-distance)
      	 (has-output sonar-tritech-Echosounder sonar-tritech-Echosounder.ground-distance)
      	 (output-port sonar-tritech-Echosounder.state)
      	 (has-output sonar-tritech-Echosounder sonar-tritech-Echosounder.state)
      	 (output-port sonar-tritech-Micron.ground-distance)
      	 (has-output sonar-tritech-Micron sonar-tritech-Micron.ground-distance)
      	 (output-port sonar-tritech-Micron.sonar-beam)
      	 (has-output sonar-tritech-Micron sonar-tritech-Micron.sonar-beam)
      	 (output-port sonar-tritech-Micron.state)
      	 (has-output sonar-tritech-Micron sonar-tritech-Micron.state)
      	 (output-port sonar-tritech-Profiling.profiling-scan)
      	 (has-output sonar-tritech-Profiling sonar-tritech-Profiling.profiling-scan)
      	 (output-port sonar-tritech-Profiling.state)
      	 (has-output sonar-tritech-Profiling sonar-tritech-Profiling.state)
      	 (output-port battery-watcher-Task.battery-info)
      	 (has-output battery-watcher-Task battery-watcher-Task.battery-info)
      	 (output-port orientation-estimator-BaseEstimator.stream-aligner-status)
      	 (has-output orientation-estimator-BaseEstimator orientation-estimator-BaseEstimator.stream-aligner-status)
      	 (output-port orientation-estimator-BaseEstimator.attitude-b-g)
      	 (has-output orientation-estimator-BaseEstimator orientation-estimator-BaseEstimator.attitude-b-g)
      	 (output-port orientation-estimator-BaseEstimator.state)
      	 (has-output orientation-estimator-BaseEstimator orientation-estimator-BaseEstimator.state)
      	 (output-port orientation-estimator-IKF.transformer-stream-aligner-status)
      	 (has-output orientation-estimator-IKF orientation-estimator-IKF.transformer-stream-aligner-status)
      	 (output-port orientation-estimator-IKF.transformer-status)
      	 (has-output orientation-estimator-IKF orientation-estimator-IKF.transformer-status)
      	 (output-port orientation-estimator-IKF.attitude-b-g)
      	 (has-output orientation-estimator-IKF orientation-estimator-IKF.attitude-b-g)
      	 (output-port orientation-estimator-IKF.state)
      	 (has-output orientation-estimator-IKF orientation-estimator-IKF.state)
      	 (output-port orientation-estimator-IKFEstimator.stream-aligner-status)
      	 (has-output orientation-estimator-IKFEstimator orientation-estimator-IKFEstimator.stream-aligner-status)
      	 (output-port orientation-estimator-IKFEstimator.attitude-b-g)
      	 (has-output orientation-estimator-IKFEstimator orientation-estimator-IKFEstimator.attitude-b-g)
      	 (output-port orientation-estimator-IKFEstimator.inputs-backup)
      	 (has-output orientation-estimator-IKFEstimator orientation-estimator-IKFEstimator.inputs-backup)
      	 (output-port orientation-estimator-IKFEstimator.state)
      	 (has-output orientation-estimator-IKFEstimator orientation-estimator-IKFEstimator.state)
      	 (output-port orientation-estimator-UKFEstimator.stream-aligner-status)
      	 (has-output orientation-estimator-UKFEstimator orientation-estimator-UKFEstimator.stream-aligner-status)
      	 (output-port orientation-estimator-UKFEstimator.attitude-b-g)
      	 (has-output orientation-estimator-UKFEstimator orientation-estimator-UKFEstimator.attitude-b-g)
      	 (output-port orientation-estimator-UKFEstimator.state)
      	 (has-output orientation-estimator-UKFEstimator orientation-estimator-UKFEstimator.state)
      	 (output-port hsv-mosaicing-Task.result)
      	 (has-output hsv-mosaicing-Task hsv-mosaicing-Task.result)
      	 (output-port hsv-mosaicing-Task.state)
      	 (has-output hsv-mosaicing-Task hsv-mosaicing-Task.state)
      	 (output-port sonar-wall-hough-Task.lines)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.lines)
      	 (output-port sonar-wall-hough-Task.peaks)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.peaks)
      	 (output-port sonar-wall-hough-Task.houghspace)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.houghspace)
      	 (output-port sonar-wall-hough-Task.position)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.position)
      	 (output-port sonar-wall-hough-Task.position-quality)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.position-quality)
      	 (output-port sonar-wall-hough-Task.state)
      	 (has-output sonar-wall-hough-Task sonar-wall-hough-Task.state)
      	 (output-port pose-estimation-BaseTask.pose-samples)
      	 (has-output pose-estimation-BaseTask pose-estimation-BaseTask.pose-samples)
      	 (output-port pose-estimation-BaseTask.state)
      	 (has-output pose-estimation-BaseTask pose-estimation-BaseTask.state)
      	 (output-port pose-estimation-UWPoseEstimator.transformer-stream-aligner-status)
      	 (has-output pose-estimation-UWPoseEstimator pose-estimation-UWPoseEstimator.transformer-stream-aligner-status)
      	 (output-port pose-estimation-UWPoseEstimator.transformer-status)
      	 (has-output pose-estimation-UWPoseEstimator pose-estimation-UWPoseEstimator.transformer-status)
      	 (output-port pose-estimation-VehiclePoseEstimator.transformer-stream-aligner-status)
      	 (has-output pose-estimation-VehiclePoseEstimator pose-estimation-VehiclePoseEstimator.transformer-stream-aligner-status)
      	 (output-port pose-estimation-VehiclePoseEstimator.transformer-status)
      	 (has-output pose-estimation-VehiclePoseEstimator pose-estimation-VehiclePoseEstimator.transformer-status)
      	 (output-port auv-waypoint-navigator-Task.relative-position-command)
      	 (has-output auv-waypoint-navigator-Task auv-waypoint-navigator-Task.relative-position-command)
      	 (output-port auv-waypoint-navigator-Task.current-delta)
      	 (has-output auv-waypoint-navigator-Task auv-waypoint-navigator-Task.current-delta)
      	 (output-port auv-waypoint-navigator-Task.current-waypoint)
      	 (has-output auv-waypoint-navigator-Task auv-waypoint-navigator-Task.current-waypoint)
      	 (output-port auv-waypoint-navigator-Task.queue-size)
      	 (has-output auv-waypoint-navigator-Task auv-waypoint-navigator-Task.queue-size)
      	 (output-port auv-waypoint-navigator-Task.state)
      	 (has-output auv-waypoint-navigator-Task auv-waypoint-navigator-Task.state)
      	 (output-port gps-helper-GPSFaker.position-samples)
      	 (has-output gps-helper-GPSFaker gps-helper-GPSFaker.position-samples)
      	 (output-port gps-helper-GPSFaker.state)
      	 (has-output gps-helper-GPSFaker gps-helper-GPSFaker.state)
      	 (output-port gps-helper-MapToGPS.gps-position)
      	 (has-output gps-helper-MapToGPS gps-helper-MapToGPS.gps-position)
      	 (output-port gps-helper-MapToGPS.transformer-stream-aligner-status)
      	 (has-output gps-helper-MapToGPS gps-helper-MapToGPS.transformer-stream-aligner-status)
      	 (output-port gps-helper-MapToGPS.transformer-status)
      	 (has-output gps-helper-MapToGPS gps-helper-MapToGPS.transformer-status)
      	 (output-port gps-helper-MapToGPS.state)
      	 (has-output gps-helper-MapToGPS gps-helper-MapToGPS.state)
      	 (output-port gps-helper-WaypointNavigation.target-waypoint)
      	 (has-output gps-helper-WaypointNavigation gps-helper-WaypointNavigation.target-waypoint)
      	 (output-port gps-helper-WaypointNavigation.heading-offset)
      	 (has-output gps-helper-WaypointNavigation gps-helper-WaypointNavigation.heading-offset)
      	 (output-port gps-helper-WaypointNavigation.distance-delta)
      	 (has-output gps-helper-WaypointNavigation gps-helper-WaypointNavigation.distance-delta)
      	 (output-port gps-helper-WaypointNavigation.state)
      	 (has-output gps-helper-WaypointNavigation gps-helper-WaypointNavigation.state)
      	 (output-port avalon-simulation-AsvNavigation.state)
      	 (has-output avalon-simulation-AsvNavigation avalon-simulation-AsvNavigation.state)
      	 (output-port avalon-simulation-Modem.motion-command)
      	 (has-output avalon-simulation-Modem avalon-simulation-Modem.motion-command)
      	 (output-port avalon-simulation-Modem.state)
      	 (has-output avalon-simulation-Modem avalon-simulation-Modem.state)
      	 (output-port avalon-simulation-WhiteLight.state)
      	 (has-output avalon-simulation-WhiteLight avalon-simulation-WhiteLight.state)
      	 (output-port avalon-control-FakeWriter.motion-commands)
      	 (has-output avalon-control-FakeWriter avalon-control-FakeWriter.motion-commands)
      	 (output-port avalon-control-FakeWriter.state)
      	 (has-output avalon-control-FakeWriter avalon-control-FakeWriter.state)
      	 (output-port avalon-control-MotionControlTask.hbridge-commands)
      	 (has-output avalon-control-MotionControlTask avalon-control-MotionControlTask.hbridge-commands)
      	 (output-port avalon-control-MotionControlTask.joint-commands)
      	 (has-output avalon-control-MotionControlTask avalon-control-MotionControlTask.joint-commands)
      	 (output-port avalon-control-MotionControlTask.debug)
      	 (has-output avalon-control-MotionControlTask avalon-control-MotionControlTask.debug)
      	 (output-port avalon-control-MotionControlTask.estimated-ground-pos)
      	 (has-output avalon-control-MotionControlTask avalon-control-MotionControlTask.estimated-ground-pos)
      	 (output-port avalon-control-MotionControlTask.state)
      	 (has-output avalon-control-MotionControlTask avalon-control-MotionControlTask.state)
      	 (output-port avalon-control-MotionFeedbackTask.hbridge-status)
      	 (has-output avalon-control-MotionFeedbackTask avalon-control-MotionFeedbackTask.hbridge-status)
      	 (output-port avalon-control-MotionFeedbackTask.state)
      	 (has-output avalon-control-MotionFeedbackTask avalon-control-MotionFeedbackTask.state)
      	 (output-port avalon-control-PositionControlTask.motion-commands)
      	 (has-output avalon-control-PositionControlTask avalon-control-PositionControlTask.motion-commands)
      	 (output-port avalon-control-PositionControlTask.state)
      	 (has-output avalon-control-PositionControlTask avalon-control-PositionControlTask.state)
      	 (output-port avalon-control-RelFakeWriter.position-command)
      	 (has-output avalon-control-RelFakeWriter avalon-control-RelFakeWriter.position-command)
      	 (output-port avalon-control-RelFakeWriter.state)
      	 (has-output avalon-control-RelFakeWriter avalon-control-RelFakeWriter.state)
      	 (output-port avalon-control-TrajectoryFollower.next-position)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.next-position)
      	 (output-port avalon-control-TrajectoryFollower.position-command)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.position-command)
      	 (output-port avalon-control-TrajectoryFollower.next-pos-on-spline)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.next-pos-on-spline)
      	 (output-port avalon-control-TrajectoryFollower.last-pos-on-spline)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.last-pos-on-spline)
      	 (output-port avalon-control-TrajectoryFollower.segment-dist)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.segment-dist)
      	 (output-port avalon-control-TrajectoryFollower.world-command)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.world-command)
      	 (output-port avalon-control-TrajectoryFollower.state)
      	 (has-output avalon-control-TrajectoryFollower avalon-control-TrajectoryFollower.state)
      	 (output-port hbridge-CommandWriter.state)
      	 (has-output hbridge-CommandWriter hbridge-CommandWriter.state)
      	 (output-port hbridge-CommandWriter.can-out)
      	 (has-output hbridge-CommandWriter hbridge-CommandWriter.can-out)
      	 (output-port hbridge-CommandWriter.speedCtrlDebug)
      	 (has-output hbridge-CommandWriter hbridge-CommandWriter.speedCtrlDebug)
      	 (output-port hbridge-CommandWriter.fakeReader)
      	 (has-output hbridge-CommandWriter hbridge-CommandWriter.fakeReader)
      	 (output-port hbridge-SensorReader.state)
      	 (has-output hbridge-SensorReader hbridge-SensorReader.state)
      	 (output-port hbridge-SensorReader.can-out)
      	 (has-output hbridge-SensorReader hbridge-SensorReader.can-out)
      	 (output-port hbridge-SensorReader.status-samples)
      	 (has-output hbridge-SensorReader hbridge-SensorReader.status-samples)
      	 (output-port sonar-structure-servoing-Task.position-command)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.position-command)
      	 (output-port sonar-structure-servoing-Task.aligned-position-command)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.aligned-position-command)
      	 (output-port sonar-structure-servoing-Task.world-command)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.world-command)
      	 (output-port sonar-structure-servoing-Task.debug-data)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.debug-data)
      	 (output-port sonar-structure-servoing-Task.transformer-stream-aligner-status)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.transformer-stream-aligner-status)
      	 (output-port sonar-structure-servoing-Task.transformer-status)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.transformer-status)
      	 (output-port sonar-structure-servoing-Task.state)
      	 (has-output sonar-structure-servoing-Task sonar-structure-servoing-Task.state)
      	 (output-port depth-map-LaserToPoints.pointcloud)
      	 (has-output depth-map-LaserToPoints depth-map-LaserToPoints.pointcloud)
      	 (output-port depth-map-LaserToPoints.state)
      	 (has-output depth-map-LaserToPoints depth-map-LaserToPoints.state)
      	 (output-port depth-map-Task.output)
      	 (has-output depth-map-Task depth-map-Task.output)
      	 (output-port depth-map-Task.state)
      	 (has-output depth-map-Task depth-map-Task.state)
      	 (output-port depth-map-frame-export.frames)
      	 (has-output depth-map-frame-export depth-map-frame-export.frames)
      	 (output-port depth-map-frame-export.minZ)
      	 (has-output depth-map-frame-export depth-map-frame-export.minZ)
      	 (output-port depth-map-frame-export.maxZ)
      	 (has-output depth-map-frame-export depth-map-frame-export.maxZ)
      	 (output-port depth-map-frame-export.state)
      	 (has-output depth-map-frame-export depth-map-frame-export.state)
      	 (output-port depth-map-sonar-ground-distance.new-feature)
      	 (has-output depth-map-sonar-ground-distance depth-map-sonar-ground-distance.new-feature)
      	 (output-port depth-map-sonar-ground-distance.state)
      	 (has-output depth-map-sonar-ground-distance depth-map-sonar-ground-distance.state)
      	 (output-port wall-servoing-DualSonarServoing.position-command)
      	 (has-output wall-servoing-DualSonarServoing wall-servoing-DualSonarServoing.position-command)
      	 (output-port wall-servoing-DualSonarServoing.aligned-command)
      	 (has-output wall-servoing-DualSonarServoing wall-servoing-DualSonarServoing.aligned-command)
      	 (output-port wall-servoing-DualSonarServoing.wall-servoing-debug)
      	 (has-output wall-servoing-DualSonarServoing wall-servoing-DualSonarServoing.wall-servoing-debug)
      	 (output-port wall-servoing-SingleSonarServoing.position-command)
      	 (has-output wall-servoing-SingleSonarServoing wall-servoing-SingleSonarServoing.position-command)
      	 (output-port wall-servoing-SingleSonarServoing.aligned-position-command)
      	 (has-output wall-servoing-SingleSonarServoing wall-servoing-SingleSonarServoing.aligned-position-command)
      	 (output-port wall-servoing-SingleSonarServoing.world-command)
      	 (has-output wall-servoing-SingleSonarServoing wall-servoing-SingleSonarServoing.world-command)
      	 (output-port wall-servoing-SingleSonarServoing.wall-servoing-debug)
      	 (has-output wall-servoing-SingleSonarServoing wall-servoing-SingleSonarServoing.wall-servoing-debug)
      	 (output-port wall-servoing-SingleSonarServoing.wall)
      	 (has-output wall-servoing-SingleSonarServoing wall-servoing-SingleSonarServoing.wall)
      	 (output-port wall-servoing-SonarServoing.state)
      	 (has-output wall-servoing-SonarServoing wall-servoing-SonarServoing.state)
      	 (output-port wall-servoing-WallDetector.point-cloud)
      	 (has-output wall-servoing-WallDetector wall-servoing-WallDetector.point-cloud)
      	 (output-port wall-servoing-WallDetector.wall)
      	 (has-output wall-servoing-WallDetector wall-servoing-WallDetector.wall)
      	 (output-port wall-servoing-WallDetector.state)
      	 (has-output wall-servoing-WallDetector wall-servoing-WallDetector.state)
      	 (output-port wall-servoing-WallServoing.motion-command)
      	 (has-output wall-servoing-WallServoing wall-servoing-WallServoing.motion-command)
      	 (output-port wall-servoing-WallServoing.world-command)
      	 (has-output wall-servoing-WallServoing wall-servoing-WallServoing.world-command)
      	 (output-port wall-servoing-WallServoing.aligned-velocity-command)
      	 (has-output wall-servoing-WallServoing wall-servoing-WallServoing.aligned-velocity-command)
      	 (output-port wall-servoing-WallServoing.state)
      	 (has-output wall-servoing-WallServoing wall-servoing-WallServoing.state)
      	 (output-port camera-unicap-CameraTask.frame)
      	 (has-output camera-unicap-CameraTask camera-unicap-CameraTask.frame)
      	 (output-port camera-unicap-CameraTask.state)
      	 (has-output camera-unicap-CameraTask camera-unicap-CameraTask.state)
      	 (output-port task-scheduler-Task.state)
      	 (has-output task-scheduler-Task task-scheduler-Task.state)
      	 (output-port sonar-blueview-Task.frame)
      	 (has-output sonar-blueview-Task sonar-blueview-Task.frame)
      	 (output-port sonar-blueview-Task.state)
      	 (has-output sonar-blueview-Task sonar-blueview-Task.state)
      	 (output-port auv-helper-depth-and-orientation-fusion.pose-samples)
      	 (has-output auv-helper-depth-and-orientation-fusion auv-helper-depth-and-orientation-fusion.pose-samples)
      	 (output-port auv-helper-depth-and-orientation-fusion.stream-aligner-status)
      	 (has-output auv-helper-depth-and-orientation-fusion auv-helper-depth-and-orientation-fusion.stream-aligner-status)
      	 (output-port auv-helper-depth-and-orientation-fusion.state)
      	 (has-output auv-helper-depth-and-orientation-fusion auv-helper-depth-and-orientation-fusion.state)
      	 (output-port sysmon-Task.ocu-markers)
      	 (has-output sysmon-Task sysmon-Task.ocu-markers)
      	 (output-port sysmon-Task.annotations)
      	 (has-output sysmon-Task sysmon-Task.annotations)
      	 (output-port sysmon-Task.system-status)
      	 (has-output sysmon-Task sysmon-Task.system-status)
      	 (output-port sysmon-Task.state)
      	 (has-output sysmon-Task sysmon-Task.state)
      	 (output-port logger-Logger.state)
      	 (has-output logger-Logger logger-Logger.state)
      	 (output-port fog-kvh-Dsp3000Task.rotation)
      	 (has-output fog-kvh-Dsp3000Task fog-kvh-Dsp3000Task.rotation)
      	 (output-port fog-kvh-Dsp3000Task.orientation-samples)
      	 (has-output fog-kvh-Dsp3000Task fog-kvh-Dsp3000Task.orientation-samples)
      	 (output-port fog-kvh-Dsp3000Task.timestamp-estimator-status)
      	 (has-output fog-kvh-Dsp3000Task fog-kvh-Dsp3000Task.timestamp-estimator-status)
      	 (output-port fog-kvh-Dsp3000Task.state)
      	 (has-output fog-kvh-Dsp3000Task fog-kvh-Dsp3000Task.state)
      	 (output-port rear-sonar-distance-estimator-Task.ground-distance)
      	 (has-output rear-sonar-distance-estimator-Task rear-sonar-distance-estimator-Task.ground-distance)
      	 (output-port rear-sonar-distance-estimator-Task.state)
      	 (has-output rear-sonar-distance-estimator-Task rear-sonar-distance-estimator-Task.state)
      	 (output-port transformer-Task.configuration-state)
      	 (has-output transformer-Task transformer-Task.configuration-state)
      	 (output-port transformer-Task.state)
      	 (has-output transformer-Task transformer-Task.state)
      	 (output-port camera-base-Task.frame)
      	 (has-output camera-base-Task camera-base-Task.frame)
      	 (output-port camera-base-Task.frame-raw)
      	 (has-output camera-base-Task camera-base-Task.frame-raw)
      	 (output-port camera-base-Task.state)
      	 (has-output camera-base-Task camera-base-Task.state)
      	 (output-port low-level-driver-LowLevelTask.state)
      	 (has-output low-level-driver-LowLevelTask low-level-driver-LowLevelTask.state)
      	 (output-port wall-orientation-correction-OrientationInMap.orientation-in-map)
      	 (has-output wall-orientation-correction-OrientationInMap wall-orientation-correction-OrientationInMap.orientation-in-map)
      	 (output-port wall-orientation-correction-OrientationInMap.transformer-stream-aligner-status)
      	 (has-output wall-orientation-correction-OrientationInMap wall-orientation-correction-OrientationInMap.transformer-stream-aligner-status)
      	 (output-port wall-orientation-correction-OrientationInMap.transformer-status)
      	 (has-output wall-orientation-correction-OrientationInMap wall-orientation-correction-OrientationInMap.transformer-status)
      	 (output-port wall-orientation-correction-OrientationInMap.state)
      	 (has-output wall-orientation-correction-OrientationInMap wall-orientation-correction-OrientationInMap.state)
      	 (output-port wall-orientation-correction-Task.orientation-in-world)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.orientation-in-world)
      	 (output-port wall-orientation-correction-Task.angle-in-world)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.angle-in-world)
      	 (output-port wall-orientation-correction-Task.debug-data)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.debug-data)
      	 (output-port wall-orientation-correction-Task.transformer-stream-aligner-status)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.transformer-stream-aligner-status)
      	 (output-port wall-orientation-correction-Task.transformer-status)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.transformer-status)
      	 (output-port wall-orientation-correction-Task.state)
      	 (has-output wall-orientation-correction-Task wall-orientation-correction-Task.state)
      	 (output-port structure-servoing-Alignment.world-command)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.world-command)
      	 (output-port structure-servoing-Alignment.aligned-speed-command)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.aligned-speed-command)
      	 (output-port structure-servoing-Alignment.left)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.left)
      	 (output-port structure-servoing-Alignment.right)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.right)
      	 (output-port structure-servoing-Alignment.top)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.top)
      	 (output-port structure-servoing-Alignment.bottom)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.bottom)
      	 (output-port structure-servoing-Alignment.size)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.size)
      	 (output-port structure-servoing-Alignment.x)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.x)
      	 (output-port structure-servoing-Alignment.y)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.y)
      	 (output-port structure-servoing-Alignment.state)
      	 (has-output structure-servoing-Alignment structure-servoing-Alignment.state)
      	 (output-port structure-servoing-Task.servoed-angle)
      	 (has-output structure-servoing-Task structure-servoing-Task.servoed-angle)
      	 (output-port structure-servoing-Task.angle-speed)
      	 (has-output structure-servoing-Task structure-servoing-Task.angle-speed)
      	 (output-port structure-servoing-Task.world-command)
      	 (has-output structure-servoing-Task structure-servoing-Task.world-command)
      	 (output-port structure-servoing-Task.aligned-speed-command)
      	 (has-output structure-servoing-Task structure-servoing-Task.aligned-speed-command)
      	 (output-port structure-servoing-Task.found-structure)
      	 (has-output structure-servoing-Task structure-servoing-Task.found-structure)
      	 (output-port structure-servoing-Task.state)
      	 (has-output structure-servoing-Task structure-servoing-Task.state)
      	 (output-port structure-servoing-Task.left)
      	 (has-output structure-servoing-Task structure-servoing-Task.left)
      	 (output-port structure-servoing-Task.right)
      	 (has-output structure-servoing-Task structure-servoing-Task.right)
      	 (output-port structure-servoing-Task.top)
      	 (has-output structure-servoing-Task structure-servoing-Task.top)
      	 (output-port structure-servoing-Task.bottom)
      	 (has-output structure-servoing-Task structure-servoing-Task.bottom)
      	 (output-port structure-servoing-Task.size)
      	 (has-output structure-servoing-Task structure-servoing-Task.size)
      	 (output-port structure-servoing-Task.heading)
      	 (has-output structure-servoing-Task structure-servoing-Task.heading)
      	 (output-port structure-servoing-Task.cnt-left)
      	 (has-output structure-servoing-Task structure-servoing-Task.cnt-left)
      	 (output-port structure-servoing-Task.cnt-right)
      	 (has-output structure-servoing-Task structure-servoing-Task.cnt-right)
      	 (output-port structure-servoing-Task.cnt-top)
      	 (has-output structure-servoing-Task structure-servoing-Task.cnt-top)
      	 (output-port structure-servoing-Task.cnt-bottom)
      	 (has-output structure-servoing-Task structure-servoing-Task.cnt-bottom)
      	 (output-port image-preprocessing-BaseTask.state)
      	 (has-output image-preprocessing-BaseTask image-preprocessing-BaseTask.state)
      	 (output-port image-preprocessing-DepthImage2Pointcloud.pointcloud)
      	 (has-output image-preprocessing-DepthImage2Pointcloud image-preprocessing-DepthImage2Pointcloud.pointcloud)
      	 (output-port image-preprocessing-DepthImage2Pointcloud.stream-aligner-status)
      	 (has-output image-preprocessing-DepthImage2Pointcloud image-preprocessing-DepthImage2Pointcloud.stream-aligner-status)
      	 (output-port image-preprocessing-DepthImage2Pointcloud.state)
      	 (has-output image-preprocessing-DepthImage2Pointcloud image-preprocessing-DepthImage2Pointcloud.state)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.oframe)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.oframe)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.binary-result)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.binary-result)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.hDebug)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.hDebug)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.hDebugGray)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.hDebugGray)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.vDebug)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.vDebug)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.vDebugGray)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.vDebugGray)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.sDebug)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.sDebug)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.sDebugGray)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.sDebugGray)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.hsv-v-frame)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.hsv-v-frame)
      	 (output-port image-preprocessing-HSVSegmentationAndBlur.state)
      	 (has-output image-preprocessing-HSVSegmentationAndBlur image-preprocessing-HSVSegmentationAndBlur.state)
      	 (output-port image-preprocessing-MonoTask.oframe)
      	 (has-output image-preprocessing-MonoTask image-preprocessing-MonoTask.oframe)
      	 (output-port image-preprocessing-StereoTask.oframe-pair)
      	 (has-output image-preprocessing-StereoTask image-preprocessing-StereoTask.oframe-pair)
      	 (output-port offshore-pipeline-detector-SonarDetector.frame)
      	 (has-output offshore-pipeline-detector-SonarDetector offshore-pipeline-detector-SonarDetector.frame)
      	 (output-port offshore-pipeline-detector-SonarDetector.state)
      	 (has-output offshore-pipeline-detector-SonarDetector offshore-pipeline-detector-SonarDetector.state)
      	 (output-port offshore-pipeline-detector-Task.state)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.state)
      	 (output-port offshore-pipeline-detector-Task.pipeline)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.pipeline)
      	 (output-port offshore-pipeline-detector-Task.world-command)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.world-command)
      	 (output-port offshore-pipeline-detector-Task.aligned-position-command)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.aligned-position-command)
      	 (output-port offshore-pipeline-detector-Task.position-command)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.position-command)
      	 (output-port offshore-pipeline-detector-Task.debug)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.debug)
      	 (output-port offshore-pipeline-detector-Task.debug-frame)
      	 (has-output offshore-pipeline-detector-Task offshore-pipeline-detector-Task.debug-frame)
      	 (output-port interfaces-ActuatorCommandWriter.state)
      	 (has-output interfaces-ActuatorCommandWriter interfaces-ActuatorCommandWriter.state)
      	 (output-port interfaces-ActuatorSensorReader.state)
      	 (has-output interfaces-ActuatorSensorReader interfaces-ActuatorSensorReader.state)
      	 (output-port interfaces-IMU.orientation-samples)
      	 (has-output interfaces-IMU interfaces-IMU.orientation-samples)
      	 (output-port interfaces-IMU.calibrated-sensors)
      	 (has-output interfaces-IMU interfaces-IMU.calibrated-sensors)
      	 (output-port interfaces-IMU.state)
      	 (has-output interfaces-IMU interfaces-IMU.state)
      	 (output-port interfaces-LaserRangeFinder.scans)
      	 (has-output interfaces-LaserRangeFinder interfaces-LaserRangeFinder.scans)
      	 (output-port interfaces-LaserRangeFinder.state)
      	 (has-output interfaces-LaserRangeFinder interfaces-LaserRangeFinder.state)
      	 (output-port interfaces-Servo.upper2lower)
      	 (has-output interfaces-Servo interfaces-Servo.upper2lower)
      	 (output-port interfaces-Servo.angle)
      	 (has-output interfaces-Servo interfaces-Servo.angle)
      	 (output-port interfaces-Servo.state)
      	 (has-output interfaces-Servo interfaces-Servo.state)
      	 (output-port canbus-InterfaceTask.can-out)
      	 (has-output canbus-InterfaceTask canbus-InterfaceTask.can-out)
      	 (output-port canbus-InterfaceTask.state)
      	 (has-output canbus-InterfaceTask canbus-InterfaceTask.state)
      	 (output-port canbus-Task.stats)
      	 (has-output canbus-Task canbus-Task.stats)
      	 (output-port canbus-Task.can-status)
      	 (has-output canbus-Task canbus-Task.can-status)
      	 (output-port canbus-Task.log-message)
      	 (has-output canbus-Task canbus-Task.log-message)
      	 (output-port canbus-Task.state)
      	 (has-output canbus-Task canbus-Task.state)
      	 (output-port structured-light-Calibration.stream-aligner-status)
      	 (has-output structured-light-Calibration structured-light-Calibration.stream-aligner-status)
      	 (output-port structured-light-Calibration.state)
      	 (has-output structured-light-Calibration structured-light-Calibration.state)
      	 (output-port structured-light-Task.laser-scan)
      	 (has-output structured-light-Task structured-light-Task.laser-scan)
      	 (output-port structured-light-Task.candidates)
      	 (has-output structured-light-Task structured-light-Task.candidates)
      	 (output-port structured-light-Task.debug-frame)
      	 (has-output structured-light-Task structured-light-Task.debug-frame)
      	 (output-port structured-light-Task.state)
      	 (has-output structured-light-Task structured-light-Task.state)
      	 (output-port xsens-imu-Task.orientation-samples)
      	 (has-output xsens-imu-Task xsens-imu-Task.orientation-samples)
      	 (output-port xsens-imu-Task.calibrated-sensors)
      	 (has-output xsens-imu-Task xsens-imu-Task.calibrated-sensors)
      	 (output-port xsens-imu-Task.timestamp-estimator-status)
      	 (has-output xsens-imu-Task xsens-imu-Task.timestamp-estimator-status)
      	 (output-port xsens-imu-Task.state)
      	 (has-output xsens-imu-Task xsens-imu-Task.state)
      	 (output-port dynamixel-Task.lowerDynamixel2UpperDynamixel)
      	 (has-output dynamixel-Task dynamixel-Task.lowerDynamixel2UpperDynamixel)
      	 (output-port pddl-planner-Task.state)
      	 (has-output pddl-planner-Task pddl-planner-Task.state)
      	 (output-port video-streamer-vlc-Capturer.state)
      	 (has-output video-streamer-vlc-Capturer video-streamer-vlc-Capturer.state)
      	 (output-port video-streamer-vlc-Streamer.state)
      	 (has-output video-streamer-vlc-Streamer video-streamer-vlc-Streamer.state)
      	 (output-port controldev-GenericRawToMotion2D.motion-command)
      	 (has-output controldev-GenericRawToMotion2D controldev-GenericRawToMotion2D.motion-command)
      	 (output-port controldev-GenericRawToMotion2D.state)
      	 (has-output controldev-GenericRawToMotion2D controldev-GenericRawToMotion2D.state)
      	 (output-port controldev-GenericTask.raw-command)
      	 (has-output controldev-GenericTask controldev-GenericTask.raw-command)
      	 (output-port controldev-GenericTask.state)
      	 (has-output controldev-GenericTask controldev-GenericTask.state)
      	 (output-port controldev-RawJoystickToMotion2D.motion-command)
      	 (has-output controldev-RawJoystickToMotion2D controldev-RawJoystickToMotion2D.motion-command)
      	 (output-port controldev-RawJoystickToMotion2D.state)
      	 (has-output controldev-RawJoystickToMotion2D controldev-RawJoystickToMotion2D.state)
      	 (output-port controldev-RawWheelToMotion2D.motion-command)
      	 (has-output controldev-RawWheelToMotion2D controldev-RawWheelToMotion2D.motion-command)
      	 (output-port controldev-RawWheelToMotion2D.state)
      	 (has-output controldev-RawWheelToMotion2D controldev-RawWheelToMotion2D.state)
      	 (output-port structure-reconstruction-Task.transformer-stream-aligner-status)
      	 (has-output structure-reconstruction-Task structure-reconstruction-Task.transformer-stream-aligner-status)
      	 (output-port structure-reconstruction-Task.transformer-status)
      	 (has-output structure-reconstruction-Task structure-reconstruction-Task.transformer-status)
      	 (output-port structure-reconstruction-Task.state)
      	 (has-output structure-reconstruction-Task structure-reconstruction-Task.state)
      	 (output-port lights-Lights.light-value)
      	 (has-output lights-Lights lights-Lights.light-value)
      	 (output-port lights-Lights.state)
      	 (has-output lights-Lights lights-Lights.state)
      	 (output-port auv-control-AccelerationController.cmd-out)
      	 (has-output auv-control-AccelerationController auv-control-AccelerationController.cmd-out)
      	 (output-port auv-control-AlignedToBody.cmd-out)
      	 (has-output auv-control-AlignedToBody auv-control-AlignedToBody.cmd-out)
      	 (output-port auv-control-Base.state)
      	 (has-output auv-control-Base auv-control-Base.state)
      	 (output-port auv-control-BasePIDController.cmd-out)
      	 (has-output auv-control-BasePIDController auv-control-BasePIDController.cmd-out)
      	 (output-port auv-control-BasePIDController.pid-state)
      	 (has-output auv-control-BasePIDController auv-control-BasePIDController.pid-state)
      	 (output-port auv-control-ConstantCommand.cmd-out)
      	 (has-output auv-control-ConstantCommand auv-control-ConstantCommand.cmd-out)
      	 (output-port auv-control-ConstantCommand.state)
      	 (has-output auv-control-ConstantCommand auv-control-ConstantCommand.state)
      	 (output-port auv-control-ConstantCommandGroundFollower.floor-position)
      	 (has-output auv-control-ConstantCommandGroundFollower auv-control-ConstantCommandGroundFollower.floor-position)
      	 (output-port auv-control-ConstantCommandGroundFollower.state)
      	 (has-output auv-control-ConstantCommandGroundFollower auv-control-ConstantCommandGroundFollower.state)
      	 (output-port auv-control-ConstantCommandGroundFollower.cmd-out)
      	 (has-output auv-control-ConstantCommandGroundFollower auv-control-ConstantCommandGroundFollower.cmd-out)
      	 (output-port auv-control-MotionCommand2DConverter.cmd-out)
      	 (has-output auv-control-MotionCommand2DConverter auv-control-MotionCommand2DConverter.cmd-out)
      	 (output-port auv-control-MotionCommand2DConverter.state)
      	 (has-output auv-control-MotionCommand2DConverter auv-control-MotionCommand2DConverter.state)
      	 (output-port auv-control-OptimalHeadingController.cmd-out)
      	 (has-output auv-control-OptimalHeadingController auv-control-OptimalHeadingController.cmd-out)
      	 (output-port auv-control-WaypointNavigator.cmd-out)
      	 (has-output auv-control-WaypointNavigator auv-control-WaypointNavigator.cmd-out)
      	 (output-port auv-control-WaypointNavigator.waypoint-info)
      	 (has-output auv-control-WaypointNavigator auv-control-WaypointNavigator.waypoint-info)
      	 (output-port auv-control-WaypointNavigator.state)
      	 (has-output auv-control-WaypointNavigator auv-control-WaypointNavigator.state)
      	 (output-port auv-control-WorldToAligned.cmd-out)
      	 (has-output auv-control-WorldToAligned auv-control-WorldToAligned.cmd-out)
      	 (output-port modemdriver-Modem.data-out)
      	 (has-output modemdriver-Modem modemdriver-Modem.data-out)
      	 (output-port modemdriver-Modem.distance)
      	 (has-output modemdriver-Modem modemdriver-Modem.distance)
      	 (output-port modemdriver-Modem.out-modem-substates)
      	 (has-output modemdriver-Modem modemdriver-Modem.out-modem-substates)
      	 (output-port modemdriver-Modem.state)
      	 (has-output modemdriver-Modem modemdriver-Modem.state)
      	 (output-port modemdriver-ModemCanbus.can-out)
      	 (has-output modemdriver-ModemCanbus modemdriver-ModemCanbus.can-out)
      	 (output-port modemdriver-ModemCanbus.stats)
      	 (has-output modemdriver-ModemCanbus modemdriver-ModemCanbus.stats)
  )
  (:goal (and
      	 (is-running root)
      	 (depends root lights-Lights)
  ))
)
