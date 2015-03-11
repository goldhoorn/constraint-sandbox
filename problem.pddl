(define (problem network001)
  (:domain network)
  (:objects
        root - instance_req
        AuvControl::WorldToAligned.state - output_port
        AuvControl::WorldToAligned.cmd_out - output_port
        AuvControl::WorldToAligned.cmd_in - input_port
        AuvControl::WorldToAligned.cmd_cascade - input_port
        AuvControl::WorldToAligned.pose_samples - input_port
        AuvControl::WorldToAligned - instance_req
        WallServoing::SingleSonarServoing.state - output_port
        WallServoing::SingleSonarServoing.position_command - output_port
        WallServoing::SingleSonarServoing.aligned_position_command - output_port
        WallServoing::SingleSonarServoing.world_command - output_port
        WallServoing::SingleSonarServoing.wall_servoing_debug - output_port
        WallServoing::SingleSonarServoing.wall - output_port
        WallServoing::SingleSonarServoing.sonarbeam_feature - input_port
        WallServoing::SingleSonarServoing.orientation_sample - input_port
        WallServoing::SingleSonarServoing.position_sample - input_port
        WallServoing::SingleSonarServoing - instance_req
        Syskit::ROS::Node - instance_req
        AuvControl::PIDController.state - output_port
        AuvControl::PIDController.cmd_out - output_port
        AuvControl::PIDController.pid_state - output_port
        AuvControl::PIDController.cmd_in - input_port
        AuvControl::PIDController.cmd_cascade - input_port
        AuvControl::PIDController.pose_samples - input_port
        AuvControl::PIDController - instance_req
        WallServoing::SonarServoing.state - output_port
        WallServoing::SonarServoing - instance_req
        RTT::TaskContext - instance_req
        Syskit::RubyTaskContext - instance_req
        AvalonControl::MotionFeedbackTask.hbridge_status - output_port
        AvalonControl::MotionFeedbackTask.state - output_port
        AvalonControl::MotionFeedbackTask.hbridge_feedback - input_port
        AvalonControl::MotionFeedbackTask - instance_req
        WallOrientationCorrection::OrientationInMap.orientation_in_map - output_port
        WallOrientationCorrection::OrientationInMap.transformer_stream_aligner_status - output_port
        WallOrientationCorrection::OrientationInMap.transformer_status - output_port
        WallOrientationCorrection::OrientationInMap.state - output_port
        WallOrientationCorrection::OrientationInMap.orientation_in_world - input_port
        WallOrientationCorrection::OrientationInMap.dynamic_transformations - input_port
        WallOrientationCorrection::OrientationInMap - instance_req
        WallServoing::WallServoing.motion_command - output_port
        WallServoing::WallServoing.world_command - output_port
        WallServoing::WallServoing.aligned_velocity_command - output_port
        WallServoing::WallServoing.state - output_port
        WallServoing::WallServoing.orientation_sample - input_port
        WallServoing::WallServoing.servoing_wall - input_port
        WallServoing::WallServoing.obstacle_wall - input_port
        WallServoing::WallServoing - instance_req
        AuvControl::BasePIDController.state - output_port
        AuvControl::BasePIDController.cmd_out - output_port
        AuvControl::BasePIDController.pid_state - output_port
        AuvControl::BasePIDController.cmd_in - input_port
        AuvControl::BasePIDController.cmd_cascade - input_port
        AuvControl::BasePIDController.pose_samples - input_port
        AuvControl::BasePIDController - instance_req
        WallOrientationCorrection::Task.orientation_in_world - output_port
        WallOrientationCorrection::Task.angle_in_world - output_port
        WallOrientationCorrection::Task.debug_data - output_port
        WallOrientationCorrection::Task.transformer_stream_aligner_status - output_port
        WallOrientationCorrection::Task.transformer_status - output_port
        WallOrientationCorrection::Task.state - output_port
        WallOrientationCorrection::Task.sonarbeam_feature - input_port
        WallOrientationCorrection::Task.orientation_samples - input_port
        WallOrientationCorrection::Task.dynamic_transformations - input_port
        WallOrientationCorrection::Task - instance_req
        WallServoing::WallDetector.point_cloud - output_port
        WallServoing::WallDetector.wall - output_port
        WallServoing::WallDetector.state - output_port
        WallServoing::WallDetector.sonarbeam_feature - input_port
        WallServoing::WallDetector.orientation_sample - input_port
        WallServoing::WallDetector.position_sample - input_port
        WallServoing::WallDetector - instance_req
        AvalonControl::TrajectoryFollower.next_position - output_port
        AvalonControl::TrajectoryFollower.position_command - output_port
        AvalonControl::TrajectoryFollower.next_pos_on_spline - output_port
        AvalonControl::TrajectoryFollower.last_pos_on_spline - output_port
        AvalonControl::TrajectoryFollower.segment_dist - output_port
        AvalonControl::TrajectoryFollower.world_command - output_port
        AvalonControl::TrajectoryFollower.state - output_port
        AvalonControl::TrajectoryFollower.pose_samples - input_port
        AvalonControl::TrajectoryFollower - instance_req
        FogKvh::Dsp3000Task.rotation - output_port
        FogKvh::Dsp3000Task.orientation_samples - output_port
        FogKvh::Dsp3000Task.timestamp_estimator_status - output_port
        FogKvh::Dsp3000Task.state - output_port
        FogKvh::Dsp3000Task.config - input_port
        FogKvh::Dsp3000Task - instance_req
        ModemCan::Task.modem_out - output_port
        ModemCan::Task.canOut - output_port
        ModemCan::Task.motion_command - output_port
        ModemCan::Task.state - output_port
        ModemCan::Task.modem_in - input_port
        ModemCan::Task.canModem - input_port
        ModemCan::Task.light_value - input_port
        ModemCan::Task.position_samples - input_port
        ModemCan::Task - instance_req
        Transformer::Task.configuration_state - output_port
        Transformer::Task.state - output_port
        Transformer::Task - instance_req
        SonarStructureServoing::Task.position_command - output_port
        SonarStructureServoing::Task.aligned_position_command - output_port
        SonarStructureServoing::Task.world_command - output_port
        SonarStructureServoing::Task.debug_data - output_port
        SonarStructureServoing::Task.transformer_stream_aligner_status - output_port
        SonarStructureServoing::Task.transformer_status - output_port
        SonarStructureServoing::Task.state - output_port
        SonarStructureServoing::Task.sonarbeam_feature - input_port
        SonarStructureServoing::Task.odometry_samples - input_port
        SonarStructureServoing::Task.dynamic_transformations - input_port
        SonarStructureServoing::Task - instance_req
        AuvControl::Base.state - output_port
        AuvControl::Base.cmd_in - input_port
        AuvControl::Base.cmd_cascade - input_port
        AuvControl::Base - instance_req
        AvalonControl::RelFakeWriter.position_command - output_port
        AvalonControl::RelFakeWriter.state - output_port
        AvalonControl::RelFakeWriter - instance_req
        SonarWallHough::Task.lines - output_port
        SonarWallHough::Task.peaks - output_port
        SonarWallHough::Task.houghspace - output_port
        SonarWallHough::Task.position - output_port
        SonarWallHough::Task.position_quality - output_port
        SonarWallHough::Task.state - output_port
        SonarWallHough::Task.sonar_samples - input_port
        SonarWallHough::Task.reset - input_port
        SonarWallHough::Task.orientation_samples - input_port
        SonarWallHough::Task.pose_samples - input_port
        SonarWallHough::Task - instance_req
        PoseEstimation::UWPoseEstimator.pose_samples - output_port
        PoseEstimation::UWPoseEstimator.state - output_port
        PoseEstimation::UWPoseEstimator.transformer_stream_aligner_status - output_port
        PoseEstimation::UWPoseEstimator.transformer_status - output_port
        PoseEstimation::UWPoseEstimator.orientation_samples - input_port
        PoseEstimation::UWPoseEstimator.depth_samples - input_port
        PoseEstimation::UWPoseEstimator.dvl_velocity_samples - input_port
        PoseEstimation::UWPoseEstimator.model_velocity_samples - input_port
        PoseEstimation::UWPoseEstimator.lbl_position_samples - input_port
        PoseEstimation::UWPoseEstimator.xy_position_samples - input_port
        PoseEstimation::UWPoseEstimator.dynamic_transformations - input_port
        PoseEstimation::UWPoseEstimator - instance_req
        RearSonarDistanceEstimator::Task.ground_distance - output_port
        RearSonarDistanceEstimator::Task.state - output_port
        RearSonarDistanceEstimator::Task.BaseScan - input_port
        RearSonarDistanceEstimator::Task.depth_samples - input_port
        RearSonarDistanceEstimator::Task - instance_req
        Logger::Logger.state - output_port
        Logger::Logger - instance_req
        DepthReader::Task.depthOut - output_port
        DepthReader::Task.depth_samples - output_port
        DepthReader::Task.canOut - output_port
        DepthReader::Task.state - output_port
        DepthReader::Task.canIn - input_port
        DepthReader::Task - instance_req
        AvalonControl::FakeWriter.motion_commands - output_port
        AvalonControl::FakeWriter.state - output_port
        AvalonControl::FakeWriter - instance_req
        PoseEstimation::BaseTask.pose_samples - output_port
        PoseEstimation::BaseTask.state - output_port
        PoseEstimation::BaseTask - instance_req
        AvalonControl::MotionControlTask.hbridge_commands - output_port
        AvalonControl::MotionControlTask.joint_commands - output_port
        AvalonControl::MotionControlTask.debug - output_port
        AvalonControl::MotionControlTask.estimated_ground_pos - output_port
        AvalonControl::MotionControlTask.state - output_port
        AvalonControl::MotionControlTask.dummy_feedback - input_port
        AvalonControl::MotionControlTask.ground_distance - input_port
        AvalonControl::MotionControlTask.pose_samples - input_port
        AvalonControl::MotionControlTask.motion_commands - input_port
        AvalonControl::MotionControlTask.joints_in - input_port
        AvalonControl::MotionControlTask - instance_req
        Canbus::InterfaceTask.can_out - output_port
        Canbus::InterfaceTask.state - output_port
        Canbus::InterfaceTask.can_in - input_port
        Canbus::InterfaceTask - instance_req
        Controldev::Remote.raw_command - output_port
        Controldev::Remote.state - output_port
        Controldev::Remote.canInputDevice - input_port
        Controldev::Remote - instance_req
        Controldev::GenericRawToMotion2D.motion_command - output_port
        Controldev::GenericRawToMotion2D.state - output_port
        Controldev::GenericRawToMotion2D.raw_command - input_port
        Controldev::GenericRawToMotion2D - instance_req
        UwParticleLocalization::FastFusion.pose_samples - output_port
        UwParticleLocalization::FastFusion.state - output_port
        UwParticleLocalization::FastFusion.position_samples - input_port
        UwParticleLocalization::FastFusion.depth_samples - input_port
        UwParticleLocalization::FastFusion.orientation_samples - input_port
        UwParticleLocalization::FastFusion.velocity_samples - input_port
        UwParticleLocalization::FastFusion - instance_req
        Canbus::Task.stats - output_port
        Canbus::Task.can_status - output_port
        Canbus::Task.log_message - output_port
        Canbus::Task.state - output_port
        Canbus::Task.in - input_port
        Canbus::Task - instance_req
        AvalonControl::PositionControlTask.motion_commands - output_port
        AvalonControl::PositionControlTask.state - output_port
        AvalonControl::PositionControlTask.pose_samples - input_port
        AvalonControl::PositionControlTask.position_commands - input_port
        AvalonControl::PositionControlTask - instance_req
        Controldev::RawWheelToMotion2D.motion_command - output_port
        Controldev::RawWheelToMotion2D.state - output_port
        Controldev::RawWheelToMotion2D.raw_command - input_port
        Controldev::RawWheelToMotion2D - instance_req
        UwParticleLocalization::MotionModel.pose_samples - output_port
        UwParticleLocalization::MotionModel.stream_aligner_status - output_port
        UwParticleLocalization::MotionModel.state - output_port
        UwParticleLocalization::MotionModel.thruster_samples - input_port
        UwParticleLocalization::MotionModel.orientation_samples - input_port
        UwParticleLocalization::MotionModel - instance_req
        CameraProsilica::Task.frame - output_port
        CameraProsilica::Task.frame_raw - output_port
        CameraProsilica::Task.state - output_port
        CameraProsilica::Task - instance_req
        Buoy::ServoingOnWall.world_cmd - output_port
        Buoy::ServoingOnWall.aligned_position_cmd - output_port
        Buoy::ServoingOnWall.state - output_port
        Buoy::ServoingOnWall.buoy_samples - input_port
        Buoy::ServoingOnWall.wall_samples - input_port
        Buoy::ServoingOnWall.orientation_samples - input_port
        Buoy::ServoingOnWall - instance_req
        Controldev::RawJoystickToMotion2D.motion_command - output_port
        Controldev::RawJoystickToMotion2D.state - output_port
        Controldev::RawJoystickToMotion2D.raw_command - input_port
        Controldev::RawJoystickToMotion2D - instance_req
        Buoy::Detector2.state - output_port
        Buoy::Detector2.buoy - output_port
        Buoy::Detector2.light - output_port
        Buoy::Detector2.h_image - output_port
        Buoy::Detector2.s_image - output_port
        Buoy::Detector2.v_image - output_port
        Buoy::Detector2.binary_debug_image - output_port
        Buoy::Detector2.gray_debug_image - output_port
        Buoy::Detector2.hough_debug_image - output_port
        Buoy::Detector2.other_buoys - output_port
        Buoy::Detector2.debug_image - output_port
        Buoy::Detector2.frame - input_port
        Buoy::Detector2 - instance_req
        AuvControl::AlignedToBody.state - output_port
        AuvControl::AlignedToBody.cmd_out - output_port
        AuvControl::AlignedToBody.cmd_in - input_port
        AuvControl::AlignedToBody.cmd_cascade - input_port
        AuvControl::AlignedToBody.orientation_samples - input_port
        AuvControl::AlignedToBody - instance_req
        Controldev::SliderboxTask.raw_command - output_port
        Controldev::SliderboxTask.state - output_port
        Controldev::SliderboxTask - instance_req
        Buoy::Detector.state - output_port
        Buoy::Detector.buoy - output_port
        Buoy::Detector.light - output_port
        Buoy::Detector.h_image - output_port
        Buoy::Detector.s_image - output_port
        Buoy::Detector.v_image - output_port
        Buoy::Detector.binary_debug_image - output_port
        Buoy::Detector.gray_debug_image - output_port
        Buoy::Detector.hough_debug_image - output_port
        Buoy::Detector.other_buoys - output_port
        Buoy::Detector.debug_image - output_port
        Buoy::Detector.frame - input_port
        Buoy::Detector - instance_req
        Controldev::SteeringWheelTask.raw_command - output_port
        Controldev::SteeringWheelTask.state - output_port
        Controldev::SteeringWheelTask - instance_req
        GpsHelper::GPSFaker.position_samples - output_port
        GpsHelper::GPSFaker.state - output_port
        GpsHelper::GPSFaker - instance_req
        AuvControl::OptimalHeadingController.state - output_port
        AuvControl::OptimalHeadingController.cmd_out - output_port
        AuvControl::OptimalHeadingController.cmd_in - input_port
        AuvControl::OptimalHeadingController.cmd_cascade - input_port
        AuvControl::OptimalHeadingController.orientation_samples - input_port
        AuvControl::OptimalHeadingController - instance_req
        Controldev::Mouse3DTask.raw_command - output_port
        Controldev::Mouse3DTask.state - output_port
        Controldev::Mouse3DTask - instance_req
        UwParticleLocalization::OrientationCorrection.orientation_output - output_port
        UwParticleLocalization::OrientationCorrection.orientation_offset_corrected - output_port
        UwParticleLocalization::OrientationCorrection.state - output_port
        UwParticleLocalization::OrientationCorrection.orientation_input - input_port
        UwParticleLocalization::OrientationCorrection.orientation_offset - input_port
        UwParticleLocalization::OrientationCorrection - instance_req
        Controldev::JoyPadTask.raw_command - output_port
        Controldev::JoyPadTask.state - output_port
        Controldev::JoyPadTask - instance_req
        AuvControl::MotionCommand2DConverter.cmd_out - output_port
        AuvControl::MotionCommand2DConverter.state - output_port
        AuvControl::MotionCommand2DConverter.cmd_in - input_port
        AuvControl::MotionCommand2DConverter - instance_req
        GpsHelper::WaypointNavigation.target_waypoint - output_port
        GpsHelper::WaypointNavigation.heading_offset - output_port
        GpsHelper::WaypointNavigation.distance_delta - output_port
        GpsHelper::WaypointNavigation.state - output_port
        GpsHelper::WaypointNavigation.gps_position_samples - input_port
        GpsHelper::WaypointNavigation.pose_samples - input_port
        GpsHelper::WaypointNavigation - instance_req
        Controldev::JoystickTask.raw_command - output_port
        Controldev::JoystickTask.state - output_port
        Controldev::JoystickTask - instance_req
        UwParticleLocalization::Task.pose_samples - output_port
        UwParticleLocalization::Task.environment - output_port
        UwParticleLocalization::Task.dead_reckoning_samples - output_port
        UwParticleLocalization::Task.full_dead_reckoning - output_port
        UwParticleLocalization::Task.particles - output_port
        UwParticleLocalization::Task.debug_sonar_beam - output_port
        UwParticleLocalization::Task.stats - output_port
        UwParticleLocalization::Task.depth_grid - output_port
        UwParticleLocalization::Task.grid_map - output_port
        UwParticleLocalization::Task.debug_filtered_obstacles - output_port
        UwParticleLocalization::Task.stream_aligner_status - output_port
        UwParticleLocalization::Task.state - output_port
        UwParticleLocalization::Task.laser_samples - input_port
        UwParticleLocalization::Task.speed_samples - input_port
        UwParticleLocalization::Task.pipeline_samples - input_port
        UwParticleLocalization::Task.pose_update - input_port
        UwParticleLocalization::Task.gps_pose_samples - input_port
        UwParticleLocalization::Task.buoy_samples_orange - input_port
        UwParticleLocalization::Task.buoy_samples_white - input_port
        UwParticleLocalization::Task.thruster_samples - input_port
        UwParticleLocalization::Task.orientation_samples - input_port
        UwParticleLocalization::Task.echosounder_samples - input_port
        UwParticleLocalization::Task.obstacle_samples - input_port
        UwParticleLocalization::Task.structur_samples - input_port
        UwParticleLocalization::Task - instance_req
        Buoy::Survey.state - output_port
        Buoy::Survey.strafed_angle - output_port
        Buoy::Survey.relative_position - output_port
        Buoy::Survey.position - output_port
        Buoy::Survey.orientation_samples - input_port
        Buoy::Survey.force_cutting - input_port
        Buoy::Survey.input_buoy - input_port
        Buoy::Survey.motion_command - input_port
        Buoy::Survey.light - input_port
        Buoy::Survey.target_angle_input - input_port
        Buoy::Survey - instance_req
        Controldev::GenericTask.raw_command - output_port
        Controldev::GenericTask.state - output_port
        Controldev::GenericTask - instance_req
        AuvControl::WaypointNavigator.cmd_out - output_port
        AuvControl::WaypointNavigator.waypoint_info - output_port
        AuvControl::WaypointNavigator.state - output_port
        AuvControl::WaypointNavigator.trajectory - input_port
        AuvControl::WaypointNavigator.pose_sample - input_port
        AuvControl::WaypointNavigator - instance_req
        GpsHelper::MapToGPS.gps_position - output_port
        GpsHelper::MapToGPS.transformer_stream_aligner_status - output_port
        GpsHelper::MapToGPS.transformer_status - output_port
        GpsHelper::MapToGPS.state - output_port
        GpsHelper::MapToGPS.position_samples - input_port
        GpsHelper::MapToGPS.dynamic_transformations - input_port
        GpsHelper::MapToGPS - instance_req
        AuvControl::ConstantCommandGroundAvoidance.floor_position - output_port
        AuvControl::ConstantCommandGroundAvoidance.state - output_port
        AuvControl::ConstantCommandGroundAvoidance.cmd_out - output_port
        AuvControl::ConstantCommandGroundAvoidance.altimeter - input_port
        AuvControl::ConstantCommandGroundAvoidance.depth - input_port
        AuvControl::ConstantCommandGroundAvoidance.cmd_in - input_port
        AuvControl::ConstantCommandGroundAvoidance - instance_req
        OrientationEstimator::IKF.transformer_stream_aligner_status - output_port
        OrientationEstimator::IKF.transformer_status - output_port
        OrientationEstimator::IKF.attitude_b_g - output_port
        OrientationEstimator::IKF.state - output_port
        OrientationEstimator::IKF.imu_samples - input_port
        OrientationEstimator::IKF.fog_samples - input_port
        OrientationEstimator::IKF.initial_orientation - input_port
        OrientationEstimator::IKF.dynamic_transformations - input_port
        OrientationEstimator::IKF - instance_req
        OrientationEstimator::UKFEstimator.stream_aligner_status - output_port
        OrientationEstimator::UKFEstimator.attitude_b_g - output_port
        OrientationEstimator::UKFEstimator.state - output_port
        OrientationEstimator::UKFEstimator.imu_orientation - input_port
        OrientationEstimator::UKFEstimator.fog_samples - input_port
        OrientationEstimator::UKFEstimator.imu_samples - input_port
        OrientationEstimator::UKFEstimator - instance_req
        Lights::Lights.light_value - output_port
        Lights::Lights.state - output_port
        Lights::Lights.int_in - input_port
        Lights::Lights.can_in - input_port
        Lights::Lights - instance_req
        OffshorePipelineDetector::Task.state - output_port
        OffshorePipelineDetector::Task.pipeline - output_port
        OffshorePipelineDetector::Task.world_command - output_port
        OffshorePipelineDetector::Task.aligned_position_command - output_port
        OffshorePipelineDetector::Task.position_command - output_port
        OffshorePipelineDetector::Task.debug - output_port
        OffshorePipelineDetector::Task.debug_frame - output_port
        OffshorePipelineDetector::Task.frame - input_port
        OffshorePipelineDetector::Task.orientation_sample - input_port
        OffshorePipelineDetector::Task.altitude_samples - input_port
        OffshorePipelineDetector::Task - instance_req
        OrientationEstimator::IKFEstimator.stream_aligner_status - output_port
        OrientationEstimator::IKFEstimator.attitude_b_g - output_port
        OrientationEstimator::IKFEstimator.inputs_backup - output_port
        OrientationEstimator::IKFEstimator.state - output_port
        OrientationEstimator::IKFEstimator.imu_orientation - input_port
        OrientationEstimator::IKFEstimator.fog_samples - input_port
        OrientationEstimator::IKFEstimator.imu_samples - input_port
        OrientationEstimator::IKFEstimator - instance_req
        AuvControl::ConstantCommandGroundFollower.floor_position - output_port
        AuvControl::ConstantCommandGroundFollower.state - output_port
        AuvControl::ConstantCommandGroundFollower.cmd_out - output_port
        AuvControl::ConstantCommandGroundFollower.altimeter - input_port
        AuvControl::ConstantCommandGroundFollower.depth - input_port
        AuvControl::ConstantCommandGroundFollower.cmd_in - input_port
        AuvControl::ConstantCommandGroundFollower - instance_req
        OrientationEstimator::BaseEstimator.stream_aligner_status - output_port
        OrientationEstimator::BaseEstimator.attitude_b_g - output_port
        OrientationEstimator::BaseEstimator.state - output_port
        OrientationEstimator::BaseEstimator.imu_orientation - input_port
        OrientationEstimator::BaseEstimator.fog_samples - input_port
        OrientationEstimator::BaseEstimator.heading_correction - input_port
        OrientationEstimator::BaseEstimator - instance_req
        XsensImu::Task.orientation_samples - output_port
        XsensImu::Task.calibrated_sensors - output_port
        XsensImu::Task.timestamp_estimator_status - output_port
        XsensImu::Task.state - output_port
        XsensImu::Task.hard_timestamps - input_port
        XsensImu::Task - instance_req
        AuvControl::ConstantCommand.cmd_out - output_port
        AuvControl::ConstantCommand.state - output_port
        AuvControl::ConstantCommand - instance_req
        OffshorePipelineDetector::SonarDetector.frame - output_port
        OffshorePipelineDetector::SonarDetector.state - output_port
        OffshorePipelineDetector::SonarDetector.sonar_beam - input_port
        OffshorePipelineDetector::SonarDetector - instance_req
        LowLevelDriver::LowLevelTask.state - output_port
        LowLevelDriver::LowLevelTask.depth_samples - input_port
        LowLevelDriver::LowLevelTask.ShortExposure - input_port
        LowLevelDriver::LowLevelTask.LongExposure - input_port
        LowLevelDriver::LowLevelTask.LightValue - input_port
        LowLevelDriver::LowLevelTask.DebugLED - input_port
        LowLevelDriver::LowLevelTask.LaserRate - input_port
        LowLevelDriver::LowLevelTask - instance_req
        AuvHelper::DepthAndOrientationFusion.pose_samples - output_port
        AuvHelper::DepthAndOrientationFusion.stream_aligner_status - output_port
        AuvHelper::DepthAndOrientationFusion.state - output_port
        AuvHelper::DepthAndOrientationFusion.orientation_samples - input_port
        AuvHelper::DepthAndOrientationFusion.depth_samples - input_port
        AuvHelper::DepthAndOrientationFusion.ground_distance - input_port
        AuvHelper::DepthAndOrientationFusion - instance_req
        AuvRelPosController::Task.motion_command - output_port
        AuvRelPosController::Task.state - output_port
        AuvRelPosController::Task.position_command - input_port
        AuvRelPosController::Task.position_sample - input_port
        AuvRelPosController::Task - instance_req
        AuvControl::AccelerationController.state - output_port
        AuvControl::AccelerationController.cmd_out - output_port
        AuvControl::AccelerationController.cmd_in - input_port
        AuvControl::AccelerationController.cmd_cascade - input_port
        AuvControl::AccelerationController - instance_req
        SonarFeatureEstimator::Task.new_feature - output_port
        SonarFeatureEstimator::Task.features_out - output_port
        SonarFeatureEstimator::Task.debug_output - output_port
        SonarFeatureEstimator::Task.2d_debug_output - output_port
        SonarFeatureEstimator::Task.state - output_port
        SonarFeatureEstimator::Task.sonar_input - input_port
        SonarFeatureEstimator::Task.orientation_sample - input_port
        SonarFeatureEstimator::Task - instance_req
        PoseEstimation::VehiclePoseEstimator.pose_samples - output_port
        PoseEstimation::VehiclePoseEstimator.state - output_port
        PoseEstimation::VehiclePoseEstimator.transformer_stream_aligner_status - output_port
        PoseEstimation::VehiclePoseEstimator.transformer_status - output_port
        PoseEstimation::VehiclePoseEstimator.orientation_samples - input_port
        PoseEstimation::VehiclePoseEstimator.velocity_samples - input_port
        PoseEstimation::VehiclePoseEstimator.position_samples - input_port
        PoseEstimation::VehiclePoseEstimator.dynamic_transformations - input_port
        PoseEstimation::VehiclePoseEstimator - instance_req
        CameraUnicap::CameraTask.frame - output_port
        CameraUnicap::CameraTask.state - output_port
        CameraUnicap::CameraTask - instance_req
        SonarBlueview::Task.frame - output_port
        SonarBlueview::Task.state - output_port
        SonarBlueview::Task - instance_req
        WallServoing::DualSonarServoing.state - output_port
        WallServoing::DualSonarServoing.position_command - output_port
        WallServoing::DualSonarServoing.aligned_command - output_port
        WallServoing::DualSonarServoing.wall_servoing_debug - output_port
        WallServoing::DualSonarServoing.sonarbeam_feature_front - input_port
        WallServoing::DualSonarServoing.sonarbeam_feature_rear - input_port
        WallServoing::DualSonarServoing.orientation_sample - input_port
        WallServoing::DualSonarServoing - instance_req
        LineScanner::Task.state - output_port
        LineScanner::Task.pointcloud - output_port
        LineScanner::Task.debug - output_port
        LineScanner::Task.frame - input_port
        LineScanner::Task - instance_req
        VideoStreamerVlc::Streamer{1}.state - output_port
        VideoStreamerVlc::Streamer{1}.frame_bottom_camera - input_port
        VideoStreamerVlc::Streamer{1}.frame_front_camera - input_port
        VideoStreamerVlc::Streamer{1}.frame_blueview - input_port
        VideoStreamerVlc::Streamer{1} - instance_req
        VideoStreamerVlc::Streamer.state - output_port
        VideoStreamerVlc::Streamer.frame_bottom_camera - input_port
        VideoStreamerVlc::Streamer.frame_front_camera - input_port
        VideoStreamerVlc::Streamer.frame_blueview - input_port
        VideoStreamerVlc::Streamer - instance_req
        RawControlCommandConverter::Movement.motion_command - output_port
        RawControlCommandConverter::Movement.world_command - output_port
        RawControlCommandConverter::Movement.world_command_depth - output_port
        RawControlCommandConverter::Movement.aligned_velocity_command - output_port
        RawControlCommandConverter::Movement.state - output_port
        RawControlCommandConverter::Movement.raw_command - input_port
        RawControlCommandConverter::Movement.orientation_readings - input_port
        RawControlCommandConverter::Movement.ground_distance - input_port
        RawControlCommandConverter::Movement - instance_req
        Taskmon::Task.stats - output_port
        Taskmon::Task.state - output_port
        Taskmon::Task - instance_req
        RawControlCommandConverter::Position.position_command - output_port
        RawControlCommandConverter::Position.world_command - output_port
        RawControlCommandConverter::Position.state - output_port
        RawControlCommandConverter::Position.raw_command - input_port
        RawControlCommandConverter::Position.pose_samples - input_port
        RawControlCommandConverter::Position - instance_req
        CameraBase::Task.frame - output_port
        CameraBase::Task.frame_raw - output_port
        CameraBase::Task.state - output_port
        CameraBase::Task - instance_req
        Gps::GPSDTask.solution - output_port
        Gps::GPSDTask.position_samples - output_port
        Gps::GPSDTask.state - output_port
        Gps::GPSDTask - instance_req
        Modemdriver::ModemCanbus.data_out - output_port
        Modemdriver::ModemCanbus.distance - output_port
        Modemdriver::ModemCanbus.out_modem_substates - output_port
        Modemdriver::ModemCanbus.state - output_port
        Modemdriver::ModemCanbus.can_out - output_port
        Modemdriver::ModemCanbus.stats - output_port
        Modemdriver::ModemCanbus.data_in - input_port
        Modemdriver::ModemCanbus.can_in - input_port
        Modemdriver::ModemCanbus - instance_req
        Modemdriver::ModemSerial.data_out - output_port
        Modemdriver::ModemSerial.distance - output_port
        Modemdriver::ModemSerial.out_modem_substates - output_port
        Modemdriver::ModemSerial.state - output_port
        Modemdriver::ModemSerial.data_in - input_port
        Modemdriver::ModemSerial - instance_req
        Modemdriver::Modem.data_out - output_port
        Modemdriver::Modem.distance - output_port
        Modemdriver::Modem.out_modem_substates - output_port
        Modemdriver::Modem.state - output_port
        Modemdriver::Modem.data_in - input_port
        Modemdriver::Modem - instance_req
        Dynamixel::Task.upper2lower - output_port
        Dynamixel::Task.angle - output_port
        Dynamixel::Task.state - output_port
        Dynamixel::Task.lowerDynamixel2UpperDynamixel - output_port
        Dynamixel::Task.cmd_angle - input_port
        Dynamixel::Task - instance_req
        HsvMosaicing::Task.result - output_port
        HsvMosaicing::Task.state - output_port
        HsvMosaicing::Task.frame - input_port
        HsvMosaicing::Task - instance_req
        Sysmon::Task.ocu_markers - output_port
        Sysmon::Task.annotations - output_port
        Sysmon::Task.system_status - output_port
        Sysmon::Task.state - output_port
        Sysmon::Task.can_in_system_status - input_port
        Sysmon::Task.can_in_experiment_markers - input_port
        Sysmon::Task.in_experiment_markers - input_port
        Sysmon::Task.in_modem_substates - input_port
        Sysmon::Task - instance_req
        StructureServoing::Alignment.world_command - output_port
        StructureServoing::Alignment.aligned_speed_command - output_port
        StructureServoing::Alignment.left - output_port
        StructureServoing::Alignment.right - output_port
        StructureServoing::Alignment.top - output_port
        StructureServoing::Alignment.bottom - output_port
        StructureServoing::Alignment.size - output_port
        StructureServoing::Alignment.x - output_port
        StructureServoing::Alignment.y - output_port
        StructureServoing::Alignment.state - output_port
        StructureServoing::Alignment.input - input_port
        StructureServoing::Alignment - instance_req
        StructureServoing::Task.servoed_angle - output_port
        StructureServoing::Task.angle_speed - output_port
        StructureServoing::Task.world_command - output_port
        StructureServoing::Task.aligned_speed_command - output_port
        StructureServoing::Task.found_structure - output_port
        StructureServoing::Task.state - output_port
        StructureServoing::Task.left - output_port
        StructureServoing::Task.right - output_port
        StructureServoing::Task.top - output_port
        StructureServoing::Task.bottom - output_port
        StructureServoing::Task.size - output_port
        StructureServoing::Task.heading - output_port
        StructureServoing::Task.cnt_left - output_port
        StructureServoing::Task.cnt_right - output_port
        StructureServoing::Task.cnt_top - output_port
        StructureServoing::Task.cnt_bottom - output_port
        StructureServoing::Task.rbs - input_port
        StructureServoing::Task.input - input_port
        StructureServoing::Task - instance_req
        PipelineInspection::ColorFilter.frame_out - output_port
        PipelineInspection::ColorFilter.green_frame - output_port
        PipelineInspection::ColorFilter.diff_frame - output_port
        PipelineInspection::ColorFilter.state - output_port
        PipelineInspection::ColorFilter.frame_in - input_port
        PipelineInspection::ColorFilter - instance_req
        Gps::BaseTask.solution - output_port
        Gps::BaseTask.position_samples - output_port
        Gps::BaseTask.state - output_port
        Gps::BaseTask - instance_req
        Hbridge::CommandWriter.state - output_port
        Hbridge::CommandWriter.can_out - output_port
        Hbridge::CommandWriter.speedCtrlDebug - output_port
        Hbridge::CommandWriter.fakeReader - output_port
        Hbridge::CommandWriter.can_in - input_port
        Hbridge::CommandWriter.command - input_port
        Hbridge::CommandWriter - instance_req
        StructuredLight::Calibration.stream_aligner_status - output_port
        StructuredLight::Calibration.state - output_port
        StructuredLight::Calibration.laser_scan - input_port
        StructuredLight::Calibration.calibration - input_port
        StructuredLight::Calibration - instance_req
        AuvWaypointNavigator::Task.relative_position_command - output_port
        AuvWaypointNavigator::Task.current_delta - output_port
        AuvWaypointNavigator::Task.current_waypoint - output_port
        AuvWaypointNavigator::Task.queue_size - output_port
        AuvWaypointNavigator::Task.state - output_port
        AuvWaypointNavigator::Task.trajectory - input_port
        AuvWaypointNavigator::Task.pose_samples - input_port
        AuvWaypointNavigator::Task - instance_req
        StructuredLight::Task.laser_scan - output_port
        StructuredLight::Task.candidates - output_port
        StructuredLight::Task.debug_frame - output_port
        StructuredLight::Task.state - output_port
        StructuredLight::Task.frame_pair - input_port
        StructuredLight::Task.frame - input_port
        StructuredLight::Task - instance_req
        PipelineInspection::Inspection.inspectionStatus - output_port
        PipelineInspection::Inspection.pipePoints - output_port
        PipelineInspection::Inspection.debugFrame - output_port
        PipelineInspection::Inspection.pipeMap - output_port
        PipelineInspection::Inspection.state - output_port
        PipelineInspection::Inspection.laserSamples - input_port
        PipelineInspection::Inspection.laserPoints - input_port
        PipelineInspection::Inspection.laserPointCloud - input_port
        PipelineInspection::Inspection.pipeline - input_port
        PipelineInspection::Inspection.dead_reckoning - input_port
        PipelineInspection::Inspection - instance_req
        Hbridge::SensorReader.state - output_port
        Hbridge::SensorReader.can_out - output_port
        Hbridge::SensorReader.status_samples - output_port
        Hbridge::SensorReader.can_in - input_port
        Hbridge::SensorReader - instance_req
        PipelineInspection::LaserSimulation.laserPoints - output_port
        PipelineInspection::LaserSimulation.laserPointCloud - output_port
        PipelineInspection::LaserSimulation.vehiclePos - output_port
        PipelineInspection::LaserSimulation.state - output_port
        PipelineInspection::LaserSimulation - instance_req
        SonarFeatureDetector::Task.features - output_port
        SonarFeatureDetector::Task.next_target - output_port
        SonarFeatureDetector::Task.next_target_feature - output_port
        SonarFeatureDetector::Task.next_target_command - output_port
        SonarFeatureDetector::Task.state - output_port
        SonarFeatureDetector::Task.grid_maps - input_port
        SonarFeatureDetector::Task.pose_samples - input_port
        SonarFeatureDetector::Task - instance_req
        ImagePreprocessing::StereoTask.state - output_port
        ImagePreprocessing::StereoTask.oframe_pair - output_port
        ImagePreprocessing::StereoTask.frame_left - input_port
        ImagePreprocessing::StereoTask.frame_right - input_port
        ImagePreprocessing::StereoTask - instance_req
        Interfaces::IMU.orientation_samples - output_port
        Interfaces::IMU.calibrated_sensors - output_port
        Interfaces::IMU.state - output_port
        Interfaces::IMU - instance_req
        ImagePreprocessing::HSVSegmentationAndBlur.oframe - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.binary_result - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.hDebug - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.hDebugGray - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.vDebug - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.vDebugGray - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.sDebug - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.sDebugGray - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.hsv_v_frame - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.state - output_port
        ImagePreprocessing::HSVSegmentationAndBlur.frame - input_port
        ImagePreprocessing::HSVSegmentationAndBlur - instance_req
        BatteryWatcher::Task.can_out - output_port
        BatteryWatcher::Task.state - output_port
        BatteryWatcher::Task.battery_info - output_port
        BatteryWatcher::Task.can_in - input_port
        BatteryWatcher::Task - instance_req
        Gps::MB500Task.solution - output_port
        Gps::MB500Task.position_samples - output_port
        Gps::MB500Task.state - output_port
        Gps::MB500Task.constellation - output_port
        Gps::MB500Task.time - output_port
        Gps::MB500Task - instance_req
        StructureReconstruction::Task.transformer_stream_aligner_status - output_port
        StructureReconstruction::Task.transformer_status - output_port
        StructureReconstruction::Task.state - output_port
        StructureReconstruction::Task.front_camera - input_port
        StructureReconstruction::Task.bottom_camera - input_port
        StructureReconstruction::Task.dynamic_transformations - input_port
        StructureReconstruction::Task - instance_req
        Interfaces::LaserRangeFinder.scans - output_port
        Interfaces::LaserRangeFinder.state - output_port
        Interfaces::LaserRangeFinder - instance_req
        SonarTritech::Micron.ground_distance - output_port
        SonarTritech::Micron.sonar_beam - output_port
        SonarTritech::Micron.state - output_port
        SonarTritech::Micron - instance_req
        VideoStreamerVlc::Capturer.state - output_port
        VideoStreamerVlc::Capturer - instance_req
        SonarTritech::Echosounder.ground_distance - output_port
        SonarTritech::Echosounder.state - output_port
        SonarTritech::Echosounder - instance_req
        SonarTritech::Profiling.profiling_scan - output_port
        SonarTritech::Profiling.state - output_port
        SonarTritech::Profiling - instance_req
        FrameDemultiplexer::Task.oframe_pair - output_port
        FrameDemultiplexer::Task.oframe - output_port
        FrameDemultiplexer::Task.state - output_port
        FrameDemultiplexer::Task.iframe - input_port
        FrameDemultiplexer::Task - instance_req
        ImagePreprocessing::MonoTask.state - output_port
        ImagePreprocessing::MonoTask.oframe - output_port
        ImagePreprocessing::MonoTask.frame - input_port
        ImagePreprocessing::MonoTask - instance_req
        Interfaces::ActuatorSensorReader.state - output_port
        Interfaces::ActuatorSensorReader - instance_req
        ImagePreprocessing::DepthImage2Pointcloud.pointcloud - output_port
        ImagePreprocessing::DepthImage2Pointcloud.stream_aligner_status - output_port
        ImagePreprocessing::DepthImage2Pointcloud.state - output_port
        ImagePreprocessing::DepthImage2Pointcloud.color_frame - input_port
        ImagePreprocessing::DepthImage2Pointcloud.frame - input_port
        ImagePreprocessing::DepthImage2Pointcloud - instance_req
        Interfaces::ActuatorCommandWriter.state - output_port
        Interfaces::ActuatorCommandWriter - instance_req
        ImagePreprocessing::BaseTask.state - output_port
        ImagePreprocessing::BaseTask - instance_req
        Interfaces::Servo.upper2lower - output_port
        Interfaces::Servo.angle - output_port
        Interfaces::Servo.state - output_port
        Interfaces::Servo.cmd_angle - input_port
        Interfaces::Servo - instance_req
        PoseAuv::PoseEstimatorCmp - instance_req
        Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        PoseAuv::PoseEstimatorBlindCmp - instance_req
        PoseAuv::InitialOrientationEstimatorCmp - instance_req
        Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        PoseAuv::IKFOrientationEstimatorCmp - instance_req
        Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        ConstantWorldXYVelocityCommand - instance_req
        ConsWA - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        GPSHelper::GPSWaypointsCmp - instance_req
        Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Base::ControlLoop - instance_req
        LowLevel::Cmp - instance_req
        Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        Wall::Follower__controller.is_a__AuvRelPosController::Task__ - instance_req
        Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_ - instance_req
        Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__AuvRelPosController::Task__ - instance_req
        Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        AuvControl::JoystickCommandCmp - instance_req
        Modem::ModemCmp - instance_req
        AuvControl::DepthFusionCmp - instance_req
        AuvControl::SimpleMove - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Structure::Detector - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Hbridge::ControlSystem - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__ - instance_req
        AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        AuvControl::SimplePosMove - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Buoy::DetectorCmp_Base::ControllerSrv_ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        AuvCont::WorldPositionCmp - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__ - instance_req
        blueview_cmp - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        front_camera_cmp - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        bottom_camera_cmp - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Structure::Alignment - instance_req
        AuvControl::TrajectoryMove - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Localization::SonarFeatureDetectorCmp - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Localization::FixMapHack - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        AuvCont::ConstantCommandGroundAvoidanceCmp - instance_req
        Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Localization::HoughDetector - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Localization::DeadReckoning - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Pipeline::Detector_new - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Localization::ParticleDetector - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Buoy::DoubleBuoyCmp - instance_req
        AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__ - instance_req
        Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        Buoy::DetectorCmp2 - instance_req
        Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvCont::BuoyWallCmp - instance_req
        Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Detector - instance_req
        Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        AuvCont::StructureCmp - instance_req
        Structure::StructureReconstructionComp - instance_req
        Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        AuvCont::MoveCmp - instance_req
        Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__ - instance_req
        Pipeline::Follower - instance_req
        AuvCont::PositionMoveCmp - instance_req
        Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Structure::SonarStructureServoingComp - instance_req
        AuvCont::WorldXYPositionCmp - instance_req
        AuvCont::Trajectory - instance_req
        Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        AuvCont::WorldYPositionXVelocityCmp - instance_req
        Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__ - instance_req
        Wall::Follower - instance_req
        Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__ - instance_req
        Pipeline::Follower__controller.is_a__AuvRelPosController::Task__ - instance_req
        AuvCont::WorldXYVelocityCmp - instance_req
        Wall::DetectorNew - instance_req
        AuvCont::WorldXYZPositionCmp - instance_req
        Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__ - instance_req
        Wall::Detector - instance_req
        Buoy::DetectorCmp - instance_req
        Buoy::ControllerNewCmp - instance_req
        Buoy::DetectorNewCmp - instance_req
        Dev::Sensors::DepthReaderAvalon - instance_req
        Syskit::ComBus - instance_req
        Syskit::Device - instance_req
        Auv::ModemConnectionSrv - instance_req
        Dev::Bus::CAN::BusSrv - instance_req
        Base::GroundDistanceSrv - instance_req
        Base::DVLSrv - instance_req
        Base::VelocitySrv - instance_req
        Base::PoseDeltaSrv - instance_req
        Dev::Sensors::Modem - instance_req
        Base::RelativePoseSrv - instance_req
        Base::GlobalPoseSrv - instance_req
        Base::TransformationSrv - instance_req
        Base::PoseSrv - instance_req
        Base::OrientationWithZSrv - instance_req
        Base::ZProviderSrv - instance_req
        Dev::Sensors::KVH::DSP3000 - instance_req
        Base::OrientationSrv - instance_req
        Base::PositionSrv - instance_req
        Dev::Controldev::Mouse3D - instance_req
        Dev::Bus::CAN::ClientSrv - instance_req
        Base::RotationSrv - instance_req
        Dev::Controldev::Joystick - instance_req
        Dev::Bus::CAN::ClientOutSrv - instance_req
        Base::MapSrv - instance_req
        Base::OrientationToCorrectSrv - instance_req
        Auv::StructuredLightPairSrv - instance_req
        Dev::Bus::CAN::ClientInSrv - instance_req
        Auv::SoundSourceDirectionSrv - instance_req
        Dev::Bus::CAN::BusOutSrv - instance_req
        Dev::Sensors::BlueView - instance_req
        Base::RawCommandControlledSystemSrv - instance_req
        Dev::Bus::CAN::BusInSrv - instance_req
        Base::AUVRelativeMotionControlledSystemSrv - instance_req
        Base::RawCommandControllerSrv - instance_req
        Base::AUVRelativeMotionControllerSrv - instance_req
        Base::AUVRelativeMotionCommandConsumerSrv - instance_req
        Dev::Bus::CAN::BusBaseSrv - instance_req
        Dev::Bus::CAN - instance_req
        Base::RawCommandCommandConsumerSrv - instance_req
        Base::WorldZRollPitchYawSrv - instance_req
        Base::AUVMotionControlledSystemSrv - instance_req
        Base::WorldXYVelocityControllerSrv - instance_req
        Base::AUVMotionControllerSrv - instance_req
        Base::AUVMotionCommandConsumerSrv - instance_req
        Base::WorldYPositionXVelocityControllerSrv - instance_req
        Base::WorldXYZPositionControllerSrv - instance_req
        Base::Motion2DControlledSystemSrv - instance_req
        Base::Motion2DControllerSrv - instance_req
        Base::Motion2DCommandConsumerSrv - instance_req
        Base::WorldXYPositionControllerSrv - instance_req
        Base::WorldXYZRollPitchYawControllerSrv - instance_req
        Base::JointsControlledSystemSrv - instance_req
        Dev::Actuators::Dynamixel - instance_req
        Base::WorldXYZRollPitchYawControlledSystemSrv - instance_req
        Base::JointsControllerSrv - instance_req
        Dev::Actuators::PTU - instance_req
        Base::JointsCommandSrv - instance_req
        Dev::Sensors::TimestamperDev - instance_req
        Base::JointsStatusSrv - instance_req
        Dev::Sensors::Cameras::USB - instance_req
        Base::JointsCommandConsumerSrv - instance_req
        Dev::Sensors::Cameras::Prosilica - instance_req
        Dev::Sensors::Cameras::Firewire - instance_req
        Dev::Sensors::XsensAHRS - instance_req
        Dev::Sensors::Hokuyo - instance_req
        Dev::Sensors::GPS - instance_req
        Base::PointcloudProviderSrv - instance_req
        Base::SonarScanProviderSrv - instance_req
        Base::ActuatorControlledSystemSrv - instance_req
        Base::LaserRangeFinderSrv - instance_req
        Base::ActuatorControllerSrv - instance_req
        Base::DistanceImageProviderSrv - instance_req
        Base::ActuatorStatusSrv - instance_req
        Base::StereoPairProviderSrv - instance_req
        Base::ActuatorCommandConsumerSrv - instance_req
        Base::ImageProviderSrv - instance_req
        Base::CalibratedIMUSensorsSrv - instance_req
        Base::CompensatedIMUSensorsSrv - instance_req
        Base::ControlledSystemSrv - instance_req
        Base::ControllerSrv - instance_req
        Base::IMUSensorsSrv - instance_req
        Dev::Actuators::Lights - instance_req
        Base::ImageConsumerSrv - instance_req
        Base::TimestampInputSrv - instance_req
        Base::TimestamperSrv - instance_req
        WallServoing::WallOrientationSrv - instance_req
        Dev::ASVModem - instance_req
        Dev::Sensors::Hbridge - instance_req
        Dev::Actuators::Hbridge - instance_req
        Dev::SystemStatus - instance_req
        Localization::HoughSrv - instance_req
        Dev::Sensors::Battery - instance_req
        Dev::Micron - instance_req
        Dev::Echosounder - instance_req
        Dev::Profiling - instance_req
  )
  (:init

        (composition  root)
        (task  AuvControl::WorldToAligned)
        (task  WallServoing::SingleSonarServoing)
        (task  Syskit::ROS::Node)
        (task  AuvControl::PIDController)
        (task  WallServoing::SonarServoing)
        (task  RTT::TaskContext)
        (task  Syskit::RubyTaskContext)
        (task  AvalonControl::MotionFeedbackTask)
        (task  WallOrientationCorrection::OrientationInMap)
        (task  WallServoing::WallServoing)
        (task  AuvControl::BasePIDController)
        (task  WallOrientationCorrection::Task)
        (task  WallServoing::WallDetector)
        (task  AvalonControl::TrajectoryFollower)
        (task  FogKvh::Dsp3000Task)
        (task  ModemCan::Task)
        (task  Transformer::Task)
        (task  SonarStructureServoing::Task)
        (task  AuvControl::Base)
        (task  AvalonControl::RelFakeWriter)
        (task  SonarWallHough::Task)
        (task  PoseEstimation::UWPoseEstimator)
        (task  RearSonarDistanceEstimator::Task)
        (task  Logger::Logger)
        (task  DepthReader::Task)
        (task  AvalonControl::FakeWriter)
        (task  PoseEstimation::BaseTask)
        (task  AvalonControl::MotionControlTask)
        (task  Canbus::InterfaceTask)
        (task  Controldev::Remote)
        (task  Controldev::GenericRawToMotion2D)
        (task  UwParticleLocalization::FastFusion)
        (task  Canbus::Task)
        (task  AvalonControl::PositionControlTask)
        (task  Controldev::RawWheelToMotion2D)
        (task  UwParticleLocalization::MotionModel)
        (task  CameraProsilica::Task)
        (task  Buoy::ServoingOnWall)
        (task  Controldev::RawJoystickToMotion2D)
        (task  Buoy::Detector2)
        (task  AuvControl::AlignedToBody)
        (task  Controldev::SliderboxTask)
        (task  Buoy::Detector)
        (task  Controldev::SteeringWheelTask)
        (task  GpsHelper::GPSFaker)
        (task  AuvControl::OptimalHeadingController)
        (task  Controldev::Mouse3DTask)
        (task  UwParticleLocalization::OrientationCorrection)
        (task  Controldev::JoyPadTask)
        (task  AuvControl::MotionCommand2DConverter)
        (task  GpsHelper::WaypointNavigation)
        (task  Controldev::JoystickTask)
        (task  UwParticleLocalization::Task)
        (task  Buoy::Survey)
        (task  Controldev::GenericTask)
        (task  AuvControl::WaypointNavigator)
        (task  GpsHelper::MapToGPS)
        (task  AuvControl::ConstantCommandGroundAvoidance)
        (task  OrientationEstimator::IKF)
        (task  OrientationEstimator::UKFEstimator)
        (task  Lights::Lights)
        (task  OffshorePipelineDetector::Task)
        (task  OrientationEstimator::IKFEstimator)
        (task  AuvControl::ConstantCommandGroundFollower)
        (task  OrientationEstimator::BaseEstimator)
        (task  XsensImu::Task)
        (task  AuvControl::ConstantCommand)
        (task  OffshorePipelineDetector::SonarDetector)
        (task  LowLevelDriver::LowLevelTask)
        (task  AuvHelper::DepthAndOrientationFusion)
        (task  AuvRelPosController::Task)
        (task  AuvControl::AccelerationController)
        (task  SonarFeatureEstimator::Task)
        (task  PoseEstimation::VehiclePoseEstimator)
        (task  CameraUnicap::CameraTask)
        (task  SonarBlueview::Task)
        (task  WallServoing::DualSonarServoing)
        (task  LineScanner::Task)
        (task  VideoStreamerVlc::Streamer{1})
        (task  VideoStreamerVlc::Streamer)
        (task  RawControlCommandConverter::Movement)
        (task  Taskmon::Task)
        (task  RawControlCommandConverter::Position)
        (task  CameraBase::Task)
        (task  Gps::GPSDTask)
        (task  Modemdriver::ModemCanbus)
        (task  Modemdriver::ModemSerial)
        (task  Modemdriver::Modem)
        (task  Dynamixel::Task)
        (task  HsvMosaicing::Task)
        (task  Sysmon::Task)
        (task  StructureServoing::Alignment)
        (task  StructureServoing::Task)
        (task  PipelineInspection::ColorFilter)
        (task  Gps::BaseTask)
        (task  Hbridge::CommandWriter)
        (task  StructuredLight::Calibration)
        (task  AuvWaypointNavigator::Task)
        (task  StructuredLight::Task)
        (task  PipelineInspection::Inspection)
        (task  Hbridge::SensorReader)
        (task  PipelineInspection::LaserSimulation)
        (task  SonarFeatureDetector::Task)
        (task  ImagePreprocessing::StereoTask)
        (task  Interfaces::IMU)
        (task  ImagePreprocessing::HSVSegmentationAndBlur)
        (task  BatteryWatcher::Task)
        (task  Gps::MB500Task)
        (task  StructureReconstruction::Task)
        (task  Interfaces::LaserRangeFinder)
        (task  SonarTritech::Micron)
        (task  VideoStreamerVlc::Capturer)
        (task  SonarTritech::Echosounder)
        (task  SonarTritech::Profiling)
        (task  FrameDemultiplexer::Task)
        (task  ImagePreprocessing::MonoTask)
        (task  Interfaces::ActuatorSensorReader)
        (task  ImagePreprocessing::DepthImage2Pointcloud)
        (task  Interfaces::ActuatorCommandWriter)
        (task  ImagePreprocessing::BaseTask)
        (task  Interfaces::Servo)
        (composition  PoseAuv::PoseEstimatorCmp)
        (composition  Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  PoseAuv::PoseEstimatorBlindCmp)
        (composition  PoseAuv::InitialOrientationEstimatorCmp)
        (composition  Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  PoseAuv::IKFOrientationEstimatorCmp)
        (composition  Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  ConstantWorldXYVelocityCommand)
        (composition  ConsWA)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  GPSHelper::GPSWaypointsCmp)
        (composition  Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Base::ControlLoop)
        (composition  LowLevel::Cmp)
        (composition  Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  Wall::Follower__controller.is_a__AuvRelPosController::Task__)
        (composition  Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_)
        (composition  Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__AuvRelPosController::Task__)
        (composition  Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  AuvControl::JoystickCommandCmp)
        (composition  Modem::ModemCmp)
        (composition  AuvControl::DepthFusionCmp)
        (composition  AuvControl::SimpleMove)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Structure::Detector)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Hbridge::ControlSystem)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__)
        (composition  AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  AuvControl::SimplePosMove)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Buoy::DetectorCmp_Base::ControllerSrv_)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  AuvCont::WorldPositionCmp)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__)
        (composition  blueview_cmp)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  front_camera_cmp)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  bottom_camera_cmp)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__)
        (composition  AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Structure::Alignment)
        (composition  AuvControl::TrajectoryMove)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Localization::SonarFeatureDetectorCmp)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Localization::FixMapHack)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  AuvCont::ConstantCommandGroundAvoidanceCmp)
        (composition  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Localization::HoughDetector)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Localization::DeadReckoning)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Pipeline::Detector_new)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Localization::ParticleDetector)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Buoy::FollowerCmp)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Buoy::DoubleBuoyCmp)
        (composition  AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__)
        (composition  Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  Buoy::DetectorCmp2)
        (composition  Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvCont::BuoyWallCmp)
        (composition  Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Detector)
        (composition  Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  AuvCont::StructureCmp)
        (composition  Structure::StructureReconstructionComp)
        (composition  Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  AuvCont::MoveCmp)
        (composition  Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__)
        (composition  Pipeline::Follower)
        (composition  AuvCont::PositionMoveCmp)
        (composition  Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Structure::SonarStructureServoingComp)
        (composition  AuvCont::WorldXYPositionCmp)
        (composition  AuvCont::Trajectory)
        (composition  Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  AuvCont::WorldYPositionXVelocityCmp)
        (composition  Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__)
        (composition  Wall::Follower)
        (composition  Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__)
        (composition  Pipeline::Follower__controller.is_a__AuvRelPosController::Task__)
        (composition  AuvCont::WorldXYVelocityCmp)
        (composition  Wall::DetectorNew)
        (composition  AuvCont::WorldXYZPositionCmp)
        (composition  Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__)
        (composition  Wall::Detector)
        (composition  Buoy::DetectorCmp)
        (composition  Buoy::ControllerNewCmp)
        (composition  Buoy::DetectorNewCmp)
        (data-service  Dev::Sensors::DepthReaderAvalon)
        (data-service  Syskit::ComBus)
        (data-service  Syskit::Device)
        (data-service  Auv::ModemConnectionSrv)
        (data-service  Dev::Bus::CAN::BusSrv)
        (data-service  Base::GroundDistanceSrv)
        (data-service  Base::DVLSrv)
        (data-service  Base::VelocitySrv)
        (data-service  Base::PoseDeltaSrv)
        (data-service  Dev::Sensors::Modem)
        (data-service  Base::RelativePoseSrv)
        (data-service  Base::GlobalPoseSrv)
        (data-service  Base::TransformationSrv)
        (data-service  Base::PoseSrv)
        (data-service  Base::OrientationWithZSrv)
        (data-service  Base::ZProviderSrv)
        (data-service  Dev::Sensors::KVH::DSP3000)
        (data-service  Base::OrientationSrv)
        (data-service  Base::PositionSrv)
        (data-service  Dev::Controldev::Mouse3D)
        (data-service  Dev::Bus::CAN::ClientSrv)
        (data-service  Base::RotationSrv)
        (data-service  Dev::Controldev::Joystick)
        (data-service  Dev::Bus::CAN::ClientOutSrv)
        (data-service  Base::MapSrv)
        (data-service  Base::OrientationToCorrectSrv)
        (data-service  Auv::StructuredLightPairSrv)
        (data-service  Dev::Bus::CAN::ClientInSrv)
        (data-service  Auv::SoundSourceDirectionSrv)
        (data-service  Dev::Bus::CAN::BusOutSrv)
        (data-service  Dev::Sensors::BlueView)
        (data-service  Base::RawCommandControlledSystemSrv)
        (data-service  Dev::Bus::CAN::BusInSrv)
        (data-service  Base::AUVRelativeMotionControlledSystemSrv)
        (data-service  Base::RawCommandControllerSrv)
        (data-service  Base::AUVRelativeMotionControllerSrv)
        (data-service  Base::AUVRelativeMotionCommandConsumerSrv)
        (data-service  Dev::Bus::CAN::BusBaseSrv)
        (data-service  Dev::Bus::CAN)
        (data-service  Base::RawCommandCommandConsumerSrv)
        (data-service  Base::WorldZRollPitchYawSrv)
        (data-service  Base::AUVMotionControlledSystemSrv)
        (data-service  Base::WorldXYVelocityControllerSrv)
        (data-service  Base::AUVMotionControllerSrv)
        (data-service  Base::AUVMotionCommandConsumerSrv)
        (data-service  Base::WorldYPositionXVelocityControllerSrv)
        (data-service  Base::WorldXYZPositionControllerSrv)
        (data-service  Base::Motion2DControlledSystemSrv)
        (data-service  Base::Motion2DControllerSrv)
        (data-service  Base::Motion2DCommandConsumerSrv)
        (data-service  Base::WorldXYPositionControllerSrv)
        (data-service  Base::WorldXYZRollPitchYawControllerSrv)
        (data-service  Base::JointsControlledSystemSrv)
        (data-service  Dev::Actuators::Dynamixel)
        (data-service  Base::WorldXYZRollPitchYawControlledSystemSrv)
        (data-service  Base::JointsControllerSrv)
        (data-service  Dev::Actuators::PTU)
        (data-service  Base::JointsCommandSrv)
        (data-service  Dev::Sensors::TimestamperDev)
        (data-service  Base::JointsStatusSrv)
        (data-service  Dev::Sensors::Cameras::USB)
        (data-service  Base::JointsCommandConsumerSrv)
        (data-service  Dev::Sensors::Cameras::Prosilica)
        (data-service  Dev::Sensors::Cameras::Firewire)
        (data-service  Dev::Sensors::XsensAHRS)
        (data-service  Dev::Sensors::Hokuyo)
        (data-service  Dev::Sensors::GPS)
        (data-service  Base::PointcloudProviderSrv)
        (data-service  Base::SonarScanProviderSrv)
        (data-service  Base::ActuatorControlledSystemSrv)
        (data-service  Base::LaserRangeFinderSrv)
        (data-service  Base::ActuatorControllerSrv)
        (data-service  Base::DistanceImageProviderSrv)
        (data-service  Base::ActuatorStatusSrv)
        (data-service  Base::StereoPairProviderSrv)
        (data-service  Base::ActuatorCommandConsumerSrv)
        (data-service  Base::ImageProviderSrv)
        (data-service  Base::CalibratedIMUSensorsSrv)
        (data-service  Base::CompensatedIMUSensorsSrv)
        (data-service  Base::ControlledSystemSrv)
        (data-service  Base::ControllerSrv)
        (data-service  Base::IMUSensorsSrv)
        (data-service  Dev::Actuators::Lights)
        (data-service  Base::ImageConsumerSrv)
        (data-service  Base::TimestampInputSrv)
        (data-service  Base::TimestamperSrv)
        (data-service  WallServoing::WallOrientationSrv)
        (data-service  Dev::ASVModem)
        (data-service  Dev::Sensors::Hbridge)
        (data-service  Dev::Actuators::Hbridge)
        (data-service  Dev::SystemStatus)
        (data-service  Localization::HoughSrv)
        (data-service  Dev::Sensors::Battery)
        (data-service  Dev::Micron)
        (data-service  Dev::Echosounder)
        (data-service  Dev::Profiling)

            (depends PoseAuv::PoseEstimatorCmp  Base::PositionSrv)
            (depends PoseAuv::PoseEstimatorCmp  PoseEstimation::UWPoseEstimator)
            (depends PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::VelocitySrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::DVLSrv)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvRelPosController::Task)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  PoseEstimation::UWPoseEstimator)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::VelocitySrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::DVLSrv)
            (depends PoseAuv::InitialOrientationEstimatorCmp  WallOrientationCorrection::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  OrientationEstimator::BaseEstimator)
            (depends PoseAuv::InitialOrientationEstimatorCmp  XsensImu::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  FogKvh::Dsp3000Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  Base::SonarScanProviderSrv)
            (depends PoseAuv::InitialOrientationEstimatorCmp  SonarFeatureEstimator::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  PoseAuv::IKFOrientationEstimatorCmp)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends PoseAuv::IKFOrientationEstimatorCmp  OrientationEstimator::BaseEstimator)
            (depends PoseAuv::IKFOrientationEstimatorCmp  WallOrientationCorrection::OrientationInMap)
            (depends PoseAuv::IKFOrientationEstimatorCmp  XsensImu::Task)
            (depends PoseAuv::IKFOrientationEstimatorCmp  FogKvh::Dsp3000Task)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AvalonControl::MotionControlTask)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends ConstantWorldXYVelocityCommand  AuvControl::ConstantCommand)
            (depends ConstantWorldXYVelocityCommand  ConsWA)
            (depends ConsWA  AuvControl::ConstantCommand)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends GPSHelper::GPSWaypointsCmp  GpsHelper::WaypointNavigation)
            (depends GPSHelper::GPSWaypointsCmp  Base::PositionSrv)
            (depends GPSHelper::GPSWaypointsCmp  Base::PoseSrv)
            (depends Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Base::ControlLoop  Base::ControllerSrv)
            (depends Base::ControlLoop  Base::ControlledSystemSrv)
            (depends LowLevel::Cmp  LowLevelDriver::LowLevelTask)
            (depends LowLevel::Cmp  Base::OrientationWithZSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Wall::Detector)
            (depends Wall::Follower__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends Wall::Follower__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends Wall::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__AuvRelPosController::Task__  Wall::Detector)
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Buoy::Detector)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Detector)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends AuvControl::JoystickCommandCmp  Base::RawCommandControllerSrv)
            (depends AuvControl::JoystickCommandCmp  Base::OrientationWithZSrv)
            (depends AuvControl::JoystickCommandCmp  RawControlCommandConverter::Movement)
            (depends AuvControl::JoystickCommandCmp  Base::GroundDistanceSrv)
            (depends Modem::ModemCmp  Dev::ASVModem)
            (depends AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (depends AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (depends AuvControl::DepthFusionCmp  AuvHelper::DepthAndOrientationFusion)
            (depends AuvControl::SimpleMove  AvalonControl::FakeWriter)
            (depends AuvControl::SimpleMove  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Detector)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Structure::Detector  StructureServoing::Task)
            (depends Structure::Detector  HsvMosaicing::Task)
            (depends Structure::Detector  ImagePreprocessing::HSVSegmentationAndBlur)
            (depends Structure::Detector  Base::ImageProviderSrv)
            (depends Structure::Detector  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Detector)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Hbridge::ControlSystem  Dev::Actuators::Hbridge)
            (depends Hbridge::ControlSystem  Dev::Actuators::Hbridge)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends AuvControl::SimplePosMove  AvalonControl::RelFakeWriter)
            (depends AuvControl::SimplePosMove  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Buoy::Detector)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Buoy::DetectorCmp)
            (depends AuvCont::WorldPositionCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldPositionCmp  Base::PoseSrv)
            (depends AuvCont::WorldPositionCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::WorldPositionCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldPositionCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldPositionCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldPositionCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldPositionCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Detector)
            (depends Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Buoy::DetectorCmp)
            (depends blueview_cmp  VideoStreamerVlc::Streamer)
            (depends blueview_cmp  sonar)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Buoy::DetectorCmp)
            (depends front_camera_cmp  VideoStreamerVlc::Streamer)
            (depends front_camera_cmp  driver)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::DetectorCmp)
            (depends bottom_camera_cmp  VideoStreamerVlc::Streamer)
            (depends bottom_camera_cmp  driver)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Structure::Alignment  StructureServoing::Alignment)
            (depends Structure::Alignment  HsvMosaicing::Task)
            (depends Structure::Alignment  ImagePreprocessing::HSVSegmentationAndBlur)
            (depends Structure::Alignment  Base::ImageProviderSrv)
            (depends AuvControl::TrajectoryMove  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Localization::SonarFeatureDetectorCmp  SonarFeatureDetector::Task)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::PoseSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Detector)
            (depends Localization::FixMapHack  SonarFeatureDetector::Task)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  AuvControl::ConstantCommandGroundAvoidance)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  Base::GroundDistanceSrv)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  Base::ZProviderSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Detector)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Localization::HoughDetector  SonarWallHough::Task)
            (depends Localization::HoughDetector  Base::SonarScanProviderSrv)
            (depends Localization::HoughDetector  Base::OrientationSrv)
            (depends Localization::HoughDetector  Base::DVLSrv)
            (depends Localization::HoughDetector  UwParticleLocalization::OrientationCorrection)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends Localization::DeadReckoning  UwParticleLocalization::MotionModel)
            (depends Localization::DeadReckoning  Base::OrientationWithZSrv)
            (depends Localization::DeadReckoning  Base::JointsStatusSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::PoseSrv)
            (depends Pipeline::Detector_new  OffshorePipelineDetector::Task)
            (depends Pipeline::Detector_new  Base::ImageProviderSrv)
            (depends Pipeline::Detector_new  Base::OrientationWithZSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Localization::ParticleDetector  UwParticleLocalization::Task)
            (depends Localization::ParticleDetector  Base::SonarScanProviderSrv)
            (depends Localization::ParticleDetector  SonarFeatureEstimator::Task)
            (depends Localization::ParticleDetector  Base::OrientationWithZSrv)
            (depends Localization::ParticleDetector  Base::JointsStatusSrv)
            (depends Localization::ParticleDetector  SonarFeatureDetector::Task)
            (depends Localization::ParticleDetector  Localization::HoughSrv)
            (depends Localization::ParticleDetector  Base::GroundDistanceSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Detector)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Buoy::FollowerCmp  Buoy::DetectorCmp_Base::ControllerSrv_)
            (depends Buoy::FollowerCmp  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp  Buoy::DetectorCmp)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::DoubleBuoyCmp  Base::MapSrv)
            (depends Buoy::DoubleBuoyCmp  Buoy::DetectorCmp)
            (depends Buoy::DoubleBuoyCmp  Buoy::DetectorCmp2)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::PoseSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Wall::Detector)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::TrajectoryFollower)
            (depends Buoy::DetectorCmp2  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp2  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp2  Buoy::Detector2)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Detector)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Wall::Detector)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Detector)
            (depends AuvCont::BuoyWallCmp  Base::WorldXYZPositionControllerSrv)
            (depends AuvCont::BuoyWallCmp  Base::WorldXYZPositionControllerSrv)
            (depends AuvCont::BuoyWallCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::BuoyWallCmp  Base::PoseSrv)
            (depends AuvCont::BuoyWallCmp  AuvControl::WorldToAligned)
            (depends AuvCont::BuoyWallCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::BuoyWallCmp  AuvControl::PIDController)
            (depends AuvCont::BuoyWallCmp  AuvControl::PIDController)
            (depends AuvCont::BuoyWallCmp  AuvControl::AccelerationController)
            (depends AuvCont::BuoyWallCmp  AuvControl::AlignedToBody)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::PositionControlTask)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Detector)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Detector)
            (depends Pipeline::Detector  OffshorePipelineDetector::Task)
            (depends Pipeline::Detector  Base::ImageProviderSrv)
            (depends Pipeline::Detector  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Detector)
            (depends AuvCont::StructureCmp  Base::WorldXYVelocityControllerSrv)
            (depends AuvCont::StructureCmp  Base::WorldXYVelocityControllerSrv)
            (depends AuvCont::StructureCmp  Base::PoseSrv)
            (depends AuvCont::StructureCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::StructureCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::StructureCmp  AuvControl::WorldToAligned)
            (depends AuvCont::StructureCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::StructureCmp  AuvControl::PIDController)
            (depends AuvCont::StructureCmp  AuvControl::PIDController)
            (depends AuvCont::StructureCmp  AuvControl::AccelerationController)
            (depends AuvCont::StructureCmp  AuvControl::AlignedToBody)
            (depends Structure::StructureReconstructionComp  StructureReconstruction::Task)
            (depends Structure::StructureReconstructionComp  Base::ImageProviderSrv)
            (depends Structure::StructureReconstructionComp  Base::ImageProviderSrv)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Wall::Detector)
            (depends AuvCont::MoveCmp  ConstantWorldXYVelocityCommand)
            (depends AuvCont::MoveCmp  ConstantWorldXYVelocityCommand)
            (depends AuvCont::MoveCmp  Base::PoseSrv)
            (depends AuvCont::MoveCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::MoveCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::MoveCmp  AuvControl::WorldToAligned)
            (depends AuvCont::MoveCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::MoveCmp  AuvControl::PIDController)
            (depends AuvCont::MoveCmp  AuvControl::PIDController)
            (depends AuvCont::MoveCmp  AuvControl::AccelerationController)
            (depends AuvCont::MoveCmp  AuvControl::AlignedToBody)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Detector)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  _Base::ControllerSrv,Base::Motion2DControlledSystemSrv_)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Wall::Detector)
            (depends Pipeline::Follower  Pipeline::Detector)
            (depends Pipeline::Follower  Base::ControlledSystemSrv)
            (depends Pipeline::Follower  Pipeline::Detector)
            (depends AuvCont::PositionMoveCmp  AuvControl::ConstantCommand)
            (depends AuvCont::PositionMoveCmp  AuvControl::ConstantCommand)
            (depends AuvCont::PositionMoveCmp  Base::PoseSrv)
            (depends AuvCont::PositionMoveCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::PositionMoveCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::PositionMoveCmp  AuvControl::WorldToAligned)
            (depends AuvCont::PositionMoveCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::PositionMoveCmp  AuvControl::PIDController)
            (depends AuvCont::PositionMoveCmp  AuvControl::PIDController)
            (depends AuvCont::PositionMoveCmp  AuvControl::AlignedToBody)
            (depends AuvCont::PositionMoveCmp  AuvControl::AccelerationController)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Detector)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Detector)
            (depends Structure::SonarStructureServoingComp  SonarStructureServoing::Task)
            (depends Structure::SonarStructureServoingComp  Base::SonarScanProviderSrv)
            (depends Structure::SonarStructureServoingComp  SonarFeatureEstimator::Task)
            (depends Structure::SonarStructureServoingComp  Base::PoseSrv)
            (depends AuvCont::WorldXYPositionCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldXYPositionCmp  Base::PoseSrv)
            (depends AuvCont::WorldXYPositionCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldXYPositionCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldXYPositionCmp  Base::WorldXYPositionControllerSrv)
            (depends AuvCont::Trajectory  AvalonControl::TrajectoryFollower)
            (depends AuvCont::Trajectory  AvalonControl::TrajectoryFollower)
            (depends AuvCont::Trajectory  Base::PoseSrv)
            (depends AuvCont::Trajectory  Base::JointsControlledSystemSrv)
            (depends AuvCont::Trajectory  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::Trajectory  AuvControl::WorldToAligned)
            (depends AuvCont::Trajectory  AuvControl::OptimalHeadingController)
            (depends AuvCont::Trajectory  AuvControl::PIDController)
            (depends AuvCont::Trajectory  AuvControl::PIDController)
            (depends AuvCont::Trajectory  AuvControl::AlignedToBody)
            (depends AuvCont::Trajectory  AuvControl::AccelerationController)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Wall::Detector)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Wall::Detector)
            (depends AuvCont::WorldYPositionXVelocityCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldYPositionXVelocityCmp  Base::PoseSrv)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::PIDController)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::PIDController)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldYPositionXVelocityCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldYPositionXVelocityCmp  Base::WorldYPositionXVelocityControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  _Base::ActuatorControlledSystemSrv,Base::ControllerSrv_)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Wall::Detector)
            (depends Wall::Follower  Wall::Detector)
            (depends Wall::Follower  Base::ControlledSystemSrv)
            (depends Wall::Follower  Wall::Detector)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  AvalonControl::MotionControlTask)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::GroundDistanceSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  AuvRelPosController::Task)
            (depends Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Pipeline::Detector)
            (depends AuvCont::WorldXYVelocityCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldXYVelocityCmp  Base::PoseSrv)
            (depends AuvCont::WorldXYVelocityCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldXYVelocityCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldXYVelocityCmp  Base::WorldXYVelocityControllerSrv)
            (depends Wall::DetectorNew  WallServoing::SingleSonarServoing)
            (depends Wall::DetectorNew  SonarTritech::Micron)
            (depends Wall::DetectorNew  SonarFeatureEstimator::Task)
            (depends Wall::DetectorNew  Base::OrientationWithZSrv)
            (depends Wall::DetectorNew  Base::VelocitySrv)
            (depends AuvCont::WorldXYZPositionCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldXYZPositionCmp  Base::PoseSrv)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldXYZPositionCmp  Base::WorldXYZPositionControllerSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  AvalonControl::PositionControlTask)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::PoseSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Pipeline::Detector)
            (depends Wall::Detector  WallServoing::SingleSonarServoing)
            (depends Wall::Detector  SonarTritech::Micron)
            (depends Wall::Detector  SonarFeatureEstimator::Task)
            (depends Wall::Detector  Base::OrientationWithZSrv)
            (depends Wall::Detector  Base::VelocitySrv)
            (depends Buoy::DetectorCmp  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp  Buoy::Detector)
            (depends Buoy::ControllerNewCmp  Buoy::ServoingOnWall)
            (depends Buoy::ControllerNewCmp  WallServoing::WallOrientationSrv)
            (depends Buoy::ControllerNewCmp  Base::OrientationSrv)
            (depends Buoy::ControllerNewCmp  Buoy::DetectorNewCmp)
            (depends Buoy::DetectorNewCmp  Buoy::Detector)
            (depends Buoy::DetectorNewCmp  Base::ImageProviderSrv)
            
; Begin fullfillments 
            (fullfills AvalonControl::TrajectoryFollower  Base::AUVRelativeMotionControllerSrv)
            (fullfills AvalonControl::TrajectoryFollower  Base::ControllerSrv)
            (fullfills AvalonControl::TrajectoryFollower  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills FogKvh::Dsp3000Task  Dev::Sensors::KVH::DSP3000)
            (fullfills FogKvh::Dsp3000Task  Base::RotationSrv)
            (fullfills FogKvh::Dsp3000Task  Syskit::Device)
            (fullfills ModemCan::Task  Dev::Sensors::Modem)
            (fullfills ModemCan::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills ModemCan::Task  Syskit::Device)
            (fullfills AvalonControl::RelFakeWriter  Base::AUVRelativeMotionControllerSrv)
            (fullfills AvalonControl::RelFakeWriter  Base::ControllerSrv)
            (fullfills DepthReader::Task  Dev::Sensors::DepthReaderAvalon)
            (fullfills DepthReader::Task  Dev::Bus::CAN::ClientOutSrv)
            (fullfills DepthReader::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills DepthReader::Task  Base::ZProviderSrv)
            (fullfills DepthReader::Task  Syskit::Device)
            (fullfills AvalonControl::FakeWriter  Base::AUVMotionControllerSrv)
            (fullfills AvalonControl::FakeWriter  Base::ControllerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::AUVMotionControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask  Base::AUVMotionCommandConsumerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::ControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask  Base::JointsControllerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::ControllerSrv)
            (fullfills Controldev::Remote  Dev::Bus::CAN::ClientInSrv)
            (fullfills Controldev::Remote  Dev::Controldev::Joystick)
            (fullfills Controldev::Remote  Base::RawCommandControllerSrv)
            (fullfills Controldev::Remote  Base::ControllerSrv)
            (fullfills Controldev::Remote  Syskit::Device)
            (fullfills Canbus::Task  Dev::Bus::CAN)
            (fullfills Canbus::Task  Syskit::ComBus)
            (fullfills Canbus::Task  Syskit::Device)
            (fullfills Canbus::Task  Dev::Bus::CAN::BusInSrv)
            (fullfills Canbus::Task  Dev::Bus::CAN::BusBaseSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AvalonControl::PositionControlTask  Base::ControlledSystemSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVMotionControllerSrv)
            (fullfills AvalonControl::PositionControlTask  Base::ControllerSrv)
            (fullfills CameraProsilica::Task  Dev::Sensors::Cameras::Prosilica)
            (fullfills CameraProsilica::Task  Base::ImageProviderSrv)
            (fullfills CameraProsilica::Task  Syskit::Device)
            (fullfills Controldev::Mouse3DTask  Dev::Controldev::Mouse3D)
            (fullfills Controldev::Mouse3DTask  Base::RawCommandControllerSrv)
            (fullfills Controldev::Mouse3DTask  Base::ControllerSrv)
            (fullfills Controldev::Mouse3DTask  Syskit::Device)
            (fullfills Controldev::JoystickTask  Dev::Controldev::Joystick)
            (fullfills Controldev::JoystickTask  Base::RawCommandControllerSrv)
            (fullfills Controldev::JoystickTask  Base::ControllerSrv)
            (fullfills Controldev::JoystickTask  Syskit::Device)
            (fullfills Lights::Lights  Dev::Actuators::Lights)
            (fullfills Lights::Lights  Dev::Bus::CAN::ClientInSrv)
            (fullfills Lights::Lights  Syskit::Device)
            (fullfills XsensImu::Task  Dev::Sensors::XsensAHRS)
            (fullfills XsensImu::Task  Base::CalibratedIMUSensorsSrv)
            (fullfills XsensImu::Task  Base::IMUSensorsSrv)
            (fullfills XsensImu::Task  Base::OrientationSrv)
            (fullfills XsensImu::Task  Syskit::Device)
            (fullfills XsensImu::Task  Base::TimestampInputSrv)
            (fullfills AuvControl::ConstantCommand  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills AuvRelPosController::Task  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvRelPosController::Task  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvRelPosController::Task  Base::ControlledSystemSrv)
            (fullfills AuvRelPosController::Task  Base::AUVMotionControllerSrv)
            (fullfills AuvRelPosController::Task  Base::ControllerSrv)
            (fullfills SonarBlueview::Task  Dev::Sensors::BlueView)
            (fullfills SonarBlueview::Task  Base::ImageProviderSrv)
            (fullfills SonarBlueview::Task  Syskit::Device)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills Gps::GPSDTask  Dev::Sensors::GPS)
            (fullfills Gps::GPSDTask  Base::PositionSrv)
            (fullfills Gps::GPSDTask  Syskit::Device)
            (fullfills Modemdriver::ModemCanbus  Dev::ASVModem)
            (fullfills Modemdriver::ModemCanbus  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Modemdriver::ModemCanbus  Syskit::Device)
            (fullfills Dynamixel::Task  Dev::Actuators::Dynamixel)
            (fullfills Dynamixel::Task  Syskit::Device)
            (fullfills Dynamixel::Task  Base::TransformationSrv)
            (fullfills Sysmon::Task  Dev::SystemStatus)
            (fullfills Sysmon::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills Sysmon::Task  Syskit::Device)
            (fullfills Gps::BaseTask  Dev::Sensors::GPS)
            (fullfills Gps::BaseTask  Base::PositionSrv)
            (fullfills Gps::BaseTask  Syskit::Device)
            (fullfills Hbridge::CommandWriter  Dev::Actuators::Hbridge)
            (fullfills Hbridge::CommandWriter  Syskit::Device)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientSrv)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientInSrv)
            (fullfills Hbridge::CommandWriter  Base::JointsStatusSrv)
            (fullfills Hbridge::SensorReader  Dev::Sensors::Hbridge)
            (fullfills Hbridge::SensorReader  Syskit::Device)
            (fullfills Hbridge::SensorReader  Base::JointsStatusSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientInSrv)
            (fullfills BatteryWatcher::Task  Dev::Sensors::Battery)
            (fullfills BatteryWatcher::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills BatteryWatcher::Task  Syskit::Device)
            (fullfills Gps::MB500Task  Dev::Sensors::GPS)
            (fullfills Gps::MB500Task  Base::PositionSrv)
            (fullfills Gps::MB500Task  Syskit::Device)
            (fullfills SonarTritech::Micron  Dev::Micron)
            (fullfills SonarTritech::Micron  Base::GroundDistanceSrv)
            (fullfills SonarTritech::Micron  Base::SonarScanProviderSrv)
            (fullfills SonarTritech::Micron  Syskit::Device)
            (fullfills SonarTritech::Echosounder  Dev::Echosounder)
            (fullfills SonarTritech::Echosounder  Base::GroundDistanceSrv)
            (fullfills SonarTritech::Echosounder  Syskit::Device)
            (fullfills PoseAuv::PoseEstimatorCmp  PoseAuv::PoseEstimatorCmp)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::PoseSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::PositionSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::PoseSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::PositionSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorCmp  Base::PositionSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__AuvRelPosController::Task_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  PoseAuv::PoseEstimatorBlindCmp)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::PoseSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::PositionSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::PoseSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::PositionSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationWithZSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (fullfills PoseAuv::PoseEstimatorBlindCmp  Base::PositionSrv)
            (fullfills PoseAuv::InitialOrientationEstimatorCmp  PoseAuv::InitialOrientationEstimatorCmp)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  PoseAuv::IKFOrientationEstimatorCmp)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationToCorrectSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationToCorrectSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills ConstantWorldXYVelocityCommand  ConstantWorldXYVelocityCommand)
            (fullfills ConstantWorldXYVelocityCommand  Base::WorldXYVelocityControllerSrv)
            (fullfills ConstantWorldXYVelocityCommand  Base::WorldXYVelocityControllerSrv)
            (fullfills ConsWA  ConsWA)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills GPSHelper::GPSWaypointsCmp  GPSHelper::GPSWaypointsCmp)
            (fullfills GPSHelper::GPSWaypointsCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills GPSHelper::GPSWaypointsCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop  Base::ControlLoop)
            (fullfills LowLevel::Cmp  LowLevel::Cmp)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Wall::Follower__controller.is_a__AuvRelPosController::Task__)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Buoy::DetectorCmp)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::AUVRelativeMotionControllerSrv)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ControllerSrv)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ControllerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop__controller.is_a__AuvRelPosController::Task__)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::JoystickCommandCmp  AuvControl::JoystickCommandCmp)
            (fullfills AuvControl::JoystickCommandCmp  Base::AUVMotionControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::WorldXYVelocityControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::AUVMotionControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::WorldXYVelocityControllerSrv)
            (fullfills Modem::ModemCmp  Modem::ModemCmp)
            (fullfills AuvControl::DepthFusionCmp  AuvControl::DepthFusionCmp)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::SimpleMove  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Structure::Detector  Structure::Detector)
            (fullfills Structure::Detector  Base::WorldXYVelocityControllerSrv)
            (fullfills Structure::Detector  Base::WorldXYVelocityControllerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Hbridge::ControlSystem  Hbridge::ControlSystem)
            (fullfills Hbridge::ControlSystem  Base::JointsControlledSystemSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsStatusSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsCommandConsumerSrv)
            (fullfills Hbridge::ControlSystem  Base::ControlledSystemSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsControlledSystemSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsStatusSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsCommandConsumerSrv)
            (fullfills Hbridge::ControlSystem  Base::ControlledSystemSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsStatusSrv)
            (fullfills Hbridge::ControlSystem  Base::JointsCommandConsumerSrv)
            (fullfills Hbridge::ControlSystem  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp_Base::ControllerSrv_  Buoy::DetectorCmp)
            (fullfills Buoy::DetectorCmp_Base::ControllerSrv_  Base::ControllerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvCont::WorldPositionCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::WorldPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills blueview_cmp  blueview_cmp)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills front_camera_cmp  front_camera_cmp)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills bottom_camera_cmp  bottom_camera_cmp)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Structure::Alignment  Structure::Alignment)
            (fullfills Structure::Alignment  Base::WorldXYVelocityControllerSrv)
            (fullfills Structure::Alignment  Base::WorldXYVelocityControllerSrv)
            (fullfills AuvControl::TrajectoryMove  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::SonarFeatureDetectorCmp  Localization::SonarFeatureDetectorCmp)
            (fullfills Localization::SonarFeatureDetectorCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills Localization::SonarFeatureDetectorCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::FixMapHack  Localization::FixMapHack)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::ConstantCommandGroundAvoidanceCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::HoughDetector  Localization::HoughDetector)
            (fullfills Localization::HoughDetector  Localization::HoughSrv)
            (fullfills Localization::HoughDetector  Localization::HoughSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::DeadReckoning  Localization::DeadReckoning)
            (fullfills Localization::DeadReckoning  Base::VelocitySrv)
            (fullfills Localization::DeadReckoning  Base::VelocitySrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Detector_new  Pipeline::Detector_new)
            (fullfills Pipeline::Detector_new  Base::WorldXYPositionControllerSrv)
            (fullfills Pipeline::Detector_new  Base::WorldXYPositionControllerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::ParticleDetector  Localization::ParticleDetector)
            (fullfills Localization::ParticleDetector  Base::VelocitySrv)
            (fullfills Localization::ParticleDetector  Base::PoseSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationWithZSrv)
            (fullfills Localization::ParticleDetector  Base::ZProviderSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationSrv)
            (fullfills Localization::ParticleDetector  Base::PositionSrv)
            (fullfills Localization::ParticleDetector  Base::MapSrv)
            (fullfills Localization::ParticleDetector  Base::VelocitySrv)
            (fullfills Localization::ParticleDetector  Base::PoseSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationWithZSrv)
            (fullfills Localization::ParticleDetector  Base::ZProviderSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationSrv)
            (fullfills Localization::ParticleDetector  Base::PositionSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationWithZSrv)
            (fullfills Localization::ParticleDetector  Base::ZProviderSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationSrv)
            (fullfills Localization::ParticleDetector  Base::ZProviderSrv)
            (fullfills Localization::ParticleDetector  Base::OrientationSrv)
            (fullfills Localization::ParticleDetector  Base::PositionSrv)
            (fullfills Localization::ParticleDetector  Base::MapSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::DoubleBuoyCmp  Buoy::DoubleBuoyCmp)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp2  Buoy::DetectorCmp2)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::BuoyWallCmp  AuvCont::BuoyWallCmp)
            (fullfills AuvCont::BuoyWallCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::BuoyWallCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Base::ControlLoop__controller.is_a__AvalonControl::PositionControlTask_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Detector  Pipeline::Detector)
            (fullfills Pipeline::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills Pipeline::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::StructureCmp  AuvCont::StructureCmp)
            (fullfills AuvCont::StructureCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::StructureCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
            (fullfills Structure::StructureReconstructionComp  Structure::StructureReconstructionComp)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_, controlled_system.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControllerSrv_,controlled_system.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::MoveCmp  AuvCont::MoveCmp)
            (fullfills AuvCont::MoveCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::MoveCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::Motion2DCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::Motion2DControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower  Pipeline::Follower)
            (fullfills Pipeline::Follower  Base::ControlLoop)
            (fullfills Pipeline::Follower  Base::ControlLoop)
            (fullfills AuvCont::PositionMoveCmp  AuvCont::PositionMoveCmp)
            (fullfills AuvCont::PositionMoveCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::PositionMoveCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Structure::SonarStructureServoingComp  Structure::SonarStructureServoingComp)
            (fullfills Structure::SonarStructureServoingComp  Base::AUVRelativeMotionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::WorldXYPositionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::AUVRelativeMotionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::WorldXYPositionControllerSrv)
            (fullfills AuvCont::WorldXYPositionCmp  AuvCont::WorldXYPositionCmp)
            (fullfills AuvCont::WorldXYPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  AuvCont::Trajectory)
            (fullfills AuvCont::Trajectory  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::Trajectory  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_, controlled_system.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControllerSrv_,controlled_system.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  AuvCont::WorldYPositionXVelocityCmp)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  Base::JointsCommandSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorStatusSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ActuatorCommandConsumerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::ActuatorControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower  Wall::Follower)
            (fullfills Wall::Follower  Base::ControlLoop)
            (fullfills Wall::Follower  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::MotionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Pipeline::Follower__controller.is_a__AuvRelPosController::Task__)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AuvRelPosController::Task__  Base::ControlLoop)
            (fullfills AuvCont::WorldXYVelocityCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::WorldXYVelocityCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYVelocityCmp  Base::JointsCommandSrv)
            (fullfills Wall::DetectorNew  Wall::DetectorNew)
            (fullfills Wall::DetectorNew  WallServoing::WallOrientationSrv)
            (fullfills Wall::DetectorNew  Base::WorldXYPositionControllerSrv)
            (fullfills Wall::DetectorNew  WallServoing::WallOrientationSrv)
            (fullfills Wall::DetectorNew  Base::WorldXYPositionControllerSrv)
            (fullfills AuvCont::WorldXYZPositionCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::WorldXYZPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYZPositionCmp  Base::JointsCommandSrv)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__AvalonControl::PositionControlTask__  Base::ControlLoop)
            (fullfills Wall::Detector  Wall::Detector)
            (fullfills Wall::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
            (fullfills Wall::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
            (fullfills Buoy::DetectorCmp  Buoy::DetectorCmp)
            (fullfills Buoy::ControllerNewCmp  Buoy::ControllerNewCmp)
            (fullfills Buoy::ControllerNewCmp  Base::WorldXYZPositionControllerSrv)
            (fullfills Buoy::ControllerNewCmp  Base::WorldXYZPositionControllerSrv)
            (fullfills Buoy::DetectorNewCmp  Buoy::DetectorNewCmp)

; Begin inputs 

; Begin outputs 
; Fix encoded knoeledge
            (is-root root)

; Begin requirements
            (depends root  Pipeline::Follower)

  )
  (:goal (and


    (forall (?t - instance_req)
    (exists (?r - instance_req)
            (imply
            (is-running ?t)
            (requests ?r ?t)
            ;(depends ?r ?t)
            )
            )
            )
    
    (is-running root)
  
;    (forall (?t - instance_req)
;    (forall (?r - instance_req)
;            (imply
;            (requests ?r ?t)
;            (is-running ?t)
;            )
;            )
;            )
;
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
