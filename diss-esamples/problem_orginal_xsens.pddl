(define (problem network001)
  (:domain network)
  (:objects
        root - instance_req
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
        Gps::MB500Task.solution - output_port
        Gps::MB500Task.position_samples - output_port
        Gps::MB500Task.state - output_port
        Gps::MB500Task.constellation - output_port
        Gps::MB500Task.time - output_port
        Gps::MB500Task - instance_req
        Gps::GPSDTask.solution - output_port
        Gps::GPSDTask.position_samples - output_port
        Gps::GPSDTask.state - output_port
        Gps::GPSDTask - instance_req
        Gps::BaseTask.solution - output_port
        Gps::BaseTask.position_samples - output_port
        Gps::BaseTask.state - output_port
        Gps::BaseTask - instance_req
        Canbus::InterfaceTask.can_out - output_port
        Canbus::InterfaceTask.state - output_port
        Canbus::InterfaceTask.can_in - input_port
        Canbus::InterfaceTask - instance_req
        VideoStreamerVlc::Streamer{1}.state - output_port
        VideoStreamerVlc::Streamer{1}.frame_bottom_camera - input_port
        VideoStreamerVlc::Streamer{1}.frame_front_camera - input_port
        VideoStreamerVlc::Streamer{1}.frame_blueview - input_port
        VideoStreamerVlc::Streamer{1} - instance_req
        Canbus::Task.stats - output_port
        Canbus::Task.can_status - output_port
        Canbus::Task.log_message - output_port
        Canbus::Task.state - output_port
        Canbus::Task.in - input_port
        Canbus::Task - instance_req
        Transformer::Task.configuration_state - output_port
        Transformer::Task.state - output_port
        Transformer::Task - instance_req
        VideoStreamerVlc::Capturer.state - output_port
        VideoStreamerVlc::Capturer - instance_req
        Buoy::ServoingOnWall.world_cmd - output_port
        Buoy::ServoingOnWall.aligned_position_cmd - output_port
        Buoy::ServoingOnWall.state - output_port
        Buoy::ServoingOnWall.buoy_samples - input_port
        Buoy::ServoingOnWall.wall_samples - input_port
        Buoy::ServoingOnWall.orientation_samples - input_port
        Buoy::ServoingOnWall - instance_req
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
        VideoStreamerVlc::Streamer.state - output_port
        VideoStreamerVlc::Streamer.frame_bottom_camera - input_port
        VideoStreamerVlc::Streamer.frame_front_camera - input_port
        VideoStreamerVlc::Streamer.frame_blueview - input_port
        VideoStreamerVlc::Streamer - instance_req
        XsensImu::Task.orientation_samples - output_port
        XsensImu::Task.calibrated_sensors - output_port
        XsensImu::Task.timestamp_estimator_status - output_port
        XsensImu::Task.state - output_port
        XsensImu::Task.hard_timestamps - input_port
        XsensImu::Task - instance_req
        PipelineInspection::LaserSimulation.laserPoints - output_port
        PipelineInspection::LaserSimulation.laserPointCloud - output_port
        PipelineInspection::LaserSimulation.vehiclePos - output_port
        PipelineInspection::LaserSimulation.state - output_port
        PipelineInspection::LaserSimulation - instance_req
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
        Logger::Logger.state - output_port
        Logger::Logger - instance_req
        PipelineInspection::ColorFilter.frame_out - output_port
        PipelineInspection::ColorFilter.green_frame - output_port
        PipelineInspection::ColorFilter.diff_frame - output_port
        PipelineInspection::ColorFilter.state - output_port
        PipelineInspection::ColorFilter.frame_in - input_port
        PipelineInspection::ColorFilter - instance_req
        Syskit::ROS::Node - instance_req
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
        FogKvh::Dsp3000Task.rotation - output_port
        FogKvh::Dsp3000Task.orientation_samples - output_port
        FogKvh::Dsp3000Task.timestamp_estimator_status - output_port
        FogKvh::Dsp3000Task.state - output_port
        FogKvh::Dsp3000Task.config - input_port
        FogKvh::Dsp3000Task - instance_req
        OrientationEstimator::IKF.transformer_stream_aligner_status - output_port
        OrientationEstimator::IKF.transformer_status - output_port
        OrientationEstimator::IKF.attitude_b_g - output_port
        OrientationEstimator::IKF.state - output_port
        OrientationEstimator::IKF.imu_samples - input_port
        OrientationEstimator::IKF.fog_samples - input_port
        OrientationEstimator::IKF.initial_orientation - input_port
        OrientationEstimator::IKF.dynamic_transformations - input_port
        OrientationEstimator::IKF - instance_req
        UwParticleLocalization::FastFusion.pose_samples - output_port
        UwParticleLocalization::FastFusion.state - output_port
        UwParticleLocalization::FastFusion.position_samples - input_port
        UwParticleLocalization::FastFusion.depth_samples - input_port
        UwParticleLocalization::FastFusion.orientation_samples - input_port
        UwParticleLocalization::FastFusion.velocity_samples - input_port
        UwParticleLocalization::FastFusion - instance_req
        UwParticleLocalization::MotionModel.pose_samples - output_port
        UwParticleLocalization::MotionModel.stream_aligner_status - output_port
        UwParticleLocalization::MotionModel.state - output_port
        UwParticleLocalization::MotionModel.thruster_samples - input_port
        UwParticleLocalization::MotionModel.orientation_samples - input_port
        UwParticleLocalization::MotionModel - instance_req
        LowLevelDriver::LowLevelTask.state - output_port
        LowLevelDriver::LowLevelTask.depth_samples - input_port
        LowLevelDriver::LowLevelTask.ShortExposure - input_port
        LowLevelDriver::LowLevelTask.LongExposure - input_port
        LowLevelDriver::LowLevelTask.LightValue - input_port
        LowLevelDriver::LowLevelTask.DebugLED - input_port
        LowLevelDriver::LowLevelTask.LaserRate - input_port
        LowLevelDriver::LowLevelTask - instance_req
        OrientationEstimator::BaseEstimator.stream_aligner_status - output_port
        OrientationEstimator::BaseEstimator.attitude_b_g - output_port
        OrientationEstimator::BaseEstimator.state - output_port
        OrientationEstimator::BaseEstimator.imu_orientation - input_port
        OrientationEstimator::BaseEstimator.fog_samples - input_port
        OrientationEstimator::BaseEstimator.heading_correction - input_port
        OrientationEstimator::BaseEstimator - instance_req
        StructureReconstruction::Task.transformer_stream_aligner_status - output_port
        StructureReconstruction::Task.transformer_status - output_port
        StructureReconstruction::Task.state - output_port
        StructureReconstruction::Task.front_camera - input_port
        StructureReconstruction::Task.bottom_camera - input_port
        StructureReconstruction::Task.dynamic_transformations - input_port
        StructureReconstruction::Task - instance_req
        UwParticleLocalization::OrientationCorrection.orientation_output - output_port
        UwParticleLocalization::OrientationCorrection.orientation_offset_corrected - output_port
        UwParticleLocalization::OrientationCorrection.state - output_port
        UwParticleLocalization::OrientationCorrection.orientation_input - input_port
        UwParticleLocalization::OrientationCorrection.orientation_offset - input_port
        UwParticleLocalization::OrientationCorrection - instance_req
        LineScanner::Task.state - output_port
        LineScanner::Task.pointcloud - output_port
        LineScanner::Task.debug - output_port
        LineScanner::Task.frame - input_port
        LineScanner::Task - instance_req
        Syskit::RubyTaskContext - instance_req
        AuvRelPosController::Task.motion_command - output_port
        AuvRelPosController::Task.state - output_port
        AuvRelPosController::Task.position_command - input_port
        AuvRelPosController::Task.position_sample - input_port
        AuvRelPosController::Task - instance_req
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
        ModemCan::Task.modem_out - output_port
        ModemCan::Task.canOut - output_port
        ModemCan::Task.motion_command - output_port
        ModemCan::Task.state - output_port
        ModemCan::Task.modem_in - input_port
        ModemCan::Task.canModem - input_port
        ModemCan::Task.light_value - input_port
        ModemCan::Task.position_samples - input_port
        ModemCan::Task - instance_req
        DepthReader::Task.depthOut - output_port
        DepthReader::Task.depth_samples - output_port
        DepthReader::Task.canOut - output_port
        DepthReader::Task.state - output_port
        DepthReader::Task.canIn - input_port
        DepthReader::Task - instance_req
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
        RTT::TaskContext - instance_req
        Lights::Lights.light_value - output_port
        Lights::Lights.state - output_port
        Lights::Lights.int_in - input_port
        Lights::Lights.can_in - input_port
        Lights::Lights - instance_req
        WallServoing::DualSonarServoing.state - output_port
        WallServoing::DualSonarServoing.position_command - output_port
        WallServoing::DualSonarServoing.aligned_command - output_port
        WallServoing::DualSonarServoing.wall_servoing_debug - output_port
        WallServoing::DualSonarServoing.sonarbeam_feature_front - input_port
        WallServoing::DualSonarServoing.sonarbeam_feature_rear - input_port
        WallServoing::DualSonarServoing.orientation_sample - input_port
        WallServoing::DualSonarServoing - instance_req
        AvalonControl::MotionFeedbackTask.hbridge_status - output_port
        AvalonControl::MotionFeedbackTask.state - output_port
        AvalonControl::MotionFeedbackTask.hbridge_feedback - input_port
        AvalonControl::MotionFeedbackTask - instance_req
        BatteryWatcher::Task.can_out - output_port
        BatteryWatcher::Task.state - output_port
        BatteryWatcher::Task.battery_info - output_port
        BatteryWatcher::Task.can_in - input_port
        BatteryWatcher::Task - instance_req
        ImagePreprocessing::DepthImage2Pointcloud.pointcloud - output_port
        ImagePreprocessing::DepthImage2Pointcloud.stream_aligner_status - output_port
        ImagePreprocessing::DepthImage2Pointcloud.transformer_stream_aligner_status - output_port
        ImagePreprocessing::DepthImage2Pointcloud.transformer_status - output_port
        ImagePreprocessing::DepthImage2Pointcloud.state - output_port
        ImagePreprocessing::DepthImage2Pointcloud.color_frame - input_port
        ImagePreprocessing::DepthImage2Pointcloud.frame - input_port
        ImagePreprocessing::DepthImage2Pointcloud.dynamic_transformations - input_port
        ImagePreprocessing::DepthImage2Pointcloud - instance_req
        AuvControl::AccelerationController.state - output_port
        AuvControl::AccelerationController.cmd_out - output_port
        AuvControl::AccelerationController.cmd_in - input_port
        AuvControl::AccelerationController.cmd_cascade - input_port
        AuvControl::AccelerationController - instance_req
        SonarTritech::Micron.ground_distance - output_port
        SonarTritech::Micron.sonar_beam - output_port
        SonarTritech::Micron.state - output_port
        SonarTritech::Micron - instance_req
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
        SonarFeatureEstimator::Task.new_feature - output_port
        SonarFeatureEstimator::Task.features_out - output_port
        SonarFeatureEstimator::Task.debug_output - output_port
        SonarFeatureEstimator::Task.2d_debug_output - output_port
        SonarFeatureEstimator::Task.state - output_port
        SonarFeatureEstimator::Task.sonar_input - input_port
        SonarFeatureEstimator::Task.orientation_sample - input_port
        SonarFeatureEstimator::Task - instance_req
        AvalonControl::TrajectoryFollower.next_position - output_port
        AvalonControl::TrajectoryFollower.position_command - output_port
        AvalonControl::TrajectoryFollower.next_pos_on_spline - output_port
        AvalonControl::TrajectoryFollower.last_pos_on_spline - output_port
        AvalonControl::TrajectoryFollower.segment_dist - output_port
        AvalonControl::TrajectoryFollower.world_command - output_port
        AvalonControl::TrajectoryFollower.state - output_port
        AvalonControl::TrajectoryFollower.pose_samples - input_port
        AvalonControl::TrajectoryFollower - instance_req
        WallServoing::SonarServoing.state - output_port
        WallServoing::SonarServoing - instance_req
        ImagePreprocessing::BaseTask.state - output_port
        ImagePreprocessing::BaseTask - instance_req
        PoseEstimation::BaseTask.pose_samples - output_port
        PoseEstimation::BaseTask.state - output_port
        PoseEstimation::BaseTask - instance_req
        AuvControl::AlignedToBody.state - output_port
        AuvControl::AlignedToBody.cmd_out - output_port
        AuvControl::AlignedToBody.cmd_in - input_port
        AuvControl::AlignedToBody.cmd_cascade - input_port
        AuvControl::AlignedToBody.orientation_samples - input_port
        AuvControl::AlignedToBody - instance_req
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
        PoseEstimation::UWPoseEstimator.gps_position_samples - input_port
        PoseEstimation::UWPoseEstimator.xyz_position_samples - input_port
        PoseEstimation::UWPoseEstimator.dynamic_transformations - input_port
        PoseEstimation::UWPoseEstimator - instance_req
        PoseEstimation::HighDelayPoseEstimator.pose_samples - output_port
        PoseEstimation::HighDelayPoseEstimator.state - output_port
        PoseEstimation::HighDelayPoseEstimator.transformer_stream_aligner_status - output_port
        PoseEstimation::HighDelayPoseEstimator.transformer_status - output_port
        PoseEstimation::HighDelayPoseEstimator.pose_samples_fast - input_port
        PoseEstimation::HighDelayPoseEstimator.pose_samples_slow - input_port
        PoseEstimation::HighDelayPoseEstimator.xy_position_samples - input_port
        PoseEstimation::HighDelayPoseEstimator.dynamic_transformations - input_port
        PoseEstimation::HighDelayPoseEstimator - instance_req
        PoseEstimation::VehiclePoseEstimator.pose_samples - output_port
        PoseEstimation::VehiclePoseEstimator.state - output_port
        PoseEstimation::VehiclePoseEstimator.transformer_stream_aligner_status - output_port
        PoseEstimation::VehiclePoseEstimator.transformer_status - output_port
        PoseEstimation::VehiclePoseEstimator.orientation_samples - input_port
        PoseEstimation::VehiclePoseEstimator.velocity_samples - input_port
        PoseEstimation::VehiclePoseEstimator.position_samples - input_port
        PoseEstimation::VehiclePoseEstimator.dynamic_transformations - input_port
        PoseEstimation::VehiclePoseEstimator - instance_req
        WallServoing::WallServoing.motion_command - output_port
        WallServoing::WallServoing.world_command - output_port
        WallServoing::WallServoing.aligned_velocity_command - output_port
        WallServoing::WallServoing.state - output_port
        WallServoing::WallServoing.orientation_sample - input_port
        WallServoing::WallServoing.servoing_wall - input_port
        WallServoing::WallServoing.obstacle_wall - input_port
        WallServoing::WallServoing - instance_req
        CameraProsilica::Task.frame - output_port
        CameraProsilica::Task.frame_raw - output_port
        CameraProsilica::Task.state - output_port
        CameraProsilica::Task - instance_req
        WallServoing::WallDetector.point_cloud - output_port
        WallServoing::WallDetector.wall - output_port
        WallServoing::WallDetector.state - output_port
        WallServoing::WallDetector.sonarbeam_feature - input_port
        WallServoing::WallDetector.orientation_sample - input_port
        WallServoing::WallDetector.position_sample - input_port
        WallServoing::WallDetector - instance_req
        SonarTritech::Echosounder.ground_distance - output_port
        SonarTritech::Echosounder.state - output_port
        SonarTritech::Echosounder - instance_req
        SonarTritech::Profiling.profiling_scan - output_port
        SonarTritech::Profiling.state - output_port
        SonarTritech::Profiling - instance_req
        Interfaces::Servo.upper2lower - output_port
        Interfaces::Servo.angle - output_port
        Interfaces::Servo.state - output_port
        Interfaces::Servo.cmd_angle - input_port
        Interfaces::Servo - instance_req
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.hbridge_commands - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.joint_commands - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.debug - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.estimated_ground_pos - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.state - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.command_out - output_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.dummy_feedback - input_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.ground_distance - input_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.pose_samples - input_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.motion_commands - input_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.joints_in - input_port
        AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ - instance_req
        AuvControl::WorldToAligned.state - output_port
        AuvControl::WorldToAligned.cmd_out - output_port
        AuvControl::WorldToAligned.cmd_in - input_port
        AuvControl::WorldToAligned.cmd_cascade - input_port
        AuvControl::WorldToAligned.pose_samples - input_port
        AuvControl::WorldToAligned - instance_req
        CameraBase::Preprocess.state - output_port
        CameraBase::Preprocess.oframe - output_port
        CameraBase::Preprocess.iframe - input_port
        CameraBase::Preprocess - instance_req
        AuvControl::PIDController.state - output_port
        AuvControl::PIDController.cmd_out - output_port
        AuvControl::PIDController.pid_state - output_port
        AuvControl::PIDController.cmd_in - input_port
        AuvControl::PIDController.cmd_cascade - input_port
        AuvControl::PIDController.pose_samples - input_port
        AuvControl::PIDController - instance_req
        AvalonControl::RelFakeWriter.position_command - output_port
        AvalonControl::RelFakeWriter.state - output_port
        AvalonControl::RelFakeWriter - instance_req
        AvalonControl::FakeWriter.motion_commands - output_port
        AvalonControl::FakeWriter.state - output_port
        AvalonControl::FakeWriter - instance_req
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
        AuvControl::BasePIDController.state - output_port
        AuvControl::BasePIDController.cmd_out - output_port
        AuvControl::BasePIDController.pid_state - output_port
        AuvControl::BasePIDController.cmd_in - input_port
        AuvControl::BasePIDController.cmd_cascade - input_port
        AuvControl::BasePIDController.pose_samples - input_port
        AuvControl::BasePIDController - instance_req
        Modemdriver::ModemCanbus.data_out - output_port
        Modemdriver::ModemCanbus.distance - output_port
        Modemdriver::ModemCanbus.out_modem_substates - output_port
        Modemdriver::ModemCanbus.state - output_port
        Modemdriver::ModemCanbus.can_out - output_port
        Modemdriver::ModemCanbus.stats - output_port
        Modemdriver::ModemCanbus.data_in - input_port
        Modemdriver::ModemCanbus.can_in - input_port
        Modemdriver::ModemCanbus - instance_req
        Hbridge::CommandWriter.state - output_port
        Hbridge::CommandWriter.can_out - output_port
        Hbridge::CommandWriter.speedCtrlDebug - output_port
        Hbridge::CommandWriter.fakeReader - output_port
        Hbridge::CommandWriter.can_in - input_port
        Hbridge::CommandWriter.command - input_port
        Hbridge::CommandWriter - instance_req
        Modemdriver::ModemSerial.data_out - output_port
        Modemdriver::ModemSerial.distance - output_port
        Modemdriver::ModemSerial.out_modem_substates - output_port
        Modemdriver::ModemSerial.state - output_port
        Modemdriver::ModemSerial.data_in - input_port
        Modemdriver::ModemSerial - instance_req
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
        AvalonControl::PositionControlTask.motion_commands - output_port
        AvalonControl::PositionControlTask.state - output_port
        AvalonControl::PositionControlTask.pose_samples - input_port
        AvalonControl::PositionControlTask.position_commands - input_port
        AvalonControl::PositionControlTask - instance_req
        HsvMosaicing::Task.result - output_port
        HsvMosaicing::Task.state - output_port
        HsvMosaicing::Task.frame - input_port
        HsvMosaicing::Task - instance_req
        Modemdriver::Modem.data_out - output_port
        Modemdriver::Modem.distance - output_port
        Modemdriver::Modem.out_modem_substates - output_port
        Modemdriver::Modem.state - output_port
        Modemdriver::Modem.data_in - input_port
        Modemdriver::Modem - instance_req
        AuvHelper::DepthAndOrientationFusion.pose_samples - output_port
        AuvHelper::DepthAndOrientationFusion.stream_aligner_status - output_port
        AuvHelper::DepthAndOrientationFusion.state - output_port
        AuvHelper::DepthAndOrientationFusion.orientation_samples - input_port
        AuvHelper::DepthAndOrientationFusion.depth_samples - input_port
        AuvHelper::DepthAndOrientationFusion.ground_distance - input_port
        AuvHelper::DepthAndOrientationFusion - instance_req
        GpsHelper::GPSFaker.position_samples - output_port
        GpsHelper::GPSFaker.state - output_port
        GpsHelper::GPSFaker - instance_req
        WallOrientationCorrection::OrientationInMap.orientation_in_map - output_port
        WallOrientationCorrection::OrientationInMap.transformer_stream_aligner_status - output_port
        WallOrientationCorrection::OrientationInMap.transformer_status - output_port
        WallOrientationCorrection::OrientationInMap.state - output_port
        WallOrientationCorrection::OrientationInMap.orientation_in_world - input_port
        WallOrientationCorrection::OrientationInMap.dynamic_transformations - input_port
        WallOrientationCorrection::OrientationInMap - instance_req
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
        AuvControl::Base.state - output_port
        AuvControl::Base.cmd_in - input_port
        AuvControl::Base.cmd_cascade - input_port
        AuvControl::Base - instance_req
        CameraBase::Task.frame - output_port
        CameraBase::Task.frame_raw - output_port
        CameraBase::Task.state - output_port
        CameraBase::Task - instance_req
        Hbridge::SensorReader.state - output_port
        Hbridge::SensorReader.can_out - output_port
        Hbridge::SensorReader.status_samples - output_port
        Hbridge::SensorReader.can_in - input_port
        Hbridge::SensorReader - instance_req
        AuvControl::OptimalHeadingController.state - output_port
        AuvControl::OptimalHeadingController.cmd_out - output_port
        AuvControl::OptimalHeadingController.cmd_in - input_port
        AuvControl::OptimalHeadingController.cmd_cascade - input_port
        AuvControl::OptimalHeadingController.orientation_samples - input_port
        AuvControl::OptimalHeadingController - instance_req
        OffshorePipelineDetector::SonarDetector.frame - output_port
        OffshorePipelineDetector::SonarDetector.state - output_port
        OffshorePipelineDetector::SonarDetector.sonar_beam - input_port
        OffshorePipelineDetector::SonarDetector - instance_req
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
        SonarBlueview::Task.frame - output_port
        SonarBlueview::Task.state - output_port
        SonarBlueview::Task - instance_req
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
        GpsHelper::WaypointNavigation.target_waypoint - output_port
        GpsHelper::WaypointNavigation.heading_offset - output_port
        GpsHelper::WaypointNavigation.distance_delta - output_port
        GpsHelper::WaypointNavigation.state - output_port
        GpsHelper::WaypointNavigation.gps_position_samples - input_port
        GpsHelper::WaypointNavigation.pose_samples - input_port
        GpsHelper::WaypointNavigation - instance_req
        AuvControl::MotionCommand2DConverter.cmd_out - output_port
        AuvControl::MotionCommand2DConverter.state - output_port
        AuvControl::MotionCommand2DConverter.cmd_in - input_port
        AuvControl::MotionCommand2DConverter - instance_req
        Sysmon::Task.ocu_markers - output_port
        Sysmon::Task.annotations - output_port
        Sysmon::Task.system_status - output_port
        Sysmon::Task.state - output_port
        Sysmon::Task.can_in_system_status - input_port
        Sysmon::Task.can_in_experiment_markers - input_port
        Sysmon::Task.in_experiment_markers - input_port
        Sysmon::Task.in_modem_substates - input_port
        Sysmon::Task - instance_req
        GpsHelper::MapToGPS.gps_position - output_port
        GpsHelper::MapToGPS.transformer_stream_aligner_status - output_port
        GpsHelper::MapToGPS.transformer_status - output_port
        GpsHelper::MapToGPS.state - output_port
        GpsHelper::MapToGPS.position_samples - input_port
        GpsHelper::MapToGPS.dynamic_transformations - input_port
        GpsHelper::MapToGPS - instance_req
        AuvControl::WaypointNavigator.cmd_out - output_port
        AuvControl::WaypointNavigator.waypoint_info - output_port
        AuvControl::WaypointNavigator.state - output_port
        AuvControl::WaypointNavigator.trajectory - input_port
        AuvControl::WaypointNavigator.pose_sample - input_port
        AuvControl::WaypointNavigator - instance_req
        Controldev::GenericTask.raw_command - output_port
        Controldev::GenericTask.state - output_port
        Controldev::GenericTask - instance_req
        Controldev::Remote.raw_command - output_port
        Controldev::Remote.state - output_port
        Controldev::Remote.canInputDevice - input_port
        Controldev::Remote - instance_req
        SonarFeatureDetector::Task.features - output_port
        SonarFeatureDetector::Task.next_target - output_port
        SonarFeatureDetector::Task.next_target_feature - output_port
        SonarFeatureDetector::Task.next_target_command - output_port
        SonarFeatureDetector::Task.state - output_port
        SonarFeatureDetector::Task.grid_maps - input_port
        SonarFeatureDetector::Task.pose_samples - input_port
        SonarFeatureDetector::Task - instance_req
        Controldev::JoystickTask.raw_command - output_port
        Controldev::JoystickTask.state - output_port
        Controldev::JoystickTask - instance_req
        RawControlCommandConverter::Position.position_command - output_port
        RawControlCommandConverter::Position.world_command - output_port
        RawControlCommandConverter::Position.state - output_port
        RawControlCommandConverter::Position.raw_command - input_port
        RawControlCommandConverter::Position.pose_samples - input_port
        RawControlCommandConverter::Position - instance_req
        AuvControl::ConstantCommandGroundAvoidance.floor_position - output_port
        AuvControl::ConstantCommandGroundAvoidance.state - output_port
        AuvControl::ConstantCommandGroundAvoidance.cmd_out - output_port
        AuvControl::ConstantCommandGroundAvoidance.altimeter - input_port
        AuvControl::ConstantCommandGroundAvoidance.depth - input_port
        AuvControl::ConstantCommandGroundAvoidance.cmd_in - input_port
        AuvControl::ConstantCommandGroundAvoidance - instance_req
        Controldev::GenericRawToMotion2D.motion_command - output_port
        Controldev::GenericRawToMotion2D.state - output_port
        Controldev::GenericRawToMotion2D.raw_command - input_port
        Controldev::GenericRawToMotion2D - instance_req
        ImagePreprocessing::StereoTask.state - output_port
        ImagePreprocessing::StereoTask.oframe_pair - output_port
        ImagePreprocessing::StereoTask.frame_left - input_port
        ImagePreprocessing::StereoTask.frame_right - input_port
        ImagePreprocessing::StereoTask - instance_req
        Controldev::JoyPadTask.raw_command - output_port
        Controldev::JoyPadTask.state - output_port
        Controldev::JoyPadTask - instance_req
        RawControlCommandConverter::Movement.motion_command - output_port
        RawControlCommandConverter::Movement.world_command - output_port
        RawControlCommandConverter::Movement.world_command_depth - output_port
        RawControlCommandConverter::Movement.aligned_velocity_command - output_port
        RawControlCommandConverter::Movement.state - output_port
        RawControlCommandConverter::Movement.raw_command - input_port
        RawControlCommandConverter::Movement.orientation_readings - input_port
        RawControlCommandConverter::Movement.ground_distance - input_port
        RawControlCommandConverter::Movement - instance_req
        Controldev::Mouse3DTask.raw_command - output_port
        Controldev::Mouse3DTask.state - output_port
        Controldev::Mouse3DTask - instance_req
        Controldev::RawWheelToMotion2D.motion_command - output_port
        Controldev::RawWheelToMotion2D.state - output_port
        Controldev::RawWheelToMotion2D.raw_command - input_port
        Controldev::RawWheelToMotion2D - instance_req
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
        Interfaces::IMU.orientation_samples - output_port
        Interfaces::IMU.calibrated_sensors - output_port
        Interfaces::IMU.state - output_port
        Interfaces::IMU - instance_req
        Controldev::SteeringWheelTask.raw_command - output_port
        Controldev::SteeringWheelTask.state - output_port
        Controldev::SteeringWheelTask - instance_req
        AuvControl::ConstantCommandGroundFollower.floor_position - output_port
        AuvControl::ConstantCommandGroundFollower.state - output_port
        AuvControl::ConstantCommandGroundFollower.cmd_out - output_port
        AuvControl::ConstantCommandGroundFollower.altimeter - input_port
        AuvControl::ConstantCommandGroundFollower.depth - input_port
        AuvControl::ConstantCommandGroundFollower.cmd_in - input_port
        AuvControl::ConstantCommandGroundFollower - instance_req
        Controldev::RawJoystickToMotion2D.motion_command - output_port
        Controldev::RawJoystickToMotion2D.state - output_port
        Controldev::RawJoystickToMotion2D.raw_command - input_port
        Controldev::RawJoystickToMotion2D - instance_req
        Interfaces::LaserRangeFinder.scans - output_port
        Interfaces::LaserRangeFinder.state - output_port
        Interfaces::LaserRangeFinder - instance_req
        Controldev::SliderboxTask.raw_command - output_port
        Controldev::SliderboxTask.state - output_port
        Controldev::SliderboxTask - instance_req
        AuvControl::ConstantCommand.cmd_out - output_port
        AuvControl::ConstantCommand.state - output_port
        AuvControl::ConstantCommand - instance_req
        ImagePreprocessing::MonoTask.state - output_port
        ImagePreprocessing::MonoTask.oframe - output_port
        ImagePreprocessing::MonoTask.frame - input_port
        ImagePreprocessing::MonoTask - instance_req
        Interfaces::ActuatorSensorReader.state - output_port
        Interfaces::ActuatorSensorReader - instance_req
        Interfaces::ActuatorCommandWriter.state - output_port
        Interfaces::ActuatorCommandWriter - instance_req
        AuvWaypointNavigator::Task.relative_position_command - output_port
        AuvWaypointNavigator::Task.current_delta - output_port
        AuvWaypointNavigator::Task.current_waypoint - output_port
        AuvWaypointNavigator::Task.queue_size - output_port
        AuvWaypointNavigator::Task.state - output_port
        AuvWaypointNavigator::Task.trajectory - input_port
        AuvWaypointNavigator::Task.pose_samples - input_port
        AuvWaypointNavigator::Task - instance_req
        RearSonarDistanceEstimator::Task.ground_distance - output_port
        RearSonarDistanceEstimator::Task.state - output_port
        RearSonarDistanceEstimator::Task.BaseScan - input_port
        RearSonarDistanceEstimator::Task.depth_samples - input_port
        RearSonarDistanceEstimator::Task - instance_req
        Taskmon::Task.stats - output_port
        Taskmon::Task.state - output_port
        Taskmon::Task - instance_req
        Dynamixel::Task.upper2lower - output_port
        Dynamixel::Task.angle - output_port
        Dynamixel::Task.lowerDynamixel2UpperDynamixel - output_port
        Dynamixel::Task.state - output_port
        Dynamixel::Task.cmd_angle - input_port
        Dynamixel::Task - instance_req
        StructuredLight::Calibration.stream_aligner_status - output_port
        StructuredLight::Calibration.state - output_port
        StructuredLight::Calibration.laser_scan - input_port
        StructuredLight::Calibration.calibration - input_port
        StructuredLight::Calibration - instance_req
        StructuredLight::Task.laser_scan - output_port
        StructuredLight::Task.candidates - output_port
        StructuredLight::Task.debug_frame - output_port
        StructuredLight::Task.state - output_port
        StructuredLight::Task.frame_pair - input_port
        StructuredLight::Task.frame - input_port
        StructuredLight::Task - instance_req
        CameraUnicap::CameraTask.frame - output_port
        CameraUnicap::CameraTask.state - output_port
        CameraUnicap::CameraTask - instance_req
        FrameDemultiplexer::Task.oframe_pair - output_port
        FrameDemultiplexer::Task.oframe - output_port
        FrameDemultiplexer::Task.state - output_port
        FrameDemultiplexer::Task.iframe - input_port
        FrameDemultiplexer::Task - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvCont::WorldXYPositionCmp - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Pipeline::Follower - instance_req
        AuvCont::WorldXYVelocityCmp - instance_req
        Pipeline::Detector - instance_req
        AuvCont::WorldPositionCmp - instance_req
        Pipeline::Detector_new - instance_req
        AuvCont::ConstantCommandGroundAvoidanceCmp - instance_req
        Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Structure::StructureReconstructionComp - instance_req
        Structure::SonarStructureServoingComp - instance_req
        Base::ControlLoop - instance_req
        Structure::Detector - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        LowLevel::Cmp - instance_req
        Structure::Alignment - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Wall::Follower - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::TrajectoryMove - instance_req
        Buoy::DetectorCmp2 - instance_req
        Hbridge::ControlSystem - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Localization::DeadReckoning - instance_req
        Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Buoy::DetectorCmp - instance_req
        AuvCont::StructureCmp - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvCont::WorldYPositionXVelocityCmp - instance_req
        Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_ - instance_req
        ConstantWorldXYVelocityCommand - instance_req
        AuvCont::MoveCmp - instance_req
        Wall::DetectorNew - instance_req
        ConsWA - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::MotionControlCmp - instance_req
        Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Buoy::ControllerNewCmp - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvCont::PositionMoveCmp - instance_req
        Localization::ParticleDetector - instance_req
        AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimplePosMove - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Buoy::DetectorNewCmp - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        Wall::Detector - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Modem::ModemCmp - instance_req
        GPSHelper::GPSWaypointsCmp - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvCont::Trajectory - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        PoseAuv::IKFOrientationEstimatorCmp - instance_req
        AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::RelPosControlCmp - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        PoseAuv::InitialOrientationEstimatorCmp - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        PoseAuv::PoseEstimatorBlindCmp - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        PoseAuv::PoseEstimatorCmp - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__ - instance_req
        blueview_cmp - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        front_camera_cmp - instance_req
        bottom_camera_cmp - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__ - instance_req
        Localization::SonarFeatureDetectorCmp - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        Localization::FixMapHack - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::PositionControlCmp - instance_req
        Buoy::FollowerCmp - instance_req
        Buoy::DetectorCmp_Base::ControllerSrv_ - instance_req
        Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__ - instance_req
        AuvControl::SimpleMove - instance_req
        Buoy::DoubleBuoyCmp - instance_req
        Localization::HoughDetector - instance_req
        AuvControl::DepthFusionCmp - instance_req
        AuvControl::JoystickCommandCmp - instance_req
        Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvCont::BuoyWallCmp - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        AuvCont::WorldXYZPositionCmp - instance_req
        Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__ - instance_req
        Base::DVLSrv - instance_req
        Dev::Bus::CAN::BusSrv - instance_req
        Base::AUVRelativeMotionControlledSystemSrv - instance_req
        Base::VelocitySrv - instance_req
        Base::LaserRangeFinderSrv - instance_req
        Base::AUVRelativeMotionControllerSrv - instance_req
        Base::PoseDeltaSrv - instance_req
        Base::AUVRelativeMotionCommandConsumerSrv - instance_req
        Base::DistanceImageProviderSrv - instance_req
        Base::RelativePoseSrv - instance_req
        Base::GlobalPoseSrv - instance_req
        Dev::Bus::CAN::BusOutSrv - instance_req
        Base::StereoPairProviderSrv - instance_req
        Base::TransformationSrv - instance_req
        Base::ImageProviderSrv - instance_req
        Dev::Bus::CAN::BusInSrv - instance_req
        Base::PoseSrv - instance_req
        Base::CalibratedIMUSensorsSrv - instance_req
        Base::OrientationWithZSrv - instance_req
        Dev::Bus::CAN::BusBaseSrv - instance_req
        Base::AUVMotionControlledSystemSrv - instance_req
        Dev::Bus::CAN - instance_req
        Base::CompensatedIMUSensorsSrv - instance_req
        Base::ZProviderSrv - instance_req
        Base::AUVMotionControllerSrv - instance_req
        Base::OrientationSrv - instance_req
        Base::AUVMotionCommandConsumerSrv - instance_req
        Base::IMUSensorsSrv - instance_req
        Base::PositionSrv - instance_req
        Base::RotationSrv - instance_req
        Base::ImageConsumerSrv - instance_req
        Base::TimestampInputSrv - instance_req
        Base::TimestamperSrv - instance_req
        Base::JointsControlledSystemSrv - instance_req
        Base::JointsControllerSrv - instance_req
        Base::JointsCommandSrv - instance_req
        Base::JointsStatusSrv - instance_req
        Dev::Sensors::KVH::DSP3000 - instance_req
        Base::JointsCommandConsumerSrv - instance_req
        Base::SonarImageProviderSrv - instance_req
        Base::ControlledSystemSrv - instance_req
        Base::ControllerSrv - instance_req
        Base::PointcloudProviderSrv - instance_req
        Base::GroundDistanceSrv - instance_req
        Base::SonarScanProviderSrv - instance_req
        Dev::Sensors::Modem - instance_req
        Dev::Sensors::DepthReaderAvalon - instance_req
        Dev::Sensors::Cameras::Vrmagic - instance_req
        Dev::Actuators::Dynamixel - instance_req
        Dev::Actuators::PTU - instance_req
        Base::MapSrv - instance_req
        Dev::Sensors::TimestamperDev - instance_req
        Base::OrientationToCorrectSrv - instance_req
        Dev::Sensors::Cameras::USB - instance_req
        Auv::StructuredLightPairSrv - instance_req
        Dev::Sensors::Cameras::Prosilica - instance_req
        Auv::SoundSourceDirectionSrv - instance_req
        Dev::Sensors::Cameras::Firewire - instance_req
        WallServoing::WallOrientationSrv - instance_req
        Dev::Sensors::XsensAHRS - instance_req
        Auv::ModemConnectionSrv - instance_req
        Dev::Sensors::Hokuyo - instance_req
        Dev::Sensors::GPS - instance_req
        Syskit::ComBus - instance_req
        Syskit::Device - instance_req
        Dev::Bus::CAN::ClientInSrv - instance_req
        Dev::Actuators::Lights - instance_req
        Dev::Bus::CAN::ClientOutSrv - instance_req
        Dev::Sensors::Hbridge - instance_req
        Dev::Bus::CAN::ClientSrv - instance_req
        Dev::Actuators::Hbridge - instance_req
        Dev::Controldev::Mouse3D - instance_req
        Dev::Controldev::Joystick - instance_req
        Base::WorldZRollPitchYawSrv - instance_req
        Dev::Micron - instance_req
        Base::WorldXYVelocityControllerSrv - instance_req
        Dev::Echosounder - instance_req
        Dev::Profiling - instance_req
        Base::RawCommandControlledSystemSrv - instance_req
        Base::WorldYPositionXVelocityControllerSrv - instance_req
        Base::WorldXYZPositionControllerSrv - instance_req
        Localization::HoughSrv - instance_req
        Base::RawCommandControllerSrv - instance_req
        Dev::ASVModem - instance_req
        Base::WorldXYPositionControllerSrv - instance_req
        Base::WorldXYZRollPitchYawControllerSrv - instance_req
        Base::WorldXYZRollPitchYawControlledSystemSrv - instance_req
        Dev::Sensors::BlueView - instance_req
        Dev::SystemStatus - instance_req
        Base::RawCommandCommandConsumerSrv - instance_req
        Dev::Sensors::Battery - instance_req
  )
  (:init

        (composition  root)
        (task  SonarWallHough::Task)
        (task  Gps::MB500Task)
        (task  Gps::GPSDTask)
        (task  Gps::BaseTask)
        (task  Canbus::InterfaceTask)
        (task  VideoStreamerVlc::Streamer{1})
        (task  Canbus::Task)
        (task  Transformer::Task)
        (task  VideoStreamerVlc::Capturer)
        (task  Buoy::ServoingOnWall)
        (task  Buoy::Detector2)
        (task  VideoStreamerVlc::Streamer)
        (task  XsensImu::Task)
        (task  PipelineInspection::LaserSimulation)
        (task  Buoy::Detector)
        (task  PipelineInspection::Inspection)
        (task  Logger::Logger)
        (task  PipelineInspection::ColorFilter)
        (task  Syskit::ROS::Node)
        (task  Buoy::Survey)
        (task  FogKvh::Dsp3000Task)
        (task  OrientationEstimator::IKF)
        (task  UwParticleLocalization::FastFusion)
        (task  UwParticleLocalization::MotionModel)
        (task  LowLevelDriver::LowLevelTask)
        (task  OrientationEstimator::BaseEstimator)
        (task  StructureReconstruction::Task)
        (task  UwParticleLocalization::OrientationCorrection)
        (task  LineScanner::Task)
        (task  Syskit::RubyTaskContext)
        (task  AuvRelPosController::Task)
        (task  UwParticleLocalization::Task)
        (task  ModemCan::Task)
        (task  DepthReader::Task)
        (task  SonarStructureServoing::Task)
        (task  RTT::TaskContext)
        (task  Lights::Lights)
        (task  WallServoing::DualSonarServoing)
        (task  AvalonControl::MotionFeedbackTask)
        (task  BatteryWatcher::Task)
        (task  ImagePreprocessing::DepthImage2Pointcloud)
        (task  AuvControl::AccelerationController)
        (task  SonarTritech::Micron)
        (task  WallServoing::SingleSonarServoing)
        (task  SonarFeatureEstimator::Task)
        (task  AvalonControl::TrajectoryFollower)
        (task  WallServoing::SonarServoing)
        (task  ImagePreprocessing::BaseTask)
        (task  PoseEstimation::BaseTask)
        (task  AuvControl::AlignedToBody)
        (task  PoseEstimation::UWPoseEstimator)
        (task  PoseEstimation::HighDelayPoseEstimator)
        (task  PoseEstimation::VehiclePoseEstimator)
        (task  WallServoing::WallServoing)
        (task  CameraProsilica::Task)
        (task  WallServoing::WallDetector)
        (task  SonarTritech::Echosounder)
        (task  SonarTritech::Profiling)
        (task  Interfaces::Servo)
        (task  AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_)
        (task  AuvControl::WorldToAligned)
        (task  CameraBase::Preprocess)
        (task  AuvControl::PIDController)
        (task  AvalonControl::RelFakeWriter)
        (task  AvalonControl::FakeWriter)
        (task  AvalonControl::MotionControlTask)
        (task  AuvControl::BasePIDController)
        (task  Modemdriver::ModemCanbus)
        (task  Hbridge::CommandWriter)
        (task  Modemdriver::ModemSerial)
        (task  WallOrientationCorrection::Task)
        (task  AvalonControl::PositionControlTask)
        (task  HsvMosaicing::Task)
        (task  Modemdriver::Modem)
        (task  AuvHelper::DepthAndOrientationFusion)
        (task  GpsHelper::GPSFaker)
        (task  WallOrientationCorrection::OrientationInMap)
        (task  OffshorePipelineDetector::Task)
        (task  AuvControl::Base)
        (task  CameraBase::Task)
        (task  Hbridge::SensorReader)
        (task  AuvControl::OptimalHeadingController)
        (task  OffshorePipelineDetector::SonarDetector)
        (task  StructureServoing::Alignment)
        (task  SonarBlueview::Task)
        (task  StructureServoing::Task)
        (task  GpsHelper::WaypointNavigation)
        (task  AuvControl::MotionCommand2DConverter)
        (task  Sysmon::Task)
        (task  GpsHelper::MapToGPS)
        (task  AuvControl::WaypointNavigator)
        (task  Controldev::GenericTask)
        (task  Controldev::Remote)
        (task  SonarFeatureDetector::Task)
        (task  Controldev::JoystickTask)
        (task  RawControlCommandConverter::Position)
        (task  AuvControl::ConstantCommandGroundAvoidance)
        (task  Controldev::GenericRawToMotion2D)
        (task  ImagePreprocessing::StereoTask)
        (task  Controldev::JoyPadTask)
        (task  RawControlCommandConverter::Movement)
        (task  Controldev::Mouse3DTask)
        (task  Controldev::RawWheelToMotion2D)
        (task  ImagePreprocessing::HSVSegmentationAndBlur)
        (task  Interfaces::IMU)
        (task  Controldev::SteeringWheelTask)
        (task  AuvControl::ConstantCommandGroundFollower)
        (task  Controldev::RawJoystickToMotion2D)
        (task  Interfaces::LaserRangeFinder)
        (task  Controldev::SliderboxTask)
        (task  AuvControl::ConstantCommand)
        (task  ImagePreprocessing::MonoTask)
        (task  Interfaces::ActuatorSensorReader)
        (task  Interfaces::ActuatorCommandWriter)
        (task  AuvWaypointNavigator::Task)
        (task  RearSonarDistanceEstimator::Task)
        (task  Taskmon::Task)
        (task  Dynamixel::Task)
        (task  StructuredLight::Calibration)
        (task  StructuredLight::Task)
        (task  CameraUnicap::CameraTask)
        (task  FrameDemultiplexer::Task)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvCont::WorldXYPositionCmp)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Pipeline::Follower)
        (composition  AuvCont::WorldXYVelocityCmp)
        (composition  Pipeline::Detector)
        (composition  AuvCont::WorldPositionCmp)
        (composition  Pipeline::Detector_new)
        (composition  AuvCont::ConstantCommandGroundAvoidanceCmp)
        (composition  Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Structure::StructureReconstructionComp)
        (composition  Structure::SonarStructureServoingComp)
        (composition  Base::ControlLoop)
        (composition  Structure::Detector)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  LowLevel::Cmp)
        (composition  Structure::Alignment)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Wall::Follower)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::TrajectoryMove)
        (composition  Buoy::DetectorCmp2)
        (composition  Hbridge::ControlSystem)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Localization::DeadReckoning)
        (composition  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Buoy::DetectorCmp)
        (composition  AuvCont::StructureCmp)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvCont::WorldYPositionXVelocityCmp)
        (composition  Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_)
        (composition  ConstantWorldXYVelocityCommand)
        (composition  AuvCont::MoveCmp)
        (composition  Wall::DetectorNew)
        (composition  ConsWA)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::MotionControlCmp)
        (composition  Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Buoy::ControllerNewCmp)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvCont::PositionMoveCmp)
        (composition  Localization::ParticleDetector)
        (composition  AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimplePosMove)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Buoy::DetectorNewCmp)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  Wall::Detector)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Modem::ModemCmp)
        (composition  GPSHelper::GPSWaypointsCmp)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvCont::Trajectory)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  PoseAuv::IKFOrientationEstimatorCmp)
        (composition  AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::RelPosControlCmp)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  PoseAuv::InitialOrientationEstimatorCmp)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  PoseAuv::PoseEstimatorBlindCmp)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  PoseAuv::PoseEstimatorCmp)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
        (composition  blueview_cmp)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  front_camera_cmp)
        (composition  bottom_camera_cmp)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
        (composition  Localization::SonarFeatureDetectorCmp)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  Localization::FixMapHack)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::PositionControlCmp)
        (composition  Buoy::FollowerCmp)
        (composition  Buoy::DetectorCmp_Base::ControllerSrv_)
        (composition  Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__)
        (composition  AuvControl::SimpleMove)
        (composition  Buoy::DoubleBuoyCmp)
        (composition  Localization::HoughDetector)
        (composition  AuvControl::DepthFusionCmp)
        (composition  AuvControl::JoystickCommandCmp)
        (composition  Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvCont::BuoyWallCmp)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (composition  AuvCont::WorldXYZPositionCmp)
        (composition  Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__)
        (data-service  Base::DVLSrv)
        (data-service  Dev::Bus::CAN::BusSrv)
        (data-service  Base::AUVRelativeMotionControlledSystemSrv)
        (data-service  Base::VelocitySrv)
        (data-service  Base::LaserRangeFinderSrv)
        (data-service  Base::AUVRelativeMotionControllerSrv)
        (data-service  Base::PoseDeltaSrv)
        (data-service  Base::AUVRelativeMotionCommandConsumerSrv)
        (data-service  Base::DistanceImageProviderSrv)
        (data-service  Base::RelativePoseSrv)
        (data-service  Base::GlobalPoseSrv)
        (data-service  Dev::Bus::CAN::BusOutSrv)
        (data-service  Base::StereoPairProviderSrv)
        (data-service  Base::TransformationSrv)
        (data-service  Base::ImageProviderSrv)
        (data-service  Dev::Bus::CAN::BusInSrv)
        (data-service  Base::PoseSrv)
        (data-service  Base::CalibratedIMUSensorsSrv)
        (data-service  Base::OrientationWithZSrv)
        (data-service  Dev::Bus::CAN::BusBaseSrv)
        (data-service  Base::AUVMotionControlledSystemSrv)
        (data-service  Dev::Bus::CAN)
        (data-service  Base::CompensatedIMUSensorsSrv)
        (data-service  Base::ZProviderSrv)
        (data-service  Base::AUVMotionControllerSrv)
        (data-service  Base::OrientationSrv)
        (data-service  Base::AUVMotionCommandConsumerSrv)
        (data-service  Base::IMUSensorsSrv)
        (data-service  Base::PositionSrv)
        (data-service  Base::RotationSrv)
        (data-service  Base::ImageConsumerSrv)
        (data-service  Base::TimestampInputSrv)
        (data-service  Base::TimestamperSrv)
        (data-service  Base::JointsControlledSystemSrv)
        (data-service  Base::JointsControllerSrv)
        (data-service  Base::JointsCommandSrv)
        (data-service  Base::JointsStatusSrv)
        (data-service  Dev::Sensors::KVH::DSP3000)
        (data-service  Base::JointsCommandConsumerSrv)
        (data-service  Base::SonarImageProviderSrv)
        (data-service  Base::ControlledSystemSrv)
        (data-service  Base::ControllerSrv)
        (data-service  Base::PointcloudProviderSrv)
        (data-service  Base::GroundDistanceSrv)
        (data-service  Base::SonarScanProviderSrv)
        (data-service  Dev::Sensors::Modem)
        (data-service  Dev::Sensors::DepthReaderAvalon)
        (data-service  Dev::Sensors::Cameras::Vrmagic)
        (data-service  Dev::Actuators::Dynamixel)
        (data-service  Dev::Actuators::PTU)
        (data-service  Base::MapSrv)
        (data-service  Dev::Sensors::TimestamperDev)
        (data-service  Base::OrientationToCorrectSrv)
        (data-service  Dev::Sensors::Cameras::USB)
        (data-service  Auv::StructuredLightPairSrv)
        (data-service  Dev::Sensors::Cameras::Prosilica)
        (data-service  Auv::SoundSourceDirectionSrv)
        (data-service  Dev::Sensors::Cameras::Firewire)
        (data-service  WallServoing::WallOrientationSrv)
        (data-service  Dev::Sensors::XsensAHRS)
        (data-service  Auv::ModemConnectionSrv)
        (data-service  Dev::Sensors::Hokuyo)
        (data-service  Dev::Sensors::GPS)
        (data-service  Syskit::ComBus)
        (data-service  Syskit::Device)
        (data-service  Dev::Bus::CAN::ClientInSrv)
        (data-service  Dev::Actuators::Lights)
        (data-service  Dev::Bus::CAN::ClientOutSrv)
        (data-service  Dev::Sensors::Hbridge)
        (data-service  Dev::Bus::CAN::ClientSrv)
        (data-service  Dev::Actuators::Hbridge)
        (data-service  Dev::Controldev::Mouse3D)
        (data-service  Dev::Controldev::Joystick)
        (data-service  Base::WorldZRollPitchYawSrv)
        (data-service  Dev::Micron)
        (data-service  Base::WorldXYVelocityControllerSrv)
        (data-service  Dev::Echosounder)
        (data-service  Dev::Profiling)
        (data-service  Base::RawCommandControlledSystemSrv)
        (data-service  Base::WorldYPositionXVelocityControllerSrv)
        (data-service  Base::WorldXYZPositionControllerSrv)
        (data-service  Localization::HoughSrv)
        (data-service  Base::RawCommandControllerSrv)
        (data-service  Dev::ASVModem)
        (data-service  Base::WorldXYPositionControllerSrv)
        (data-service  Base::WorldXYZRollPitchYawControllerSrv)
        (data-service  Base::WorldXYZRollPitchYawControlledSystemSrv)
        (data-service  Dev::Sensors::BlueView)
        (data-service  Dev::SystemStatus)
        (data-service  Base::RawCommandCommandConsumerSrv)
        (data-service  Dev::Sensors::Battery)

            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Detector)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Detector)
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
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Pipeline::Detector)
            (depends Pipeline::Follower  Pipeline::Detector)
            (depends Pipeline::Follower  Base::ControlledSystemSrv)
            (depends Pipeline::Follower  Pipeline::Detector)
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
            (depends Pipeline::Detector  OffshorePipelineDetector::Task)
            (depends Pipeline::Detector  Base::ImageProviderSrv)
            (depends Pipeline::Detector  Base::OrientationWithZSrv)
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
            (depends Pipeline::Detector_new  OffshorePipelineDetector::Task)
            (depends Pipeline::Detector_new  Base::ImageProviderSrv)
            (depends Pipeline::Detector_new  Base::OrientationWithZSrv)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  AuvControl::ConstantCommandGroundAvoidance)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  Base::GroundDistanceSrv)
            (depends AuvCont::ConstantCommandGroundAvoidanceCmp  Base::ZProviderSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Structure::StructureReconstructionComp  StructureReconstruction::Task)
            (depends Structure::StructureReconstructionComp  Base::ImageProviderSrv)
            (depends Structure::StructureReconstructionComp  Base::ImageProviderSrv)
            (depends Structure::SonarStructureServoingComp  SonarStructureServoing::Task)
            (depends Structure::SonarStructureServoingComp  Base::SonarScanProviderSrv)
            (depends Structure::SonarStructureServoingComp  SonarFeatureEstimator::Task)
            (depends Structure::SonarStructureServoingComp  Base::PoseSrv)
            (depends Base::ControlLoop  Base::ControllerSrv)
            (depends Base::ControlLoop  Base::ControlledSystemSrv)
            (depends Structure::Detector  StructureServoing::Task)
            (depends Structure::Detector  HsvMosaicing::Task)
            (depends Structure::Detector  ImagePreprocessing::HSVSegmentationAndBlur)
            (depends Structure::Detector  Base::ImageProviderSrv)
            (depends Structure::Detector  Base::OrientationWithZSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends LowLevel::Cmp  LowLevelDriver::LowLevelTask)
            (depends LowLevel::Cmp  Base::OrientationWithZSrv)
            (depends Structure::Alignment  StructureServoing::Alignment)
            (depends Structure::Alignment  HsvMosaicing::Task)
            (depends Structure::Alignment  ImagePreprocessing::HSVSegmentationAndBlur)
            (depends Structure::Alignment  Base::ImageProviderSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvRelPosController::Task)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AvalonControl::MotionControlTask)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends Wall::Follower  Wall::Detector)
            (depends Wall::Follower  Base::ControlledSystemSrv)
            (depends Wall::Follower  Wall::Detector)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::TrajectoryMove  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove  Base::ControlledSystemSrv)
            (depends AuvControl::TrajectoryMove  AvalonControl::TrajectoryFollower)
            (depends AuvControl::TrajectoryMove  Base::PoseSrv)
            (depends Buoy::DetectorCmp2  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp2  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp2  Buoy::Detector2)
            (depends Hbridge::ControlSystem  Dev::Sensors::Hbridge)
            (depends Hbridge::ControlSystem  Dev::Actuators::Hbridge)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends Localization::DeadReckoning  UwParticleLocalization::MotionModel)
            (depends Localization::DeadReckoning  Base::OrientationWithZSrv)
            (depends Localization::DeadReckoning  Base::JointsStatusSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AvalonControl::PositionControlTask)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Buoy::DetectorCmp  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp  Buoy::Detector)
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
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Detector)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
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
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Buoy::Detector)
            (depends ConstantWorldXYVelocityCommand  AuvControl::ConstantCommand)
            (depends ConstantWorldXYVelocityCommand  ConsWA)
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
            (depends Wall::DetectorNew  WallServoing::SingleSonarServoing)
            (depends Wall::DetectorNew  SonarTritech::Micron)
            (depends Wall::DetectorNew  SonarFeatureEstimator::Task)
            (depends Wall::DetectorNew  Base::OrientationWithZSrv)
            (depends Wall::DetectorNew  Base::VelocitySrv)
            (depends ConsWA  AuvControl::ConstantCommand)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::MotionControlCmp  AvalonControl::MotionControlTask)
            (depends AuvControl::MotionControlCmp  Base::ControlledSystemSrv)
            (depends AuvControl::MotionControlCmp  Base::OrientationWithZSrv)
            (depends AuvControl::MotionControlCmp  Base::GroundDistanceSrv)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Base::ControlLoop__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::ControllerNewCmp  Buoy::ServoingOnWall)
            (depends Buoy::ControllerNewCmp  WallServoing::WallOrientationSrv)
            (depends Buoy::ControllerNewCmp  Base::OrientationSrv)
            (depends Buoy::ControllerNewCmp  Buoy::DetectorNewCmp)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::DetectorCmp)
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
            (depends Localization::ParticleDetector  UwParticleLocalization::Task)
            (depends Localization::ParticleDetector  Base::SonarScanProviderSrv)
            (depends Localization::ParticleDetector  SonarFeatureEstimator::Task)
            (depends Localization::ParticleDetector  Base::OrientationWithZSrv)
            (depends Localization::ParticleDetector  Base::JointsStatusSrv)
            (depends Localization::ParticleDetector  SonarFeatureDetector::Task)
            (depends Localization::ParticleDetector  Localization::HoughSrv)
            (depends Localization::ParticleDetector  Base::GroundDistanceSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::SimplePosMove  AvalonControl::RelFakeWriter)
            (depends AuvControl::SimplePosMove  Base::ControlledSystemSrv)
            (depends AuvControl::SimplePosMove  Base::PoseSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Buoy::DetectorNewCmp  Buoy::Detector)
            (depends Buoy::DetectorNewCmp  Base::ImageProviderSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Detector)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Wall::Detector  WallServoing::SingleSonarServoing)
            (depends Wall::Detector  SonarTritech::Micron)
            (depends Wall::Detector  SonarFeatureEstimator::Task)
            (depends Wall::Detector  Base::OrientationWithZSrv)
            (depends Wall::Detector  Base::VelocitySrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends Modem::ModemCmp  Dev::ASVModem)
            (depends GPSHelper::GPSWaypointsCmp  GpsHelper::WaypointNavigation)
            (depends GPSHelper::GPSWaypointsCmp  Base::PositionSrv)
            (depends GPSHelper::GPSWaypointsCmp  Base::PoseSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
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
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Detector)
            (depends PoseAuv::IKFOrientationEstimatorCmp  OrientationEstimator::BaseEstimator)
            (depends PoseAuv::IKFOrientationEstimatorCmp  WallOrientationCorrection::OrientationInMap)
            (depends PoseAuv::IKFOrientationEstimatorCmp  XsensImu::Task)
            (depends PoseAuv::IKFOrientationEstimatorCmp  FogKvh::Dsp3000Task)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::RelPosControlCmp  AuvRelPosController::Task)
            (depends AuvControl::RelPosControlCmp  Base::ControlledSystemSrv)
            (depends AuvControl::RelPosControlCmp  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends PoseAuv::InitialOrientationEstimatorCmp  WallOrientationCorrection::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  OrientationEstimator::BaseEstimator)
            (depends PoseAuv::InitialOrientationEstimatorCmp  XsensImu::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  FogKvh::Dsp3000Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  Base::SonarScanProviderSrv)
            (depends PoseAuv::InitialOrientationEstimatorCmp  SonarFeatureEstimator::Task)
            (depends PoseAuv::InitialOrientationEstimatorCmp  PoseAuv::IKFOrientationEstimatorCmp)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  PoseEstimation::UWPoseEstimator)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::OrientationSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::VelocitySrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::ZProviderSrv)
            (depends PoseAuv::PoseEstimatorBlindCmp  Base::DVLSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::PositionSrv)
            (depends PoseAuv::PoseEstimatorCmp  PoseEstimation::UWPoseEstimator)
            (depends PoseAuv::PoseEstimatorCmp  Base::OrientationSrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::VelocitySrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::ZProviderSrv)
            (depends PoseAuv::PoseEstimatorCmp  Base::DVLSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  _Base::AUVRelativeMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControllerSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Detector)
            (depends blueview_cmp  VideoStreamerVlc::Streamer)
            (depends blueview_cmp  sonar)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends front_camera_cmp  VideoStreamerVlc::Streamer)
            (depends front_camera_cmp  driver)
            (depends bottom_camera_cmp  VideoStreamerVlc::Streamer)
            (depends bottom_camera_cmp  driver)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  _Base::AUVMotionControlledSystemSrv,Base::ControllerSrv_)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::PoseSrv)
            (depends Localization::SonarFeatureDetectorCmp  SonarFeatureDetector::Task)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends Localization::FixMapHack  SonarFeatureDetector::Task)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::JointsControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::PoseSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControllerSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::PositionControlCmp  AvalonControl::PositionControlTask)
            (depends AuvControl::PositionControlCmp  Base::ControlledSystemSrv)
            (depends AuvControl::PositionControlCmp  Base::PoseSrv)
            (depends Buoy::FollowerCmp  Buoy::DetectorCmp_Base::ControllerSrv_)
            (depends Buoy::FollowerCmp  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp  Buoy::DetectorCmp)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Base::ImageProviderSrv)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Base::OrientationWithZSrv)
            (depends Buoy::DetectorCmp_Base::ControllerSrv_  Buoy::Detector)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Detector)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  _Base::ControllerSrv,Base::JointsControlledSystemSrv_)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove__controller.is_a__Base::JointsControlledSystemSrv__  Base::OrientationWithZSrv)
            (depends AuvControl::SimpleMove  AvalonControl::FakeWriter)
            (depends AuvControl::SimpleMove  Base::ControlledSystemSrv)
            (depends AuvControl::SimpleMove  Base::OrientationWithZSrv)
            (depends Buoy::DoubleBuoyCmp  Base::MapSrv)
            (depends Buoy::DoubleBuoyCmp  Buoy::DetectorCmp)
            (depends Buoy::DoubleBuoyCmp  Buoy::DetectorCmp2)
            (depends Localization::HoughDetector  SonarWallHough::Task)
            (depends Localization::HoughDetector  Base::SonarScanProviderSrv)
            (depends Localization::HoughDetector  Base::OrientationSrv)
            (depends Localization::HoughDetector  Base::DVLSrv)
            (depends Localization::HoughDetector  UwParticleLocalization::OrientationCorrection)
            (depends AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (depends AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (depends AuvControl::DepthFusionCmp  AuvHelper::DepthAndOrientationFusion)
            (depends AuvControl::JoystickCommandCmp  Base::RawCommandControllerSrv)
            (depends AuvControl::JoystickCommandCmp  Base::OrientationWithZSrv)
            (depends AuvControl::JoystickCommandCmp  RawControlCommandConverter::Movement)
            (depends AuvControl::JoystickCommandCmp  Base::GroundDistanceSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Wall::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Wall::Detector)
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
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControllerSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::DetectorCmp)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Buoy::FollowerCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Buoy::DetectorCmp)
            (depends AuvCont::WorldXYZPositionCmp  Base::JointsControlledSystemSrv)
            (depends AuvCont::WorldXYZPositionCmp  Base::PoseSrv)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::WorldToAligned)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::OptimalHeadingController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::PIDController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::AccelerationController)
            (depends AuvCont::WorldXYZPositionCmp  AuvControl::AlignedToBody)
            (depends AuvCont::WorldXYZPositionCmp  Base::WorldXYZPositionControllerSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  _Base::ControllerSrv,Base::RawCommandControlledSystemSrv_)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (depends Pipeline::Follower__controller.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Detector)
            
; Begin fullfillments 
            (fullfills Gps::MB500Task  Dev::Sensors::GPS)
            (fullfills Gps::MB500Task  Base::PositionSrv)
            (fullfills Gps::MB500Task  Syskit::Device)
            (fullfills Gps::GPSDTask  Dev::Sensors::GPS)
            (fullfills Gps::GPSDTask  Base::PositionSrv)
            (fullfills Gps::GPSDTask  Syskit::Device)
            (fullfills Gps::BaseTask  Dev::Sensors::GPS)
            (fullfills Gps::BaseTask  Base::PositionSrv)
            (fullfills Gps::BaseTask  Syskit::Device)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer{1}  Base::ImageConsumerSrv)
            (fullfills Canbus::Task  Dev::Bus::CAN)
            (fullfills Canbus::Task  Syskit::ComBus)
            (fullfills Canbus::Task  Syskit::Device)
            (fullfills Canbus::Task  Dev::Bus::CAN::BusInSrv)
            (fullfills Canbus::Task  Dev::Bus::CAN::BusBaseSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills VideoStreamerVlc::Streamer  Base::ImageConsumerSrv)
            (fullfills XsensImu::Task  Dev::Sensors::XsensAHRS)
            (fullfills XsensImu::Task  Base::CalibratedIMUSensorsSrv)
            (fullfills XsensImu::Task  Base::IMUSensorsSrv)
            (fullfills XsensImu::Task  Base::OrientationSrv)
            (fullfills XsensImu::Task  Syskit::Device)
            (fullfills XsensImu::Task  Base::TimestampInputSrv)
            (fullfills FogKvh::Dsp3000Task  Dev::Sensors::KVH::DSP3000)
            (fullfills FogKvh::Dsp3000Task  Base::RotationSrv)
            (fullfills FogKvh::Dsp3000Task  Syskit::Device)
            (fullfills AuvRelPosController::Task  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvRelPosController::Task  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvRelPosController::Task  Base::ControlledSystemSrv)
            (fullfills AuvRelPosController::Task  Base::AUVMotionControllerSrv)
            (fullfills AuvRelPosController::Task  Base::ControllerSrv)
            (fullfills ModemCan::Task  Dev::Sensors::Modem)
            (fullfills ModemCan::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills ModemCan::Task  Syskit::Device)
            (fullfills DepthReader::Task  Dev::Sensors::DepthReaderAvalon)
            (fullfills DepthReader::Task  Dev::Bus::CAN::ClientOutSrv)
            (fullfills DepthReader::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills DepthReader::Task  Base::ZProviderSrv)
            (fullfills DepthReader::Task  Syskit::Device)
            (fullfills Lights::Lights  Dev::Actuators::Lights)
            (fullfills Lights::Lights  Dev::Bus::CAN::ClientInSrv)
            (fullfills Lights::Lights  Syskit::Device)
            (fullfills BatteryWatcher::Task  Dev::Sensors::Battery)
            (fullfills BatteryWatcher::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills BatteryWatcher::Task  Syskit::Device)
            (fullfills SonarTritech::Micron  Dev::Micron)
            (fullfills SonarTritech::Micron  Base::GroundDistanceSrv)
            (fullfills SonarTritech::Micron  Base::SonarScanProviderSrv)
            (fullfills SonarTritech::Micron  Syskit::Device)
            (fullfills AvalonControl::TrajectoryFollower  Base::AUVRelativeMotionControllerSrv)
            (fullfills AvalonControl::TrajectoryFollower  Base::ControllerSrv)
            (fullfills AvalonControl::TrajectoryFollower  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills CameraProsilica::Task  Dev::Sensors::Cameras::Prosilica)
            (fullfills CameraProsilica::Task  Base::ImageProviderSrv)
            (fullfills CameraProsilica::Task  Syskit::Device)
            (fullfills SonarTritech::Echosounder  Dev::Echosounder)
            (fullfills SonarTritech::Echosounder  Base::GroundDistanceSrv)
            (fullfills SonarTritech::Echosounder  Syskit::Device)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::AUVMotionControllerSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::ControllerSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::AUVMotionControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::AUVMotionCommandConsumerSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::ControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::JointsControllerSrv)
            (fullfills AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_  Base::ControllerSrv)
            (fullfills AvalonControl::RelFakeWriter  Base::AUVRelativeMotionControllerSrv)
            (fullfills AvalonControl::RelFakeWriter  Base::ControllerSrv)
            (fullfills AvalonControl::FakeWriter  Base::AUVMotionControllerSrv)
            (fullfills AvalonControl::FakeWriter  Base::ControllerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::AUVMotionControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask  Base::AUVMotionCommandConsumerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::ControlledSystemSrv)
            (fullfills AvalonControl::MotionControlTask  Base::JointsControllerSrv)
            (fullfills AvalonControl::MotionControlTask  Base::ControllerSrv)
            (fullfills Modemdriver::ModemCanbus  Dev::ASVModem)
            (fullfills Modemdriver::ModemCanbus  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Modemdriver::ModemCanbus  Syskit::Device)
            (fullfills Hbridge::CommandWriter  Dev::Actuators::Hbridge)
            (fullfills Hbridge::CommandWriter  Syskit::Device)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientSrv)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Hbridge::CommandWriter  Dev::Bus::CAN::ClientInSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AvalonControl::PositionControlTask  Base::ControlledSystemSrv)
            (fullfills AvalonControl::PositionControlTask  Base::AUVMotionControllerSrv)
            (fullfills AvalonControl::PositionControlTask  Base::ControllerSrv)
            (fullfills Hbridge::SensorReader  Dev::Sensors::Hbridge)
            (fullfills Hbridge::SensorReader  Syskit::Device)
            (fullfills Hbridge::SensorReader  Base::JointsStatusSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientOutSrv)
            (fullfills Hbridge::SensorReader  Dev::Bus::CAN::ClientInSrv)
            (fullfills SonarBlueview::Task  Dev::Sensors::BlueView)
            (fullfills SonarBlueview::Task  Base::ImageProviderSrv)
            (fullfills SonarBlueview::Task  Syskit::Device)
            (fullfills Sysmon::Task  Dev::SystemStatus)
            (fullfills Sysmon::Task  Dev::Bus::CAN::ClientInSrv)
            (fullfills Sysmon::Task  Syskit::Device)
            (fullfills Controldev::Remote  Dev::Bus::CAN::ClientInSrv)
            (fullfills Controldev::Remote  Dev::Controldev::Joystick)
            (fullfills Controldev::Remote  Base::RawCommandControllerSrv)
            (fullfills Controldev::Remote  Base::ControllerSrv)
            (fullfills Controldev::Remote  Syskit::Device)
            (fullfills Controldev::JoystickTask  Dev::Controldev::Joystick)
            (fullfills Controldev::JoystickTask  Base::RawCommandControllerSrv)
            (fullfills Controldev::JoystickTask  Base::ControllerSrv)
            (fullfills Controldev::JoystickTask  Syskit::Device)
            (fullfills Controldev::Mouse3DTask  Dev::Controldev::Mouse3D)
            (fullfills Controldev::Mouse3DTask  Base::RawCommandControllerSrv)
            (fullfills Controldev::Mouse3DTask  Base::ControllerSrv)
            (fullfills Controldev::Mouse3DTask  Syskit::Device)
            (fullfills AuvControl::ConstantCommand  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills Dynamixel::Task  Dev::Actuators::Dynamixel)
            (fullfills Dynamixel::Task  Syskit::Device)
            (fullfills Dynamixel::Task  Base::TransformationSrv)
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
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvCont::WorldXYPositionCmp  AuvCont::WorldXYPositionCmp)
            (fullfills AuvCont::WorldXYPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYPositionCmp  Base::JointsCommandSrv)
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
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills Pipeline::Follower  Pipeline::Follower)
            (fullfills Pipeline::Follower  Base::ControlLoop)
            (fullfills Pipeline::Follower  Base::ControlLoop)
            (fullfills AuvCont::WorldXYVelocityCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::WorldXYVelocityCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYVelocityCmp  Base::JointsCommandSrv)
            (fullfills Pipeline::Detector  Pipeline::Detector)
            (fullfills Pipeline::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills Pipeline::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills Pipeline::Detector  Base::ControllerSrv)
            (fullfills AuvCont::WorldPositionCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::WorldPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldPositionCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills Pipeline::Detector_new  Pipeline::Detector_new)
            (fullfills Pipeline::Detector_new  Base::WorldXYPositionControllerSrv)
            (fullfills Pipeline::Detector_new  Base::WorldXYPositionControllerSrv)
            (fullfills AuvCont::ConstantCommandGroundAvoidanceCmp  AuvCont::ConstantCommandGroundAvoidanceCmp)
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
            (fullfills Structure::StructureReconstructionComp  Structure::StructureReconstructionComp)
            (fullfills Structure::SonarStructureServoingComp  Structure::SonarStructureServoingComp)
            (fullfills Structure::SonarStructureServoingComp  Base::AUVRelativeMotionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::WorldXYPositionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::AUVRelativeMotionControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::ControllerSrv)
            (fullfills Structure::SonarStructureServoingComp  Base::WorldXYPositionControllerSrv)
            (fullfills Base::ControlLoop  Base::ControlLoop)
            (fullfills Structure::Detector  Structure::Detector)
            (fullfills Structure::Detector  Base::WorldXYVelocityControllerSrv)
            (fullfills Structure::Detector  Base::WorldXYVelocityControllerSrv)
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
            (fullfills LowLevel::Cmp  LowLevel::Cmp)
            (fullfills Structure::Alignment  Structure::Alignment)
            (fullfills Structure::Alignment  Base::WorldXYVelocityControllerSrv)
            (fullfills Structure::Alignment  Base::WorldXYVelocityControllerSrv)
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
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv,Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills Wall::Follower  Wall::Follower)
            (fullfills Wall::Follower  Base::ControlLoop)
            (fullfills Wall::Follower  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove  AuvControl::TrajectoryMove)
            (fullfills AuvControl::TrajectoryMove  Base::ControlLoop)
            (fullfills AuvControl::TrajectoryMove  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp2  Buoy::DetectorCmp2)
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
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::DeadReckoning  Localization::DeadReckoning)
            (fullfills Localization::DeadReckoning  Base::VelocitySrv)
            (fullfills Localization::DeadReckoning  Base::VelocitySrv)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv,Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp  Buoy::DetectorCmp)
            (fullfills AuvCont::StructureCmp  AuvCont::StructureCmp)
            (fullfills AuvCont::StructureCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::StructureCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::StructureCmp  Base::JointsCommandSrv)
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
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Base::ControlLoop__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  AuvCont::WorldYPositionXVelocityCmp)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldYPositionXVelocityCmp  Base::JointsCommandSrv)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Buoy::DetectorCmp)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::AUVRelativeMotionControllerSrv)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ControllerSrv)
            (fullfills Buoy::DetectorCmp_Base::AUVRelativeMotionControllerSrv_  Base::ControllerSrv)
            (fullfills ConstantWorldXYVelocityCommand  ConstantWorldXYVelocityCommand)
            (fullfills ConstantWorldXYVelocityCommand  Base::WorldXYVelocityControllerSrv)
            (fullfills ConstantWorldXYVelocityCommand  Base::WorldXYVelocityControllerSrv)
            (fullfills AuvCont::MoveCmp  AuvCont::MoveCmp)
            (fullfills AuvCont::MoveCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::MoveCmp  AuvCont::WorldXYVelocityCmp)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::MoveCmp  Base::JointsCommandSrv)
            (fullfills Wall::DetectorNew  Wall::DetectorNew)
            (fullfills Wall::DetectorNew  WallServoing::WallOrientationSrv)
            (fullfills Wall::DetectorNew  Base::WorldXYPositionControllerSrv)
            (fullfills Wall::DetectorNew  WallServoing::WallOrientationSrv)
            (fullfills Wall::DetectorNew  Base::WorldXYPositionControllerSrv)
            (fullfills ConsWA  ConsWA)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp  AuvControl::MotionControlCmp)
            (fullfills AuvControl::MotionControlCmp  Base::ControlLoop)
            (fullfills AuvControl::MotionControlCmp  Base::ControlLoop)
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
            (fullfills Buoy::ControllerNewCmp  Buoy::ControllerNewCmp)
            (fullfills Buoy::ControllerNewCmp  Base::WorldXYZPositionControllerSrv)
            (fullfills Buoy::ControllerNewCmp  Base::WorldXYZPositionControllerSrv)
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
            (fullfills AuvCont::PositionMoveCmp  AuvCont::PositionMoveCmp)
            (fullfills AuvCont::PositionMoveCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::PositionMoveCmp  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::PositionMoveCmp  Base::WorldXYZRollPitchYawControlledSystemSrv)
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
            (fullfills AuvControl::SimplePosMove  AuvControl::SimplePosMove)
            (fullfills AuvControl::SimplePosMove  Base::ControlLoop)
            (fullfills AuvControl::SimplePosMove  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::DetectorNewCmp  Buoy::DetectorNewCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Detector  Wall::Detector)
            (fullfills Wall::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
            (fullfills Wall::Detector  Base::AUVRelativeMotionControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
            (fullfills Wall::Detector  Base::ControllerSrv)
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
            (fullfills Modem::ModemCmp  Modem::ModemCmp)
            (fullfills GPSHelper::GPSWaypointsCmp  GPSHelper::GPSWaypointsCmp)
            (fullfills GPSHelper::GPSWaypointsCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills GPSHelper::GPSWaypointsCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvCont::Trajectory  AuvCont::Trajectory)
            (fullfills AuvCont::Trajectory  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::Trajectory  AuvCont::WorldPositionCmp)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills AuvCont::Trajectory  Base::JointsCommandSrv)
            (fullfills AuvCont::Trajectory  Base::WorldXYZRollPitchYawControlledSystemSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  PoseAuv::IKFOrientationEstimatorCmp)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationToCorrectSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationToCorrectSrv)
            (fullfills PoseAuv::IKFOrientationEstimatorCmp  Base::OrientationSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp  AuvControl::RelPosControlCmp)
            (fullfills AuvControl::RelPosControlCmp  Base::ControlLoop)
            (fullfills AuvControl::RelPosControlCmp  Base::ControlLoop)
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
            (fullfills PoseAuv::InitialOrientationEstimatorCmp  PoseAuv::InitialOrientationEstimatorCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::RawCommandCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::AUVRelativeMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_, controlled_system.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControllerSrv_,controlled_system.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_, controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Pipeline::Follower)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Pipeline::Follower__controller.is_a__Base::AUVRelativeMotionControllerSrv_,controlled_system.is_a__Base::AUVRelativeMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills blueview_cmp  blueview_cmp)
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
            (fullfills front_camera_cmp  front_camera_cmp)
            (fullfills bottom_camera_cmp  bottom_camera_cmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::AUVMotionCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::AUVMotionControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::SonarFeatureDetectorCmp  Localization::SonarFeatureDetectorCmp)
            (fullfills Localization::SonarFeatureDetectorCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills Localization::SonarFeatureDetectorCmp  Base::WorldXYZRollPitchYawControllerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills Localization::FixMapHack  Localization::FixMapHack)
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
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsStatusSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::JointsCommandConsumerSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlledSystemSrv)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp__controller.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_, controlled_system.is_a__Base::JointsControlledSystemSrv__)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove__controller.is_a__Base::JointsControllerSrv_,controlled_system.is_a__Base::JointsControlledSystemSrv__  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp  AuvControl::PositionControlCmp)
            (fullfills AuvControl::PositionControlCmp  Base::ControlLoop)
            (fullfills AuvControl::PositionControlCmp  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp  Base::ControlLoop)
            (fullfills Buoy::DetectorCmp_Base::ControllerSrv_  Buoy::DetectorCmp)
            (fullfills Buoy::DetectorCmp_Base::ControllerSrv_  Base::ControllerSrv)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Wall::Follower)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Wall::Follower__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvControl::SimpleMove  AuvControl::SimpleMove)
            (fullfills AuvControl::SimpleMove  Base::ControlLoop)
            (fullfills AuvControl::SimpleMove  Base::ControlLoop)
            (fullfills Buoy::DoubleBuoyCmp  Buoy::DoubleBuoyCmp)
            (fullfills Localization::HoughDetector  Localization::HoughDetector)
            (fullfills Localization::HoughDetector  Localization::HoughSrv)
            (fullfills Localization::HoughDetector  Localization::HoughSrv)
            (fullfills AuvControl::DepthFusionCmp  AuvControl::DepthFusionCmp)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
            (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
            (fullfills AuvControl::JoystickCommandCmp  AuvControl::JoystickCommandCmp)
            (fullfills AuvControl::JoystickCommandCmp  Base::AUVMotionControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::WorldXYVelocityControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::AUVMotionControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::ControllerSrv)
            (fullfills AuvControl::JoystickCommandCmp  Base::WorldXYVelocityControllerSrv)
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
            (fullfills AuvCont::BuoyWallCmp  AuvCont::BuoyWallCmp)
            (fullfills AuvCont::BuoyWallCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::BuoyWallCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::BuoyWallCmp  Base::JointsCommandSrv)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_, controlled_system.is_a__Base::RawCommandControlledSystemSrv__)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Buoy::FollowerCmp)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
            (fullfills Buoy::FollowerCmp__controller.is_a__Base::RawCommandControllerSrv_,controlled_system.is_a__Base::RawCommandControlledSystemSrv__  Base::ControlLoop)
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
            (fullfills AuvCont::WorldXYZPositionCmp  AuvCont::WorldXYZPositionCmp)
            (fullfills AuvCont::WorldXYZPositionCmp  Base::JointsCommandSrv)
            (fullfills AuvCont::WorldXYZPositionCmp  Base::JointsCommandSrv)
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

; Begin inputs 
            (has_input SonarWallHough::Task SonarWallHough::Task.sonar_samples)
            (has_input SonarWallHough::Task SonarWallHough::Task.reset)
            (has_input SonarWallHough::Task SonarWallHough::Task.orientation_samples)
            (has_input SonarWallHough::Task SonarWallHough::Task.pose_samples)
            (has_input Canbus::InterfaceTask Canbus::InterfaceTask.can_in)
            (has_input VideoStreamerVlc::Streamer{1} VideoStreamerVlc::Streamer{1}.frame_bottom_camera)
            (has_input VideoStreamerVlc::Streamer{1} VideoStreamerVlc::Streamer{1}.frame_front_camera)
            (has_input VideoStreamerVlc::Streamer{1} VideoStreamerVlc::Streamer{1}.frame_blueview)
            (has_input Canbus::Task Canbus::Task.in)
            (has_input Buoy::ServoingOnWall Buoy::ServoingOnWall.buoy_samples)
            (has_input Buoy::ServoingOnWall Buoy::ServoingOnWall.wall_samples)
            (has_input Buoy::ServoingOnWall Buoy::ServoingOnWall.orientation_samples)
            (has_input Buoy::Detector2 Buoy::Detector2.frame)
            (has_input VideoStreamerVlc::Streamer VideoStreamerVlc::Streamer.frame_bottom_camera)
            (has_input VideoStreamerVlc::Streamer VideoStreamerVlc::Streamer.frame_front_camera)
            (has_input VideoStreamerVlc::Streamer VideoStreamerVlc::Streamer.frame_blueview)
            (has_input XsensImu::Task XsensImu::Task.hard_timestamps)
            (has_input Buoy::Detector Buoy::Detector.frame)
            (has_input PipelineInspection::Inspection PipelineInspection::Inspection.laserSamples)
            (has_input PipelineInspection::Inspection PipelineInspection::Inspection.laserPoints)
            (has_input PipelineInspection::Inspection PipelineInspection::Inspection.laserPointCloud)
            (has_input PipelineInspection::Inspection PipelineInspection::Inspection.pipeline)
            (has_input PipelineInspection::Inspection PipelineInspection::Inspection.dead_reckoning)
            (has_input PipelineInspection::ColorFilter PipelineInspection::ColorFilter.frame_in)
            (has_input Buoy::Survey Buoy::Survey.orientation_samples)
            (has_input Buoy::Survey Buoy::Survey.force_cutting)
            (has_input Buoy::Survey Buoy::Survey.input_buoy)
            (has_input Buoy::Survey Buoy::Survey.motion_command)
            (has_input Buoy::Survey Buoy::Survey.light)
            (has_input Buoy::Survey Buoy::Survey.target_angle_input)
            (has_input FogKvh::Dsp3000Task FogKvh::Dsp3000Task.config)
            (has_input OrientationEstimator::IKF OrientationEstimator::IKF.imu_samples)
            (has_input OrientationEstimator::IKF OrientationEstimator::IKF.fog_samples)
            (has_input OrientationEstimator::IKF OrientationEstimator::IKF.initial_orientation)
            (has_input OrientationEstimator::IKF OrientationEstimator::IKF.dynamic_transformations)
            (has_input UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.position_samples)
            (has_input UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.depth_samples)
            (has_input UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.orientation_samples)
            (has_input UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.velocity_samples)
            (has_input UwParticleLocalization::MotionModel UwParticleLocalization::MotionModel.thruster_samples)
            (has_input UwParticleLocalization::MotionModel UwParticleLocalization::MotionModel.orientation_samples)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.depth_samples)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.ShortExposure)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.LongExposure)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.LightValue)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.DebugLED)
            (has_input LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.LaserRate)
            (has_input OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.imu_orientation)
            (has_input OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.fog_samples)
            (has_input OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.heading_correction)
            (has_input StructureReconstruction::Task StructureReconstruction::Task.front_camera)
            (has_input StructureReconstruction::Task StructureReconstruction::Task.bottom_camera)
            (has_input StructureReconstruction::Task StructureReconstruction::Task.dynamic_transformations)
            (has_input UwParticleLocalization::OrientationCorrection UwParticleLocalization::OrientationCorrection.orientation_input)
            (has_input UwParticleLocalization::OrientationCorrection UwParticleLocalization::OrientationCorrection.orientation_offset)
            (has_input LineScanner::Task LineScanner::Task.frame)
            (has_input AuvRelPosController::Task AuvRelPosController::Task.position_command)
            (has_input AuvRelPosController::Task AuvRelPosController::Task.position_sample)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.laser_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.speed_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.pipeline_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.pose_update)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.gps_pose_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.buoy_samples_orange)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.buoy_samples_white)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.thruster_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.orientation_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.echosounder_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.obstacle_samples)
            (has_input UwParticleLocalization::Task UwParticleLocalization::Task.structur_samples)
            (has_input ModemCan::Task ModemCan::Task.modem_in)
            (has_input ModemCan::Task ModemCan::Task.canModem)
            (has_input ModemCan::Task ModemCan::Task.light_value)
            (has_input ModemCan::Task ModemCan::Task.position_samples)
            (has_input DepthReader::Task DepthReader::Task.canIn)
            (has_input SonarStructureServoing::Task SonarStructureServoing::Task.sonarbeam_feature)
            (has_input SonarStructureServoing::Task SonarStructureServoing::Task.odometry_samples)
            (has_input SonarStructureServoing::Task SonarStructureServoing::Task.dynamic_transformations)
            (has_input Lights::Lights Lights::Lights.int_in)
            (has_input Lights::Lights Lights::Lights.can_in)
            (has_input WallServoing::DualSonarServoing WallServoing::DualSonarServoing.sonarbeam_feature_front)
            (has_input WallServoing::DualSonarServoing WallServoing::DualSonarServoing.sonarbeam_feature_rear)
            (has_input WallServoing::DualSonarServoing WallServoing::DualSonarServoing.orientation_sample)
            (has_input AvalonControl::MotionFeedbackTask AvalonControl::MotionFeedbackTask.hbridge_feedback)
            (has_input BatteryWatcher::Task BatteryWatcher::Task.can_in)
            (has_input ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.color_frame)
            (has_input ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.frame)
            (has_input ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.dynamic_transformations)
            (has_input AuvControl::AccelerationController AuvControl::AccelerationController.cmd_in)
            (has_input AuvControl::AccelerationController AuvControl::AccelerationController.cmd_cascade)
            (has_input WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.sonarbeam_feature)
            (has_input WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.orientation_sample)
            (has_input WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.position_sample)
            (has_input SonarFeatureEstimator::Task SonarFeatureEstimator::Task.sonar_input)
            (has_input SonarFeatureEstimator::Task SonarFeatureEstimator::Task.orientation_sample)
            (has_input AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.pose_samples)
            (has_input AuvControl::AlignedToBody AuvControl::AlignedToBody.cmd_in)
            (has_input AuvControl::AlignedToBody AuvControl::AlignedToBody.cmd_cascade)
            (has_input AuvControl::AlignedToBody AuvControl::AlignedToBody.orientation_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.orientation_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.depth_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.dvl_velocity_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.model_velocity_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.lbl_position_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.xy_position_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.gps_position_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.xyz_position_samples)
            (has_input PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.dynamic_transformations)
            (has_input PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.pose_samples_fast)
            (has_input PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.pose_samples_slow)
            (has_input PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.xy_position_samples)
            (has_input PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.dynamic_transformations)
            (has_input PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.orientation_samples)
            (has_input PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.velocity_samples)
            (has_input PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.position_samples)
            (has_input PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.dynamic_transformations)
            (has_input WallServoing::WallServoing WallServoing::WallServoing.orientation_sample)
            (has_input WallServoing::WallServoing WallServoing::WallServoing.servoing_wall)
            (has_input WallServoing::WallServoing WallServoing::WallServoing.obstacle_wall)
            (has_input WallServoing::WallDetector WallServoing::WallDetector.sonarbeam_feature)
            (has_input WallServoing::WallDetector WallServoing::WallDetector.orientation_sample)
            (has_input WallServoing::WallDetector WallServoing::WallDetector.position_sample)
            (has_input Interfaces::Servo Interfaces::Servo.cmd_angle)
            (has_input AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.dummy_feedback)
            (has_input AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.ground_distance)
            (has_input AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.pose_samples)
            (has_input AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.motion_commands)
            (has_input AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.joints_in)
            (has_input AuvControl::WorldToAligned AuvControl::WorldToAligned.cmd_in)
            (has_input AuvControl::WorldToAligned AuvControl::WorldToAligned.cmd_cascade)
            (has_input AuvControl::WorldToAligned AuvControl::WorldToAligned.pose_samples)
            (has_input CameraBase::Preprocess CameraBase::Preprocess.iframe)
            (has_input AuvControl::PIDController AuvControl::PIDController.cmd_in)
            (has_input AuvControl::PIDController AuvControl::PIDController.cmd_cascade)
            (has_input AuvControl::PIDController AuvControl::PIDController.pose_samples)
            (has_input AvalonControl::MotionControlTask AvalonControl::MotionControlTask.dummy_feedback)
            (has_input AvalonControl::MotionControlTask AvalonControl::MotionControlTask.ground_distance)
            (has_input AvalonControl::MotionControlTask AvalonControl::MotionControlTask.pose_samples)
            (has_input AvalonControl::MotionControlTask AvalonControl::MotionControlTask.motion_commands)
            (has_input AvalonControl::MotionControlTask AvalonControl::MotionControlTask.joints_in)
            (has_input AuvControl::BasePIDController AuvControl::BasePIDController.cmd_in)
            (has_input AuvControl::BasePIDController AuvControl::BasePIDController.cmd_cascade)
            (has_input AuvControl::BasePIDController AuvControl::BasePIDController.pose_samples)
            (has_input Modemdriver::ModemCanbus Modemdriver::ModemCanbus.data_in)
            (has_input Modemdriver::ModemCanbus Modemdriver::ModemCanbus.can_in)
            (has_input Hbridge::CommandWriter Hbridge::CommandWriter.can_in)
            (has_input Hbridge::CommandWriter Hbridge::CommandWriter.command)
            (has_input Modemdriver::ModemSerial Modemdriver::ModemSerial.data_in)
            (has_input WallOrientationCorrection::Task WallOrientationCorrection::Task.sonarbeam_feature)
            (has_input WallOrientationCorrection::Task WallOrientationCorrection::Task.orientation_samples)
            (has_input WallOrientationCorrection::Task WallOrientationCorrection::Task.dynamic_transformations)
            (has_input AvalonControl::PositionControlTask AvalonControl::PositionControlTask.pose_samples)
            (has_input AvalonControl::PositionControlTask AvalonControl::PositionControlTask.position_commands)
            (has_input HsvMosaicing::Task HsvMosaicing::Task.frame)
            (has_input Modemdriver::Modem Modemdriver::Modem.data_in)
            (has_input AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.orientation_samples)
            (has_input AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.depth_samples)
            (has_input AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.ground_distance)
            (has_input WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.orientation_in_world)
            (has_input WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.dynamic_transformations)
            (has_input OffshorePipelineDetector::Task OffshorePipelineDetector::Task.frame)
            (has_input OffshorePipelineDetector::Task OffshorePipelineDetector::Task.orientation_sample)
            (has_input OffshorePipelineDetector::Task OffshorePipelineDetector::Task.altitude_samples)
            (has_input AuvControl::Base AuvControl::Base.cmd_in)
            (has_input AuvControl::Base AuvControl::Base.cmd_cascade)
            (has_input Hbridge::SensorReader Hbridge::SensorReader.can_in)
            (has_input AuvControl::OptimalHeadingController AuvControl::OptimalHeadingController.cmd_in)
            (has_input AuvControl::OptimalHeadingController AuvControl::OptimalHeadingController.cmd_cascade)
            (has_input AuvControl::OptimalHeadingController AuvControl::OptimalHeadingController.orientation_samples)
            (has_input OffshorePipelineDetector::SonarDetector OffshorePipelineDetector::SonarDetector.sonar_beam)
            (has_input StructureServoing::Alignment StructureServoing::Alignment.input)
            (has_input StructureServoing::Task StructureServoing::Task.rbs)
            (has_input StructureServoing::Task StructureServoing::Task.input)
            (has_input GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.gps_position_samples)
            (has_input GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.pose_samples)
            (has_input AuvControl::MotionCommand2DConverter AuvControl::MotionCommand2DConverter.cmd_in)
            (has_input Sysmon::Task Sysmon::Task.can_in_system_status)
            (has_input Sysmon::Task Sysmon::Task.can_in_experiment_markers)
            (has_input Sysmon::Task Sysmon::Task.in_experiment_markers)
            (has_input Sysmon::Task Sysmon::Task.in_modem_substates)
            (has_input GpsHelper::MapToGPS GpsHelper::MapToGPS.position_samples)
            (has_input GpsHelper::MapToGPS GpsHelper::MapToGPS.dynamic_transformations)
            (has_input AuvControl::WaypointNavigator AuvControl::WaypointNavigator.trajectory)
            (has_input AuvControl::WaypointNavigator AuvControl::WaypointNavigator.pose_sample)
            (has_input Controldev::Remote Controldev::Remote.canInputDevice)
            (has_input SonarFeatureDetector::Task SonarFeatureDetector::Task.grid_maps)
            (has_input SonarFeatureDetector::Task SonarFeatureDetector::Task.pose_samples)
            (has_input RawControlCommandConverter::Position RawControlCommandConverter::Position.raw_command)
            (has_input RawControlCommandConverter::Position RawControlCommandConverter::Position.pose_samples)
            (has_input AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.altimeter)
            (has_input AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.depth)
            (has_input AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.cmd_in)
            (has_input Controldev::GenericRawToMotion2D Controldev::GenericRawToMotion2D.raw_command)
            (has_input ImagePreprocessing::StereoTask ImagePreprocessing::StereoTask.frame_left)
            (has_input ImagePreprocessing::StereoTask ImagePreprocessing::StereoTask.frame_right)
            (has_input RawControlCommandConverter::Movement RawControlCommandConverter::Movement.raw_command)
            (has_input RawControlCommandConverter::Movement RawControlCommandConverter::Movement.orientation_readings)
            (has_input RawControlCommandConverter::Movement RawControlCommandConverter::Movement.ground_distance)
            (has_input Controldev::RawWheelToMotion2D Controldev::RawWheelToMotion2D.raw_command)
            (has_input ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.frame)
            (has_input AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.altimeter)
            (has_input AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.depth)
            (has_input AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.cmd_in)
            (has_input Controldev::RawJoystickToMotion2D Controldev::RawJoystickToMotion2D.raw_command)
            (has_input ImagePreprocessing::MonoTask ImagePreprocessing::MonoTask.frame)
            (has_input AuvWaypointNavigator::Task AuvWaypointNavigator::Task.trajectory)
            (has_input AuvWaypointNavigator::Task AuvWaypointNavigator::Task.pose_samples)
            (has_input RearSonarDistanceEstimator::Task RearSonarDistanceEstimator::Task.BaseScan)
            (has_input RearSonarDistanceEstimator::Task RearSonarDistanceEstimator::Task.depth_samples)
            (has_input Dynamixel::Task Dynamixel::Task.cmd_angle)
            (has_input StructuredLight::Calibration StructuredLight::Calibration.laser_scan)
            (has_input StructuredLight::Calibration StructuredLight::Calibration.calibration)
            (has_input StructuredLight::Task StructuredLight::Task.frame_pair)
            (has_input StructuredLight::Task StructuredLight::Task.frame)
            (has_input FrameDemultiplexer::Task FrameDemultiplexer::Task.iframe)

; Begin outputs 
            (has_output SonarWallHough::Task SonarWallHough::Task.lines)
            (has_output SonarWallHough::Task SonarWallHough::Task.peaks)
            (has_output SonarWallHough::Task SonarWallHough::Task.houghspace)
            (has_output SonarWallHough::Task SonarWallHough::Task.position)
            (has_output SonarWallHough::Task SonarWallHough::Task.position_quality)
            (has_output SonarWallHough::Task SonarWallHough::Task.state)
            (has_output Gps::MB500Task Gps::MB500Task.solution)
            (has_output Gps::MB500Task Gps::MB500Task.position_samples)
            (has_output Gps::MB500Task Gps::MB500Task.state)
            (has_output Gps::MB500Task Gps::MB500Task.constellation)
            (has_output Gps::MB500Task Gps::MB500Task.time)
            (has_output Gps::GPSDTask Gps::GPSDTask.solution)
            (has_output Gps::GPSDTask Gps::GPSDTask.position_samples)
            (has_output Gps::GPSDTask Gps::GPSDTask.state)
            (has_output Gps::BaseTask Gps::BaseTask.solution)
            (has_output Gps::BaseTask Gps::BaseTask.position_samples)
            (has_output Gps::BaseTask Gps::BaseTask.state)
            (has_output Canbus::InterfaceTask Canbus::InterfaceTask.can_out)
            (has_output Canbus::InterfaceTask Canbus::InterfaceTask.state)
            (has_output VideoStreamerVlc::Streamer{1} VideoStreamerVlc::Streamer{1}.state)
            (has_output Canbus::Task Canbus::Task.stats)
            (has_output Canbus::Task Canbus::Task.can_status)
            (has_output Canbus::Task Canbus::Task.log_message)
            (has_output Canbus::Task Canbus::Task.state)
            (has_output Transformer::Task Transformer::Task.configuration_state)
            (has_output Transformer::Task Transformer::Task.state)
            (has_output VideoStreamerVlc::Capturer VideoStreamerVlc::Capturer.state)
            (has_output Buoy::ServoingOnWall Buoy::ServoingOnWall.world_cmd)
            (has_output Buoy::ServoingOnWall Buoy::ServoingOnWall.aligned_position_cmd)
            (has_output Buoy::ServoingOnWall Buoy::ServoingOnWall.state)
            (has_output Buoy::Detector2 Buoy::Detector2.state)
            (has_output Buoy::Detector2 Buoy::Detector2.buoy)
            (has_output Buoy::Detector2 Buoy::Detector2.light)
            (has_output Buoy::Detector2 Buoy::Detector2.h_image)
            (has_output Buoy::Detector2 Buoy::Detector2.s_image)
            (has_output Buoy::Detector2 Buoy::Detector2.v_image)
            (has_output Buoy::Detector2 Buoy::Detector2.binary_debug_image)
            (has_output Buoy::Detector2 Buoy::Detector2.gray_debug_image)
            (has_output Buoy::Detector2 Buoy::Detector2.hough_debug_image)
            (has_output Buoy::Detector2 Buoy::Detector2.other_buoys)
            (has_output Buoy::Detector2 Buoy::Detector2.debug_image)
            (has_output VideoStreamerVlc::Streamer VideoStreamerVlc::Streamer.state)
            (has_output XsensImu::Task XsensImu::Task.orientation_samples)
            (has_output XsensImu::Task XsensImu::Task.calibrated_sensors)
            (has_output XsensImu::Task XsensImu::Task.timestamp_estimator_status)
            (has_output XsensImu::Task XsensImu::Task.state)
            (has_output PipelineInspection::LaserSimulation PipelineInspection::LaserSimulation.laserPoints)
            (has_output PipelineInspection::LaserSimulation PipelineInspection::LaserSimulation.laserPointCloud)
            (has_output PipelineInspection::LaserSimulation PipelineInspection::LaserSimulation.vehiclePos)
            (has_output PipelineInspection::LaserSimulation PipelineInspection::LaserSimulation.state)
            (has_output Buoy::Detector Buoy::Detector.state)
            (has_output Buoy::Detector Buoy::Detector.buoy)
            (has_output Buoy::Detector Buoy::Detector.light)
            (has_output Buoy::Detector Buoy::Detector.h_image)
            (has_output Buoy::Detector Buoy::Detector.s_image)
            (has_output Buoy::Detector Buoy::Detector.v_image)
            (has_output Buoy::Detector Buoy::Detector.binary_debug_image)
            (has_output Buoy::Detector Buoy::Detector.gray_debug_image)
            (has_output Buoy::Detector Buoy::Detector.hough_debug_image)
            (has_output Buoy::Detector Buoy::Detector.other_buoys)
            (has_output Buoy::Detector Buoy::Detector.debug_image)
            (has_output PipelineInspection::Inspection PipelineInspection::Inspection.inspectionStatus)
            (has_output PipelineInspection::Inspection PipelineInspection::Inspection.pipePoints)
            (has_output PipelineInspection::Inspection PipelineInspection::Inspection.debugFrame)
            (has_output PipelineInspection::Inspection PipelineInspection::Inspection.pipeMap)
            (has_output PipelineInspection::Inspection PipelineInspection::Inspection.state)
            (has_output Logger::Logger Logger::Logger.state)
            (has_output PipelineInspection::ColorFilter PipelineInspection::ColorFilter.frame_out)
            (has_output PipelineInspection::ColorFilter PipelineInspection::ColorFilter.green_frame)
            (has_output PipelineInspection::ColorFilter PipelineInspection::ColorFilter.diff_frame)
            (has_output PipelineInspection::ColorFilter PipelineInspection::ColorFilter.state)
            (has_output Buoy::Survey Buoy::Survey.state)
            (has_output Buoy::Survey Buoy::Survey.strafed_angle)
            (has_output Buoy::Survey Buoy::Survey.relative_position)
            (has_output Buoy::Survey Buoy::Survey.position)
            (has_output FogKvh::Dsp3000Task FogKvh::Dsp3000Task.rotation)
            (has_output FogKvh::Dsp3000Task FogKvh::Dsp3000Task.orientation_samples)
            (has_output FogKvh::Dsp3000Task FogKvh::Dsp3000Task.timestamp_estimator_status)
            (has_output FogKvh::Dsp3000Task FogKvh::Dsp3000Task.state)
            (has_output OrientationEstimator::IKF OrientationEstimator::IKF.transformer_stream_aligner_status)
            (has_output OrientationEstimator::IKF OrientationEstimator::IKF.transformer_status)
            (has_output OrientationEstimator::IKF OrientationEstimator::IKF.attitude_b_g)
            (has_output OrientationEstimator::IKF OrientationEstimator::IKF.state)
            (has_output UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.pose_samples)
            (has_output UwParticleLocalization::FastFusion UwParticleLocalization::FastFusion.state)
            (has_output UwParticleLocalization::MotionModel UwParticleLocalization::MotionModel.pose_samples)
            (has_output UwParticleLocalization::MotionModel UwParticleLocalization::MotionModel.stream_aligner_status)
            (has_output UwParticleLocalization::MotionModel UwParticleLocalization::MotionModel.state)
            (has_output LowLevelDriver::LowLevelTask LowLevelDriver::LowLevelTask.state)
            (has_output OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.stream_aligner_status)
            (has_output OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.attitude_b_g)
            (has_output OrientationEstimator::BaseEstimator OrientationEstimator::BaseEstimator.state)
            (has_output StructureReconstruction::Task StructureReconstruction::Task.transformer_stream_aligner_status)
            (has_output StructureReconstruction::Task StructureReconstruction::Task.transformer_status)
            (has_output StructureReconstruction::Task StructureReconstruction::Task.state)
            (has_output UwParticleLocalization::OrientationCorrection UwParticleLocalization::OrientationCorrection.orientation_output)
            (has_output UwParticleLocalization::OrientationCorrection UwParticleLocalization::OrientationCorrection.orientation_offset_corrected)
            (has_output UwParticleLocalization::OrientationCorrection UwParticleLocalization::OrientationCorrection.state)
            (has_output LineScanner::Task LineScanner::Task.state)
            (has_output LineScanner::Task LineScanner::Task.pointcloud)
            (has_output LineScanner::Task LineScanner::Task.debug)
            (has_output AuvRelPosController::Task AuvRelPosController::Task.motion_command)
            (has_output AuvRelPosController::Task AuvRelPosController::Task.state)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.pose_samples)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.environment)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.dead_reckoning_samples)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.full_dead_reckoning)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.particles)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.debug_sonar_beam)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.stats)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.depth_grid)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.grid_map)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.debug_filtered_obstacles)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.stream_aligner_status)
            (has_output UwParticleLocalization::Task UwParticleLocalization::Task.state)
            (has_output ModemCan::Task ModemCan::Task.modem_out)
            (has_output ModemCan::Task ModemCan::Task.canOut)
            (has_output ModemCan::Task ModemCan::Task.motion_command)
            (has_output ModemCan::Task ModemCan::Task.state)
            (has_output DepthReader::Task DepthReader::Task.depthOut)
            (has_output DepthReader::Task DepthReader::Task.depth_samples)
            (has_output DepthReader::Task DepthReader::Task.canOut)
            (has_output DepthReader::Task DepthReader::Task.state)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.position_command)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.aligned_position_command)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.world_command)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.debug_data)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.transformer_stream_aligner_status)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.transformer_status)
            (has_output SonarStructureServoing::Task SonarStructureServoing::Task.state)
            (has_output Lights::Lights Lights::Lights.light_value)
            (has_output Lights::Lights Lights::Lights.state)
            (has_output WallServoing::DualSonarServoing WallServoing::DualSonarServoing.state)
            (has_output WallServoing::DualSonarServoing WallServoing::DualSonarServoing.position_command)
            (has_output WallServoing::DualSonarServoing WallServoing::DualSonarServoing.aligned_command)
            (has_output WallServoing::DualSonarServoing WallServoing::DualSonarServoing.wall_servoing_debug)
            (has_output AvalonControl::MotionFeedbackTask AvalonControl::MotionFeedbackTask.hbridge_status)
            (has_output AvalonControl::MotionFeedbackTask AvalonControl::MotionFeedbackTask.state)
            (has_output BatteryWatcher::Task BatteryWatcher::Task.can_out)
            (has_output BatteryWatcher::Task BatteryWatcher::Task.state)
            (has_output BatteryWatcher::Task BatteryWatcher::Task.battery_info)
            (has_output ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.pointcloud)
            (has_output ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.stream_aligner_status)
            (has_output ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.transformer_stream_aligner_status)
            (has_output ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.transformer_status)
            (has_output ImagePreprocessing::DepthImage2Pointcloud ImagePreprocessing::DepthImage2Pointcloud.state)
            (has_output AuvControl::AccelerationController AuvControl::AccelerationController.state)
            (has_output AuvControl::AccelerationController AuvControl::AccelerationController.cmd_out)
            (has_output SonarTritech::Micron SonarTritech::Micron.ground_distance)
            (has_output SonarTritech::Micron SonarTritech::Micron.sonar_beam)
            (has_output SonarTritech::Micron SonarTritech::Micron.state)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.state)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.position_command)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.aligned_position_command)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.world_command)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.wall_servoing_debug)
            (has_output WallServoing::SingleSonarServoing WallServoing::SingleSonarServoing.wall)
            (has_output SonarFeatureEstimator::Task SonarFeatureEstimator::Task.new_feature)
            (has_output SonarFeatureEstimator::Task SonarFeatureEstimator::Task.features_out)
            (has_output SonarFeatureEstimator::Task SonarFeatureEstimator::Task.debug_output)
            (has_output SonarFeatureEstimator::Task SonarFeatureEstimator::Task.2d_debug_output)
            (has_output SonarFeatureEstimator::Task SonarFeatureEstimator::Task.state)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.next_position)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.position_command)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.next_pos_on_spline)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.last_pos_on_spline)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.segment_dist)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.world_command)
            (has_output AvalonControl::TrajectoryFollower AvalonControl::TrajectoryFollower.state)
            (has_output WallServoing::SonarServoing WallServoing::SonarServoing.state)
            (has_output ImagePreprocessing::BaseTask ImagePreprocessing::BaseTask.state)
            (has_output PoseEstimation::BaseTask PoseEstimation::BaseTask.pose_samples)
            (has_output PoseEstimation::BaseTask PoseEstimation::BaseTask.state)
            (has_output AuvControl::AlignedToBody AuvControl::AlignedToBody.state)
            (has_output AuvControl::AlignedToBody AuvControl::AlignedToBody.cmd_out)
            (has_output PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.pose_samples)
            (has_output PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.state)
            (has_output PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.transformer_stream_aligner_status)
            (has_output PoseEstimation::UWPoseEstimator PoseEstimation::UWPoseEstimator.transformer_status)
            (has_output PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.pose_samples)
            (has_output PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.state)
            (has_output PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.transformer_stream_aligner_status)
            (has_output PoseEstimation::HighDelayPoseEstimator PoseEstimation::HighDelayPoseEstimator.transformer_status)
            (has_output PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.pose_samples)
            (has_output PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.state)
            (has_output PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.transformer_stream_aligner_status)
            (has_output PoseEstimation::VehiclePoseEstimator PoseEstimation::VehiclePoseEstimator.transformer_status)
            (has_output WallServoing::WallServoing WallServoing::WallServoing.motion_command)
            (has_output WallServoing::WallServoing WallServoing::WallServoing.world_command)
            (has_output WallServoing::WallServoing WallServoing::WallServoing.aligned_velocity_command)
            (has_output WallServoing::WallServoing WallServoing::WallServoing.state)
            (has_output CameraProsilica::Task CameraProsilica::Task.frame)
            (has_output CameraProsilica::Task CameraProsilica::Task.frame_raw)
            (has_output CameraProsilica::Task CameraProsilica::Task.state)
            (has_output WallServoing::WallDetector WallServoing::WallDetector.point_cloud)
            (has_output WallServoing::WallDetector WallServoing::WallDetector.wall)
            (has_output WallServoing::WallDetector WallServoing::WallDetector.state)
            (has_output SonarTritech::Echosounder SonarTritech::Echosounder.ground_distance)
            (has_output SonarTritech::Echosounder SonarTritech::Echosounder.state)
            (has_output SonarTritech::Profiling SonarTritech::Profiling.profiling_scan)
            (has_output SonarTritech::Profiling SonarTritech::Profiling.state)
            (has_output Interfaces::Servo Interfaces::Servo.upper2lower)
            (has_output Interfaces::Servo Interfaces::Servo.angle)
            (has_output Interfaces::Servo Interfaces::Servo.state)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.hbridge_commands)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.joint_commands)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.debug)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.estimated_ground_pos)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.state)
            (has_output AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_ AvalonControl::MotionControlTask_Base::AUVMotionControllerSrv_.command_out)
            (has_output AuvControl::WorldToAligned AuvControl::WorldToAligned.state)
            (has_output AuvControl::WorldToAligned AuvControl::WorldToAligned.cmd_out)
            (has_output CameraBase::Preprocess CameraBase::Preprocess.state)
            (has_output CameraBase::Preprocess CameraBase::Preprocess.oframe)
            (has_output AuvControl::PIDController AuvControl::PIDController.state)
            (has_output AuvControl::PIDController AuvControl::PIDController.cmd_out)
            (has_output AuvControl::PIDController AuvControl::PIDController.pid_state)
            (has_output AvalonControl::RelFakeWriter AvalonControl::RelFakeWriter.position_command)
            (has_output AvalonControl::RelFakeWriter AvalonControl::RelFakeWriter.state)
            (has_output AvalonControl::FakeWriter AvalonControl::FakeWriter.motion_commands)
            (has_output AvalonControl::FakeWriter AvalonControl::FakeWriter.state)
            (has_output AvalonControl::MotionControlTask AvalonControl::MotionControlTask.hbridge_commands)
            (has_output AvalonControl::MotionControlTask AvalonControl::MotionControlTask.joint_commands)
            (has_output AvalonControl::MotionControlTask AvalonControl::MotionControlTask.debug)
            (has_output AvalonControl::MotionControlTask AvalonControl::MotionControlTask.estimated_ground_pos)
            (has_output AvalonControl::MotionControlTask AvalonControl::MotionControlTask.state)
            (has_output AuvControl::BasePIDController AuvControl::BasePIDController.state)
            (has_output AuvControl::BasePIDController AuvControl::BasePIDController.cmd_out)
            (has_output AuvControl::BasePIDController AuvControl::BasePIDController.pid_state)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.data_out)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.distance)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.out_modem_substates)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.state)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.can_out)
            (has_output Modemdriver::ModemCanbus Modemdriver::ModemCanbus.stats)
            (has_output Hbridge::CommandWriter Hbridge::CommandWriter.state)
            (has_output Hbridge::CommandWriter Hbridge::CommandWriter.can_out)
            (has_output Hbridge::CommandWriter Hbridge::CommandWriter.speedCtrlDebug)
            (has_output Hbridge::CommandWriter Hbridge::CommandWriter.fakeReader)
            (has_output Modemdriver::ModemSerial Modemdriver::ModemSerial.data_out)
            (has_output Modemdriver::ModemSerial Modemdriver::ModemSerial.distance)
            (has_output Modemdriver::ModemSerial Modemdriver::ModemSerial.out_modem_substates)
            (has_output Modemdriver::ModemSerial Modemdriver::ModemSerial.state)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.orientation_in_world)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.angle_in_world)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.debug_data)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.transformer_stream_aligner_status)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.transformer_status)
            (has_output WallOrientationCorrection::Task WallOrientationCorrection::Task.state)
            (has_output AvalonControl::PositionControlTask AvalonControl::PositionControlTask.motion_commands)
            (has_output AvalonControl::PositionControlTask AvalonControl::PositionControlTask.state)
            (has_output HsvMosaicing::Task HsvMosaicing::Task.result)
            (has_output HsvMosaicing::Task HsvMosaicing::Task.state)
            (has_output Modemdriver::Modem Modemdriver::Modem.data_out)
            (has_output Modemdriver::Modem Modemdriver::Modem.distance)
            (has_output Modemdriver::Modem Modemdriver::Modem.out_modem_substates)
            (has_output Modemdriver::Modem Modemdriver::Modem.state)
            (has_output AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.pose_samples)
            (has_output AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.stream_aligner_status)
            (has_output AuvHelper::DepthAndOrientationFusion AuvHelper::DepthAndOrientationFusion.state)
            (has_output GpsHelper::GPSFaker GpsHelper::GPSFaker.position_samples)
            (has_output GpsHelper::GPSFaker GpsHelper::GPSFaker.state)
            (has_output WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.orientation_in_map)
            (has_output WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.transformer_stream_aligner_status)
            (has_output WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.transformer_status)
            (has_output WallOrientationCorrection::OrientationInMap WallOrientationCorrection::OrientationInMap.state)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.state)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.pipeline)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.world_command)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.aligned_position_command)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.position_command)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.debug)
            (has_output OffshorePipelineDetector::Task OffshorePipelineDetector::Task.debug_frame)
            (has_output AuvControl::Base AuvControl::Base.state)
            (has_output CameraBase::Task CameraBase::Task.frame)
            (has_output CameraBase::Task CameraBase::Task.frame_raw)
            (has_output CameraBase::Task CameraBase::Task.state)
            (has_output Hbridge::SensorReader Hbridge::SensorReader.state)
            (has_output Hbridge::SensorReader Hbridge::SensorReader.can_out)
            (has_output Hbridge::SensorReader Hbridge::SensorReader.status_samples)
            (has_output AuvControl::OptimalHeadingController AuvControl::OptimalHeadingController.state)
            (has_output AuvControl::OptimalHeadingController AuvControl::OptimalHeadingController.cmd_out)
            (has_output OffshorePipelineDetector::SonarDetector OffshorePipelineDetector::SonarDetector.frame)
            (has_output OffshorePipelineDetector::SonarDetector OffshorePipelineDetector::SonarDetector.state)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.world_command)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.aligned_speed_command)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.left)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.right)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.top)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.bottom)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.size)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.x)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.y)
            (has_output StructureServoing::Alignment StructureServoing::Alignment.state)
            (has_output SonarBlueview::Task SonarBlueview::Task.frame)
            (has_output SonarBlueview::Task SonarBlueview::Task.state)
            (has_output StructureServoing::Task StructureServoing::Task.servoed_angle)
            (has_output StructureServoing::Task StructureServoing::Task.angle_speed)
            (has_output StructureServoing::Task StructureServoing::Task.world_command)
            (has_output StructureServoing::Task StructureServoing::Task.aligned_speed_command)
            (has_output StructureServoing::Task StructureServoing::Task.found_structure)
            (has_output StructureServoing::Task StructureServoing::Task.state)
            (has_output StructureServoing::Task StructureServoing::Task.left)
            (has_output StructureServoing::Task StructureServoing::Task.right)
            (has_output StructureServoing::Task StructureServoing::Task.top)
            (has_output StructureServoing::Task StructureServoing::Task.bottom)
            (has_output StructureServoing::Task StructureServoing::Task.size)
            (has_output StructureServoing::Task StructureServoing::Task.heading)
            (has_output StructureServoing::Task StructureServoing::Task.cnt_left)
            (has_output StructureServoing::Task StructureServoing::Task.cnt_right)
            (has_output StructureServoing::Task StructureServoing::Task.cnt_top)
            (has_output StructureServoing::Task StructureServoing::Task.cnt_bottom)
            (has_output GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.target_waypoint)
            (has_output GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.heading_offset)
            (has_output GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.distance_delta)
            (has_output GpsHelper::WaypointNavigation GpsHelper::WaypointNavigation.state)
            (has_output AuvControl::MotionCommand2DConverter AuvControl::MotionCommand2DConverter.cmd_out)
            (has_output AuvControl::MotionCommand2DConverter AuvControl::MotionCommand2DConverter.state)
            (has_output Sysmon::Task Sysmon::Task.ocu_markers)
            (has_output Sysmon::Task Sysmon::Task.annotations)
            (has_output Sysmon::Task Sysmon::Task.system_status)
            (has_output Sysmon::Task Sysmon::Task.state)
            (has_output GpsHelper::MapToGPS GpsHelper::MapToGPS.gps_position)
            (has_output GpsHelper::MapToGPS GpsHelper::MapToGPS.transformer_stream_aligner_status)
            (has_output GpsHelper::MapToGPS GpsHelper::MapToGPS.transformer_status)
            (has_output GpsHelper::MapToGPS GpsHelper::MapToGPS.state)
            (has_output AuvControl::WaypointNavigator AuvControl::WaypointNavigator.cmd_out)
            (has_output AuvControl::WaypointNavigator AuvControl::WaypointNavigator.waypoint_info)
            (has_output AuvControl::WaypointNavigator AuvControl::WaypointNavigator.state)
            (has_output Controldev::GenericTask Controldev::GenericTask.raw_command)
            (has_output Controldev::GenericTask Controldev::GenericTask.state)
            (has_output Controldev::Remote Controldev::Remote.raw_command)
            (has_output Controldev::Remote Controldev::Remote.state)
            (has_output SonarFeatureDetector::Task SonarFeatureDetector::Task.features)
            (has_output SonarFeatureDetector::Task SonarFeatureDetector::Task.next_target)
            (has_output SonarFeatureDetector::Task SonarFeatureDetector::Task.next_target_feature)
            (has_output SonarFeatureDetector::Task SonarFeatureDetector::Task.next_target_command)
            (has_output SonarFeatureDetector::Task SonarFeatureDetector::Task.state)
            (has_output Controldev::JoystickTask Controldev::JoystickTask.raw_command)
            (has_output Controldev::JoystickTask Controldev::JoystickTask.state)
            (has_output RawControlCommandConverter::Position RawControlCommandConverter::Position.position_command)
            (has_output RawControlCommandConverter::Position RawControlCommandConverter::Position.world_command)
            (has_output RawControlCommandConverter::Position RawControlCommandConverter::Position.state)
            (has_output AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.floor_position)
            (has_output AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.state)
            (has_output AuvControl::ConstantCommandGroundAvoidance AuvControl::ConstantCommandGroundAvoidance.cmd_out)
            (has_output Controldev::GenericRawToMotion2D Controldev::GenericRawToMotion2D.motion_command)
            (has_output Controldev::GenericRawToMotion2D Controldev::GenericRawToMotion2D.state)
            (has_output ImagePreprocessing::StereoTask ImagePreprocessing::StereoTask.state)
            (has_output ImagePreprocessing::StereoTask ImagePreprocessing::StereoTask.oframe_pair)
            (has_output Controldev::JoyPadTask Controldev::JoyPadTask.raw_command)
            (has_output Controldev::JoyPadTask Controldev::JoyPadTask.state)
            (has_output RawControlCommandConverter::Movement RawControlCommandConverter::Movement.motion_command)
            (has_output RawControlCommandConverter::Movement RawControlCommandConverter::Movement.world_command)
            (has_output RawControlCommandConverter::Movement RawControlCommandConverter::Movement.world_command_depth)
            (has_output RawControlCommandConverter::Movement RawControlCommandConverter::Movement.aligned_velocity_command)
            (has_output RawControlCommandConverter::Movement RawControlCommandConverter::Movement.state)
            (has_output Controldev::Mouse3DTask Controldev::Mouse3DTask.raw_command)
            (has_output Controldev::Mouse3DTask Controldev::Mouse3DTask.state)
            (has_output Controldev::RawWheelToMotion2D Controldev::RawWheelToMotion2D.motion_command)
            (has_output Controldev::RawWheelToMotion2D Controldev::RawWheelToMotion2D.state)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.oframe)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.binary_result)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.hDebug)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.hDebugGray)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.vDebug)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.vDebugGray)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.sDebug)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.sDebugGray)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.hsv_v_frame)
            (has_output ImagePreprocessing::HSVSegmentationAndBlur ImagePreprocessing::HSVSegmentationAndBlur.state)
            (has_output Interfaces::IMU Interfaces::IMU.orientation_samples)
            (has_output Interfaces::IMU Interfaces::IMU.calibrated_sensors)
            (has_output Interfaces::IMU Interfaces::IMU.state)
            (has_output Controldev::SteeringWheelTask Controldev::SteeringWheelTask.raw_command)
            (has_output Controldev::SteeringWheelTask Controldev::SteeringWheelTask.state)
            (has_output AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.floor_position)
            (has_output AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.state)
            (has_output AuvControl::ConstantCommandGroundFollower AuvControl::ConstantCommandGroundFollower.cmd_out)
            (has_output Controldev::RawJoystickToMotion2D Controldev::RawJoystickToMotion2D.motion_command)
            (has_output Controldev::RawJoystickToMotion2D Controldev::RawJoystickToMotion2D.state)
            (has_output Interfaces::LaserRangeFinder Interfaces::LaserRangeFinder.scans)
            (has_output Interfaces::LaserRangeFinder Interfaces::LaserRangeFinder.state)
            (has_output Controldev::SliderboxTask Controldev::SliderboxTask.raw_command)
            (has_output Controldev::SliderboxTask Controldev::SliderboxTask.state)
            (has_output AuvControl::ConstantCommand AuvControl::ConstantCommand.cmd_out)
            (has_output AuvControl::ConstantCommand AuvControl::ConstantCommand.state)
            (has_output ImagePreprocessing::MonoTask ImagePreprocessing::MonoTask.state)
            (has_output ImagePreprocessing::MonoTask ImagePreprocessing::MonoTask.oframe)
            (has_output Interfaces::ActuatorSensorReader Interfaces::ActuatorSensorReader.state)
            (has_output Interfaces::ActuatorCommandWriter Interfaces::ActuatorCommandWriter.state)
            (has_output AuvWaypointNavigator::Task AuvWaypointNavigator::Task.relative_position_command)
            (has_output AuvWaypointNavigator::Task AuvWaypointNavigator::Task.current_delta)
            (has_output AuvWaypointNavigator::Task AuvWaypointNavigator::Task.current_waypoint)
            (has_output AuvWaypointNavigator::Task AuvWaypointNavigator::Task.queue_size)
            (has_output AuvWaypointNavigator::Task AuvWaypointNavigator::Task.state)
            (has_output RearSonarDistanceEstimator::Task RearSonarDistanceEstimator::Task.ground_distance)
            (has_output RearSonarDistanceEstimator::Task RearSonarDistanceEstimator::Task.state)
            (has_output Taskmon::Task Taskmon::Task.stats)
            (has_output Taskmon::Task Taskmon::Task.state)
            (has_output Dynamixel::Task Dynamixel::Task.upper2lower)
            (has_output Dynamixel::Task Dynamixel::Task.angle)
            (has_output Dynamixel::Task Dynamixel::Task.lowerDynamixel2UpperDynamixel)
            (has_output Dynamixel::Task Dynamixel::Task.state)
            (has_output StructuredLight::Calibration StructuredLight::Calibration.stream_aligner_status)
            (has_output StructuredLight::Calibration StructuredLight::Calibration.state)
            (has_output StructuredLight::Task StructuredLight::Task.laser_scan)
            (has_output StructuredLight::Task StructuredLight::Task.candidates)
            (has_output StructuredLight::Task StructuredLight::Task.debug_frame)
            (has_output StructuredLight::Task StructuredLight::Task.state)
            (has_output CameraUnicap::CameraTask CameraUnicap::CameraTask.frame)
            (has_output CameraUnicap::CameraTask CameraUnicap::CameraTask.state)
            (has_output FrameDemultiplexer::Task FrameDemultiplexer::Task.oframe_pair)
            (has_output FrameDemultiplexer::Task FrameDemultiplexer::Task.oframe)
            (has_output FrameDemultiplexer::Task FrameDemultiplexer::Task.state)
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
