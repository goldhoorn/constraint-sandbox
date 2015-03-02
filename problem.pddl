(define (problem network001)
  (:domain network)
  (:objects
        root - instance_req
        auv_rel_pos_controller::Task.position_command - input_port
        auv_rel_pos_controller::Task.position_sample - input_port
        auv_rel_pos_controller::Task.motion_command - output_port
        auv_rel_pos_controller::Task.state - output_port
        auv_rel_pos_controller::Task - instance_req
        pipeline_inspection::ColorFilter.frame_in - input_port
        pipeline_inspection::ColorFilter.frame_out - output_port
        pipeline_inspection::ColorFilter.green_frame - output_port
        pipeline_inspection::ColorFilter.diff_frame - output_port
        pipeline_inspection::ColorFilter.state - output_port
        pipeline_inspection::ColorFilter - instance_req
        pipeline_inspection::Inspection.laserSamples - input_port
        pipeline_inspection::Inspection.laserPoints - input_port
        pipeline_inspection::Inspection.laserPointCloud - input_port
        pipeline_inspection::Inspection.pipeline - input_port
        pipeline_inspection::Inspection.dead_reckoning - input_port
        pipeline_inspection::Inspection.inspectionStatus - output_port
        pipeline_inspection::Inspection.pipePoints - output_port
        pipeline_inspection::Inspection.debugFrame - output_port
        pipeline_inspection::Inspection.pipeMap - output_port
        pipeline_inspection::Inspection.state - output_port
        pipeline_inspection::Inspection - instance_req
        pipeline_inspection::LaserSimulation.laserPoints - output_port
        pipeline_inspection::LaserSimulation.laserPointCloud - output_port
        pipeline_inspection::LaserSimulation.vehiclePos - output_port
        pipeline_inspection::LaserSimulation.state - output_port
        pipeline_inspection::LaserSimulation - instance_req
        uw_particle_localization::FastFusion.position_samples - input_port
        uw_particle_localization::FastFusion.depth_samples - input_port
        uw_particle_localization::FastFusion.orientation_samples - input_port
        uw_particle_localization::FastFusion.velocity_samples - input_port
        uw_particle_localization::FastFusion.pose_samples - output_port
        uw_particle_localization::FastFusion.state - output_port
        uw_particle_localization::FastFusion - instance_req
        uw_particle_localization::MotionModel.thruster_samples - input_port
        uw_particle_localization::MotionModel.orientation_samples - input_port
        uw_particle_localization::MotionModel.pose_samples - output_port
        uw_particle_localization::MotionModel.stream_aligner_status - output_port
        uw_particle_localization::MotionModel.state - output_port
        uw_particle_localization::MotionModel - instance_req
        uw_particle_localization::OrientationCorrection.orientation_input - input_port
        uw_particle_localization::OrientationCorrection.orientation_offset - input_port
        uw_particle_localization::OrientationCorrection.orientation_output - output_port
        uw_particle_localization::OrientationCorrection.orientation_offset_corrected - output_port
        uw_particle_localization::OrientationCorrection.state - output_port
        uw_particle_localization::OrientationCorrection - instance_req
        uw_particle_localization::Task.laser_samples - input_port
        uw_particle_localization::Task.speed_samples - input_port
        uw_particle_localization::Task.pipeline_samples - input_port
        uw_particle_localization::Task.pose_update - input_port
        uw_particle_localization::Task.gps_pose_samples - input_port
        uw_particle_localization::Task.buoy_samples_orange - input_port
        uw_particle_localization::Task.buoy_samples_white - input_port
        uw_particle_localization::Task.thruster_samples - input_port
        uw_particle_localization::Task.orientation_samples - input_port
        uw_particle_localization::Task.echosounder_samples - input_port
        uw_particle_localization::Task.obstacle_samples - input_port
        uw_particle_localization::Task.structur_samples - input_port
        uw_particle_localization::Task.pose_samples - output_port
        uw_particle_localization::Task.environment - output_port
        uw_particle_localization::Task.dead_reckoning_samples - output_port
        uw_particle_localization::Task.full_dead_reckoning - output_port
        uw_particle_localization::Task.particles - output_port
        uw_particle_localization::Task.debug_sonar_beam - output_port
        uw_particle_localization::Task.stats - output_port
        uw_particle_localization::Task.depth_grid - output_port
        uw_particle_localization::Task.grid_map - output_port
        uw_particle_localization::Task.debug_filtered_obstacles - output_port
        uw_particle_localization::Task.stream_aligner_status - output_port
        uw_particle_localization::Task.state - output_port
        uw_particle_localization::Task - instance_req
        gps::BaseTask.solution - output_port
        gps::BaseTask.position_samples - output_port
        gps::BaseTask.state - output_port
        gps::BaseTask - instance_req
        gps::GPSDTask - instance_req
        gps::MB500Task.constellation - output_port
        gps::MB500Task.time - output_port
        gps::MB500Task - instance_req
        line_scanner::Task.frame - input_port
        line_scanner::Task.state - output_port
        line_scanner::Task.pointcloud - output_port
        line_scanner::Task.debug - output_port
        line_scanner::Task - instance_req
        message_producer::Task.messages - output_port
        message_producer::Task.state - output_port
        message_producer::Task - instance_req
        buoy::Detector.frame - input_port
        buoy::Detector.state - output_port
        buoy::Detector.buoy - output_port
        buoy::Detector.light - output_port
        buoy::Detector.h_image - output_port
        buoy::Detector.s_image - output_port
        buoy::Detector.v_image - output_port
        buoy::Detector.binary_debug_image - output_port
        buoy::Detector.gray_debug_image - output_port
        buoy::Detector.hough_debug_image - output_port
        buoy::Detector.other_buoys - output_port
        buoy::Detector.debug_image - output_port
        buoy::Detector - instance_req
        buoy::Detector2.frame - input_port
        buoy::Detector2.state - output_port
        buoy::Detector2.buoy - output_port
        buoy::Detector2.light - output_port
        buoy::Detector2.h_image - output_port
        buoy::Detector2.s_image - output_port
        buoy::Detector2.v_image - output_port
        buoy::Detector2.binary_debug_image - output_port
        buoy::Detector2.gray_debug_image - output_port
        buoy::Detector2.hough_debug_image - output_port
        buoy::Detector2.other_buoys - output_port
        buoy::Detector2.debug_image - output_port
        buoy::Detector2 - instance_req
        buoy::ServoingOnWall.buoy_samples - input_port
        buoy::ServoingOnWall.wall_samples - input_port
        buoy::ServoingOnWall.orientation_samples - input_port
        buoy::ServoingOnWall.world_cmd - output_port
        buoy::ServoingOnWall.aligned_position_cmd - output_port
        buoy::ServoingOnWall.state - output_port
        buoy::ServoingOnWall - instance_req
        buoy::Survey.orientation_samples - input_port
        buoy::Survey.force_cutting - input_port
        buoy::Survey.input_buoy - input_port
        buoy::Survey.motion_command - input_port
        buoy::Survey.light - input_port
        buoy::Survey.target_angle_input - input_port
        buoy::Survey.state - output_port
        buoy::Survey.strafed_angle - output_port
        buoy::Survey.relative_position - output_port
        buoy::Survey.position - output_port
        buoy::Survey - instance_req
        mars::Plugin.state - output_port
        mars::Altimeter.ground_distance - output_port
        mars::Altimeter - instance_req
        mars::AuvController - instance_req
        mars::ForceApplier.command - input_port
        mars::ForceApplier.status - output_port
        mars::AuvMotion - instance_req
        mars::Camera.frame - output_port
        mars::Camera - instance_req
        mars::CameraPlugin - instance_req
        mars::DepthCamera.distance_image - output_port
        mars::DepthCamera - instance_req
        mars::ForceApplier - instance_req
        mars::ForceTorque6DOF.wrenches - output_port
        mars::ForceTorque6DOF - instance_req
        mars::HighResRangeFinder.pointcloud - output_port
        mars::HighResRangeFinder - instance_req
        mars::IMU.orientation_samples - output_port
        mars::IMU.calibrated_sensors - output_port
        mars::IMU.pose_samples - output_port
        mars::IMU - instance_req
        mars::Joints.command - input_port
        mars::Joints.status_samples - output_port
        mars::Joints.current_values - output_port
        mars::Joints.transforms - output_port
        mars::Joints - instance_req
        mars::LaserRangeFinder.scans - output_port
        mars::LaserRangeFinder - instance_req
        mars::Plugin - instance_req
        mars::RotatingLaserRangeFinder.pointcloud - output_port
        mars::RotatingLaserRangeFinder - instance_req
        mars::Sonar.sonar_beam - output_port
        mars::Sonar - instance_req
        mars::Task.control_action - input_port
        mars::Task.time - output_port
        mars::Task.simulated_time - output_port
        mars::Task.state - output_port
        mars::Task - instance_req
        mars::Trigger - instance_req
        sonar_feature_estimator::Task.sonar_input - input_port
        sonar_feature_estimator::Task.orientation_sample - input_port
        sonar_feature_estimator::Task.new_feature - output_port
        sonar_feature_estimator::Task.features_out - output_port
        sonar_feature_estimator::Task.debug_output - output_port
        sonar_feature_estimator::Task.2d_debug_output - output_port
        sonar_feature_estimator::Task.state - output_port
        sonar_feature_estimator::Task - instance_req
        modem_can::Task.modem_in - input_port
        modem_can::Task.canModem - input_port
        modem_can::Task.light_value - input_port
        modem_can::Task.position_samples - input_port
        modem_can::Task.modem_out - output_port
        modem_can::Task.canOut - output_port
        modem_can::Task.motion_command - output_port
        modem_can::Task.state - output_port
        modem_can::Task - instance_req
        taskmon::Task.stats - output_port
        taskmon::Task.state - output_port
        taskmon::Task - instance_req
        depth_reader::Task.canIn - input_port
        depth_reader::Task.depthOut - output_port
        depth_reader::Task.depth_samples - output_port
        depth_reader::Task.canOut - output_port
        depth_reader::Task.state - output_port
        depth_reader::Task - instance_req
        raw_control_command_converter::Movement.raw_command - input_port
        raw_control_command_converter::Movement.orientation_readings - input_port
        raw_control_command_converter::Movement.ground_distance - input_port
        raw_control_command_converter::Movement.motion_command - output_port
        raw_control_command_converter::Movement.world_command - output_port
        raw_control_command_converter::Movement.world_command_depth - output_port
        raw_control_command_converter::Movement.aligned_velocity_command - output_port
        raw_control_command_converter::Movement.state - output_port
        raw_control_command_converter::Movement - instance_req
        raw_control_command_converter::Position.raw_command - input_port
        raw_control_command_converter::Position.pose_samples - input_port
        raw_control_command_converter::Position.position_command - output_port
        raw_control_command_converter::Position.world_command - output_port
        raw_control_command_converter::Position.state - output_port
        raw_control_command_converter::Position - instance_req
        sonar_feature_detector::Task.grid_maps - input_port
        sonar_feature_detector::Task.pose_samples - input_port
        sonar_feature_detector::Task.features - output_port
        sonar_feature_detector::Task.next_target - output_port
        sonar_feature_detector::Task.next_target_feature - output_port
        sonar_feature_detector::Task.next_target_command - output_port
        sonar_feature_detector::Task.state - output_port
        sonar_feature_detector::Task - instance_req
        frame_demultiplexer::Task.iframe - input_port
        frame_demultiplexer::Task.oframe_pair - output_port
        frame_demultiplexer::Task.oframe - output_port
        frame_demultiplexer::Task.state - output_port
        frame_demultiplexer::Task - instance_req
        sonar_tritech::Echosounder.ground_distance - output_port
        sonar_tritech::Echosounder.state - output_port
        sonar_tritech::Echosounder - instance_req
        sonar_tritech::Micron.ground_distance - output_port
        sonar_tritech::Micron.sonar_beam - output_port
        sonar_tritech::Micron.state - output_port
        sonar_tritech::Micron - instance_req
        sonar_tritech::Profiling.profiling_scan - output_port
        sonar_tritech::Profiling.state - output_port
        sonar_tritech::Profiling - instance_req
        canbus::InterfaceTask.can_in - input_port
        canbus::InterfaceTask.can_out - output_port
        canbus::InterfaceTask.state - output_port
        battery_watcher::Task.battery_info - output_port
        battery_watcher::Task - instance_req
        orientation_estimator::BaseEstimator.imu_orientation - input_port
        orientation_estimator::BaseEstimator.fog_samples - input_port
        orientation_estimator::BaseEstimator.heading_correction - input_port
        orientation_estimator::BaseEstimator.stream_aligner_status - output_port
        orientation_estimator::BaseEstimator.attitude_b_g - output_port
        orientation_estimator::BaseEstimator.state - output_port
        orientation_estimator::BaseEstimator - instance_req
        orientation_estimator::IKF.imu_samples - input_port
        orientation_estimator::IKF.fog_samples - input_port
        orientation_estimator::IKF.initial_orientation - input_port
        orientation_estimator::IKF.dynamic_transformations - input_port
        orientation_estimator::IKF.transformer_stream_aligner_status - output_port
        orientation_estimator::IKF.transformer_status - output_port
        orientation_estimator::IKF.attitude_b_g - output_port
        orientation_estimator::IKF.state - output_port
        orientation_estimator::IKF - instance_req
        orientation_estimator::IKFEstimator.imu_orientation - input_port
        orientation_estimator::IKFEstimator.fog_samples - input_port
        orientation_estimator::IKFEstimator.imu_samples - input_port
        orientation_estimator::IKFEstimator.stream_aligner_status - output_port
        orientation_estimator::IKFEstimator.attitude_b_g - output_port
        orientation_estimator::IKFEstimator.inputs_backup - output_port
        orientation_estimator::IKFEstimator.state - output_port
        orientation_estimator::IKFEstimator - instance_req
        orientation_estimator::UKFEstimator.imu_orientation - input_port
        orientation_estimator::UKFEstimator.fog_samples - input_port
        orientation_estimator::UKFEstimator.imu_samples - input_port
        orientation_estimator::UKFEstimator.stream_aligner_status - output_port
        orientation_estimator::UKFEstimator.attitude_b_g - output_port
        orientation_estimator::UKFEstimator.state - output_port
        orientation_estimator::UKFEstimator - instance_req
        hsv_mosaicing::Task.frame - input_port
        hsv_mosaicing::Task.result - output_port
        hsv_mosaicing::Task.state - output_port
        hsv_mosaicing::Task - instance_req
        sonar_wall_hough::Task.sonar_samples - input_port
        sonar_wall_hough::Task.reset - input_port
        sonar_wall_hough::Task.orientation_samples - input_port
        sonar_wall_hough::Task.pose_samples - input_port
        sonar_wall_hough::Task.lines - output_port
        sonar_wall_hough::Task.peaks - output_port
        sonar_wall_hough::Task.houghspace - output_port
        sonar_wall_hough::Task.position - output_port
        sonar_wall_hough::Task.position_quality - output_port
        sonar_wall_hough::Task.state - output_port
        sonar_wall_hough::Task - instance_req
        pose_estimation::BaseTask.pose_samples - output_port
        pose_estimation::BaseTask.state - output_port
        pose_estimation::BaseTask - instance_req
        pose_estimation::UWPoseEstimator.orientation_samples - input_port
        pose_estimation::UWPoseEstimator.depth_samples - input_port
        pose_estimation::UWPoseEstimator.dvl_velocity_samples - input_port
        pose_estimation::UWPoseEstimator.model_velocity_samples - input_port
        pose_estimation::UWPoseEstimator.lbl_position_samples - input_port
        pose_estimation::UWPoseEstimator.xy_position_samples - input_port
        pose_estimation::UWPoseEstimator.dynamic_transformations - input_port
        pose_estimation::UWPoseEstimator.transformer_stream_aligner_status - output_port
        pose_estimation::UWPoseEstimator.transformer_status - output_port
        pose_estimation::UWPoseEstimator - instance_req
        pose_estimation::VehiclePoseEstimator.orientation_samples - input_port
        pose_estimation::VehiclePoseEstimator.velocity_samples - input_port
        pose_estimation::VehiclePoseEstimator.position_samples - input_port
        pose_estimation::VehiclePoseEstimator.dynamic_transformations - input_port
        pose_estimation::VehiclePoseEstimator.transformer_stream_aligner_status - output_port
        pose_estimation::VehiclePoseEstimator.transformer_status - output_port
        pose_estimation::VehiclePoseEstimator - instance_req
        auv_waypoint_navigator::Task.trajectory - input_port
        auv_waypoint_navigator::Task.pose_samples - input_port
        auv_waypoint_navigator::Task.relative_position_command - output_port
        auv_waypoint_navigator::Task.current_delta - output_port
        auv_waypoint_navigator::Task.current_waypoint - output_port
        auv_waypoint_navigator::Task.queue_size - output_port
        auv_waypoint_navigator::Task.state - output_port
        auv_waypoint_navigator::Task - instance_req
        camera_base::Task.frame - output_port
        camera_base::Task.frame_raw - output_port
        camera_base::Task.state - output_port
        camera_aravis::Task - instance_req
        gps_helper::GPSFaker.position_samples - output_port
        gps_helper::GPSFaker.state - output_port
        gps_helper::GPSFaker - instance_req
        gps_helper::MapToGPS.position_samples - input_port
        gps_helper::MapToGPS.dynamic_transformations - input_port
        gps_helper::MapToGPS.gps_position - output_port
        gps_helper::MapToGPS.transformer_stream_aligner_status - output_port
        gps_helper::MapToGPS.transformer_status - output_port
        gps_helper::MapToGPS.state - output_port
        gps_helper::MapToGPS - instance_req
        gps_helper::WaypointNavigation.gps_position_samples - input_port
        gps_helper::WaypointNavigation.pose_samples - input_port
        gps_helper::WaypointNavigation.target_waypoint - output_port
        gps_helper::WaypointNavigation.heading_offset - output_port
        gps_helper::WaypointNavigation.distance_delta - output_port
        gps_helper::WaypointNavigation.state - output_port
        gps_helper::WaypointNavigation - instance_req
        avalon_simulation::AsvNavigation.state - output_port
        avalon_simulation::AsvNavigation - instance_req
        avalon_simulation::LineLaser - instance_req
        avalon_simulation::Modem.light_value - input_port
        avalon_simulation::Modem.position_samples - input_port
        avalon_simulation::Modem.motion_command - output_port
        avalon_simulation::Modem.state - output_port
        avalon_simulation::Modem - instance_req
        avalon_simulation::WhiteLight.state - output_port
        avalon_simulation::WhiteLight - instance_req
        avalon_control::FakeWriter.motion_commands - output_port
        avalon_control::FakeWriter.state - output_port
        avalon_control::FakeWriter - instance_req
        avalon_control::MotionControlTask.dummy_feedback - input_port
        avalon_control::MotionControlTask.ground_distance - input_port
        avalon_control::MotionControlTask.pose_samples - input_port
        avalon_control::MotionControlTask.motion_commands - input_port
        avalon_control::MotionControlTask.joints_in - input_port
        avalon_control::MotionControlTask.hbridge_commands - output_port
        avalon_control::MotionControlTask.joint_commands - output_port
        avalon_control::MotionControlTask.debug - output_port
        avalon_control::MotionControlTask.estimated_ground_pos - output_port
        avalon_control::MotionControlTask.state - output_port
        avalon_control::MotionControlTask - instance_req
        avalon_control::MotionFeedbackTask.hbridge_feedback - input_port
        avalon_control::MotionFeedbackTask.hbridge_status - output_port
        avalon_control::MotionFeedbackTask.state - output_port
        avalon_control::MotionFeedbackTask - instance_req
        avalon_control::PositionControlTask.pose_samples - input_port
        avalon_control::PositionControlTask.position_commands - input_port
        avalon_control::PositionControlTask.motion_commands - output_port
        avalon_control::PositionControlTask.state - output_port
        avalon_control::PositionControlTask - instance_req
        avalon_control::RelFakeWriter.position_command - output_port
        avalon_control::RelFakeWriter.state - output_port
        avalon_control::RelFakeWriter - instance_req
        avalon_control::TrajectoryFollower.pose_samples - input_port
        avalon_control::TrajectoryFollower.next_position - output_port
        avalon_control::TrajectoryFollower.position_command - output_port
        avalon_control::TrajectoryFollower.next_pos_on_spline - output_port
        avalon_control::TrajectoryFollower.last_pos_on_spline - output_port
        avalon_control::TrajectoryFollower.segment_dist - output_port
        avalon_control::TrajectoryFollower.world_command - output_port
        avalon_control::TrajectoryFollower.state - output_port
        avalon_control::TrajectoryFollower - instance_req
        hbridge::CommandWriter.can_in - input_port
        hbridge::CommandWriter.command - input_port
        hbridge::CommandWriter.state - output_port
        hbridge::CommandWriter.can_out - output_port
        hbridge::CommandWriter.speedCtrlDebug - output_port
        hbridge::CommandWriter.fakeReader - output_port
        hbridge::CommandWriter - instance_req
        hbridge::SensorReader.can_in - input_port
        hbridge::SensorReader.state - output_port
        hbridge::SensorReader.can_out - output_port
        hbridge::SensorReader.status_samples - output_port
        hbridge::SensorReader - instance_req
        sonar_structure_servoing::Task.sonarbeam_feature - input_port
        sonar_structure_servoing::Task.odometry_samples - input_port
        sonar_structure_servoing::Task.dynamic_transformations - input_port
        sonar_structure_servoing::Task.position_command - output_port
        sonar_structure_servoing::Task.aligned_position_command - output_port
        sonar_structure_servoing::Task.world_command - output_port
        sonar_structure_servoing::Task.debug_data - output_port
        sonar_structure_servoing::Task.transformer_stream_aligner_status - output_port
        sonar_structure_servoing::Task.transformer_status - output_port
        sonar_structure_servoing::Task.state - output_port
        sonar_structure_servoing::Task - instance_req
        depth_map::LaserToPoints.laser_scan - input_port
        depth_map::LaserToPoints.rbs - input_port
        depth_map::LaserToPoints.pointcloud - output_port
        depth_map::LaserToPoints.state - output_port
        depth_map::LaserToPoints - instance_req
        depth_map::Task.input - input_port
        depth_map::Task.output - output_port
        depth_map::Task.state - output_port
        depth_map::Task - instance_req
        depth_map::frame_export.pointcloud - input_port
        depth_map::frame_export.frames - output_port
        depth_map::frame_export.minZ - output_port
        depth_map::frame_export.maxZ - output_port
        depth_map::frame_export.state - output_port
        depth_map::frame_export - instance_req
        depth_map::sonar_ground_distance.sonar_input - input_port
        depth_map::sonar_ground_distance.new_feature - output_port
        depth_map::sonar_ground_distance.state - output_port
        depth_map::sonar_ground_distance - instance_req
        wall_servoing::DualSonarServoing.sonarbeam_feature_front - input_port
        wall_servoing::DualSonarServoing.sonarbeam_feature_rear - input_port
        wall_servoing::DualSonarServoing.orientation_sample - input_port
        wall_servoing::SonarServoing.state - output_port
        wall_servoing::DualSonarServoing.position_command - output_port
        wall_servoing::DualSonarServoing.aligned_command - output_port
        wall_servoing::DualSonarServoing.wall_servoing_debug - output_port
        wall_servoing::DualSonarServoing - instance_req
        wall_servoing::SingleSonarServoing.sonarbeam_feature - input_port
        wall_servoing::SingleSonarServoing.orientation_sample - input_port
        wall_servoing::SingleSonarServoing.position_sample - input_port
        wall_servoing::SingleSonarServoing.position_command - output_port
        wall_servoing::SingleSonarServoing.aligned_position_command - output_port
        wall_servoing::SingleSonarServoing.world_command - output_port
        wall_servoing::SingleSonarServoing.wall_servoing_debug - output_port
        wall_servoing::SingleSonarServoing.wall - output_port
        wall_servoing::SingleSonarServoing - instance_req
        wall_servoing::SonarServoing - instance_req
        wall_servoing::WallDetector.sonarbeam_feature - input_port
        wall_servoing::WallDetector.orientation_sample - input_port
        wall_servoing::WallDetector.position_sample - input_port
        wall_servoing::WallDetector.point_cloud - output_port
        wall_servoing::WallDetector.wall - output_port
        wall_servoing::WallDetector.state - output_port
        wall_servoing::WallDetector - instance_req
        wall_servoing::WallServoing.orientation_sample - input_port
        wall_servoing::WallServoing.servoing_wall - input_port
        wall_servoing::WallServoing.obstacle_wall - input_port
        wall_servoing::WallServoing.motion_command - output_port
        wall_servoing::WallServoing.world_command - output_port
        wall_servoing::WallServoing.aligned_velocity_command - output_port
        wall_servoing::WallServoing.state - output_port
        wall_servoing::WallServoing - instance_req
        camera_unicap::CameraTask.frame - output_port
        camera_unicap::CameraTask.state - output_port
        camera_unicap::CameraTask - instance_req
        task_scheduler::Task.trigger - input_port
        task_scheduler::Task.state - output_port
        task_scheduler::Task - instance_req
        sonar_blueview::Task.frame - output_port
        sonar_blueview::Task.state - output_port
        sonar_blueview::Task - instance_req
        auv_helper::depth_and_orientation_fusion.orientation_samples - input_port
        auv_helper::depth_and_orientation_fusion.depth_samples - input_port
        auv_helper::depth_and_orientation_fusion.ground_distance - input_port
        auv_helper::depth_and_orientation_fusion.pose_samples - output_port
        auv_helper::depth_and_orientation_fusion.stream_aligner_status - output_port
        auv_helper::depth_and_orientation_fusion.state - output_port
        auv_helper::depth_and_orientation_fusion - instance_req
        sysmon::Task.can_in_system_status - input_port
        sysmon::Task.can_in_experiment_markers - input_port
        sysmon::Task.in_experiment_markers - input_port
        sysmon::Task.in_modem_substates - input_port
        sysmon::Task.ocu_markers - output_port
        sysmon::Task.annotations - output_port
        sysmon::Task.system_status - output_port
        sysmon::Task.state - output_port
        sysmon::Task - instance_req
        logger::Logger.state - output_port
        logger::Logger - instance_req
        fog_kvh::Dsp3000Task.config - input_port
        fog_kvh::Dsp3000Task.rotation - output_port
        fog_kvh::Dsp3000Task.orientation_samples - output_port
        fog_kvh::Dsp3000Task.timestamp_estimator_status - output_port
        fog_kvh::Dsp3000Task.state - output_port
        fog_kvh::Dsp3000Task - instance_req
        rear_sonar_distance_estimator::Task.BaseScan - input_port
        rear_sonar_distance_estimator::Task.depth_samples - input_port
        rear_sonar_distance_estimator::Task.ground_distance - output_port
        rear_sonar_distance_estimator::Task.state - output_port
        rear_sonar_distance_estimator::Task - instance_req
        transformer::Task.configuration_state - output_port
        transformer::Task.state - output_port
        transformer::Task - instance_req
        camera_base::Task - instance_req
        low_level_driver::LowLevelTask.depth_samples - input_port
        low_level_driver::LowLevelTask.ShortExposure - input_port
        low_level_driver::LowLevelTask.LongExposure - input_port
        low_level_driver::LowLevelTask.LightValue - input_port
        low_level_driver::LowLevelTask.DebugLED - input_port
        low_level_driver::LowLevelTask.LaserRate - input_port
        low_level_driver::LowLevelTask.state - output_port
        low_level_driver::LowLevelTask - instance_req
        wall_orientation_correction::OrientationInMap.orientation_in_world - input_port
        wall_orientation_correction::OrientationInMap.dynamic_transformations - input_port
        wall_orientation_correction::OrientationInMap.orientation_in_map - output_port
        wall_orientation_correction::OrientationInMap.transformer_stream_aligner_status - output_port
        wall_orientation_correction::OrientationInMap.transformer_status - output_port
        wall_orientation_correction::OrientationInMap.state - output_port
        wall_orientation_correction::OrientationInMap - instance_req
        wall_orientation_correction::Task.sonarbeam_feature - input_port
        wall_orientation_correction::Task.orientation_samples - input_port
        wall_orientation_correction::Task.dynamic_transformations - input_port
        wall_orientation_correction::Task.orientation_in_world - output_port
        wall_orientation_correction::Task.angle_in_world - output_port
        wall_orientation_correction::Task.debug_data - output_port
        wall_orientation_correction::Task.transformer_stream_aligner_status - output_port
        wall_orientation_correction::Task.transformer_status - output_port
        wall_orientation_correction::Task.state - output_port
        wall_orientation_correction::Task - instance_req
        structure_servoing::Alignment.input - input_port
        structure_servoing::Alignment.world_command - output_port
        structure_servoing::Alignment.aligned_speed_command - output_port
        structure_servoing::Alignment.left - output_port
        structure_servoing::Alignment.right - output_port
        structure_servoing::Alignment.top - output_port
        structure_servoing::Alignment.bottom - output_port
        structure_servoing::Alignment.size - output_port
        structure_servoing::Alignment.x - output_port
        structure_servoing::Alignment.y - output_port
        structure_servoing::Alignment.state - output_port
        structure_servoing::Alignment - instance_req
        structure_servoing::Task.rbs - input_port
        structure_servoing::Task.input - input_port
        structure_servoing::Task.servoed_angle - output_port
        structure_servoing::Task.angle_speed - output_port
        structure_servoing::Task.world_command - output_port
        structure_servoing::Task.aligned_speed_command - output_port
        structure_servoing::Task.found_structure - output_port
        structure_servoing::Task.state - output_port
        structure_servoing::Task.left - output_port
        structure_servoing::Task.right - output_port
        structure_servoing::Task.top - output_port
        structure_servoing::Task.bottom - output_port
        structure_servoing::Task.size - output_port
        structure_servoing::Task.heading - output_port
        structure_servoing::Task.cnt_left - output_port
        structure_servoing::Task.cnt_right - output_port
        structure_servoing::Task.cnt_top - output_port
        structure_servoing::Task.cnt_bottom - output_port
        structure_servoing::Task - instance_req
        image_preprocessing::BaseTask.state - output_port
        image_preprocessing::BaseTask - instance_req
        image_preprocessing::DepthImage2Pointcloud.color_frame - input_port
        image_preprocessing::DepthImage2Pointcloud.frame - input_port
        image_preprocessing::DepthImage2Pointcloud.pointcloud - output_port
        image_preprocessing::DepthImage2Pointcloud.stream_aligner_status - output_port
        image_preprocessing::DepthImage2Pointcloud.state - output_port
        image_preprocessing::DepthImage2Pointcloud - instance_req
        image_preprocessing::HSVSegmentationAndBlur.frame - input_port
        image_preprocessing::HSVSegmentationAndBlur.oframe - output_port
        image_preprocessing::HSVSegmentationAndBlur.binary_result - output_port
        image_preprocessing::HSVSegmentationAndBlur.hDebug - output_port
        image_preprocessing::HSVSegmentationAndBlur.hDebugGray - output_port
        image_preprocessing::HSVSegmentationAndBlur.vDebug - output_port
        image_preprocessing::HSVSegmentationAndBlur.vDebugGray - output_port
        image_preprocessing::HSVSegmentationAndBlur.sDebug - output_port
        image_preprocessing::HSVSegmentationAndBlur.sDebugGray - output_port
        image_preprocessing::HSVSegmentationAndBlur.hsv_v_frame - output_port
        image_preprocessing::HSVSegmentationAndBlur.state - output_port
        image_preprocessing::HSVSegmentationAndBlur - instance_req
        image_preprocessing::MonoTask.frame - input_port
        image_preprocessing::MonoTask.oframe - output_port
        image_preprocessing::MonoTask - instance_req
        image_preprocessing::StereoTask.frame_left - input_port
        image_preprocessing::StereoTask.frame_right - input_port
        image_preprocessing::StereoTask.oframe_pair - output_port
        image_preprocessing::StereoTask - instance_req
        offshore_pipeline_detector::SonarDetector.sonar_beam - input_port
        offshore_pipeline_detector::SonarDetector.frame - output_port
        offshore_pipeline_detector::SonarDetector.state - output_port
        offshore_pipeline_detector::SonarDetector - instance_req
        offshore_pipeline_detector::Task.frame - input_port
        offshore_pipeline_detector::Task.orientation_sample - input_port
        offshore_pipeline_detector::Task.altitude_samples - input_port
        offshore_pipeline_detector::Task.state - output_port
        offshore_pipeline_detector::Task.pipeline - output_port
        offshore_pipeline_detector::Task.world_command - output_port
        offshore_pipeline_detector::Task.aligned_position_command - output_port
        offshore_pipeline_detector::Task.position_command - output_port
        offshore_pipeline_detector::Task.debug - output_port
        offshore_pipeline_detector::Task.debug_frame - output_port
        offshore_pipeline_detector::Task - instance_req
        interfaces::ActuatorCommandWriter.state - output_port
        interfaces::ActuatorCommandWriter - instance_req
        interfaces::ActuatorSensorReader.state - output_port
        interfaces::ActuatorSensorReader - instance_req
        interfaces::IMU.orientation_samples - output_port
        interfaces::IMU.calibrated_sensors - output_port
        interfaces::IMU.state - output_port
        interfaces::IMU - instance_req
        interfaces::LaserRangeFinder.scans - output_port
        interfaces::LaserRangeFinder.state - output_port
        interfaces::LaserRangeFinder - instance_req
        interfaces::Servo.cmd_angle - input_port
        interfaces::Servo.upper2lower - output_port
        interfaces::Servo.angle - output_port
        interfaces::Servo.state - output_port
        interfaces::Servo - instance_req
        canbus::InterfaceTask - instance_req
        canbus::Task.in - input_port
        canbus::Task.stats - output_port
        canbus::Task.can_status - output_port
        canbus::Task.log_message - output_port
        canbus::Task.state - output_port
        canbus::Task - instance_req
        structured_light::Calibration.laser_scan - input_port
        structured_light::Calibration.calibration - input_port
        structured_light::Calibration.stream_aligner_status - output_port
        structured_light::Calibration.state - output_port
        structured_light::Calibration - instance_req
        structured_light::Task.frame_pair - input_port
        structured_light::Task.frame - input_port
        structured_light::Task.laser_scan - output_port
        structured_light::Task.candidates - output_port
        structured_light::Task.debug_frame - output_port
        structured_light::Task.state - output_port
        structured_light::Task - instance_req
        xsens_imu::Task.hard_timestamps - input_port
        xsens_imu::Task.orientation_samples - output_port
        xsens_imu::Task.calibrated_sensors - output_port
        xsens_imu::Task.timestamp_estimator_status - output_port
        xsens_imu::Task.state - output_port
        xsens_imu::Task - instance_req
        camera_prosilica::Task - instance_req
        dynamixel::Task.lowerDynamixel2UpperDynamixel - output_port
        dynamixel::Task - instance_req
        pddl_planner::Task.state - output_port
        pddl_planner::Task - instance_req
        video_streamer_vlc::Capturer.state - output_port
        video_streamer_vlc::Capturer - instance_req
        video_streamer_vlc::Streamer.state - output_port
        video_streamer_vlc::Streamer - instance_req
        controldev::GenericRawToMotion2D.raw_command - input_port
        controldev::GenericRawToMotion2D.motion_command - output_port
        controldev::GenericRawToMotion2D.state - output_port
        controldev::GenericRawToMotion2D - instance_req
        controldev::GenericTask.raw_command - output_port
        controldev::GenericTask.state - output_port
        controldev::GenericTask - instance_req
        controldev::JoyPadTask - instance_req
        controldev::JoystickTask - instance_req
        controldev::Mouse3DTask - instance_req
        controldev::RawJoystickToMotion2D.raw_command - input_port
        controldev::RawJoystickToMotion2D.motion_command - output_port
        controldev::RawJoystickToMotion2D.state - output_port
        controldev::RawJoystickToMotion2D - instance_req
        controldev::RawWheelToMotion2D.raw_command - input_port
        controldev::RawWheelToMotion2D.motion_command - output_port
        controldev::RawWheelToMotion2D.state - output_port
        controldev::RawWheelToMotion2D - instance_req
        controldev::Remote.canInputDevice - input_port
        controldev::Remote - instance_req
        controldev::SliderboxTask - instance_req
        controldev::SteeringWheelTask - instance_req
        structure_reconstruction::Task.front_camera - input_port
        structure_reconstruction::Task.bottom_camera - input_port
        structure_reconstruction::Task.dynamic_transformations - input_port
        structure_reconstruction::Task.transformer_stream_aligner_status - output_port
        structure_reconstruction::Task.transformer_status - output_port
        structure_reconstruction::Task.state - output_port
        structure_reconstruction::Task - instance_req
        lights::Lights.int_in - input_port
        lights::Lights.can_in - input_port
        lights::Lights.light_value - output_port
        lights::Lights.state - output_port
        lights::Lights - instance_req
        auv_control::Base.cmd_in - input_port
        auv_control::Base.cmd_cascade - input_port
        auv_control::Base.state - output_port
        auv_control::AccelerationController.cmd_out - output_port
        auv_control::AccelerationController - instance_req
        auv_control::AlignedToBody.orientation_samples - input_port
        auv_control::AlignedToBody.cmd_out - output_port
        auv_control::AlignedToBody - instance_req
        auv_control::Base - instance_req
        auv_control::BasePIDController.pose_samples - input_port
        auv_control::BasePIDController.cmd_out - output_port
        auv_control::BasePIDController.pid_state - output_port
        auv_control::BasePIDController - instance_req
        auv_control::ConstantCommand.cmd_out - output_port
        auv_control::ConstantCommand.state - output_port
        auv_control::ConstantCommand - instance_req
        auv_control::ConstantCommandGroundFollower.altimeter - input_port
        auv_control::ConstantCommandGroundFollower.depth - input_port
        auv_control::ConstantCommandGroundFollower.cmd_in - input_port
        auv_control::ConstantCommandGroundFollower.floor_position - output_port
        auv_control::ConstantCommandGroundFollower.state - output_port
        auv_control::ConstantCommandGroundFollower.cmd_out - output_port
        auv_control::ConstantCommandGroundAvoidance - instance_req
        auv_control::ConstantCommandGroundFollower - instance_req
        auv_control::MotionCommand2DConverter.cmd_in - input_port
        auv_control::MotionCommand2DConverter.cmd_out - output_port
        auv_control::MotionCommand2DConverter.state - output_port
        auv_control::MotionCommand2DConverter - instance_req
        auv_control::OptimalHeadingController.orientation_samples - input_port
        auv_control::OptimalHeadingController.cmd_out - output_port
        auv_control::OptimalHeadingController - instance_req
        auv_control::PIDController - instance_req
        auv_control::WaypointNavigator.trajectory - input_port
        auv_control::WaypointNavigator.pose_sample - input_port
        auv_control::WaypointNavigator.cmd_out - output_port
        auv_control::WaypointNavigator.waypoint_info - output_port
        auv_control::WaypointNavigator.state - output_port
        auv_control::WaypointNavigator - instance_req
        auv_control::WorldToAligned.pose_samples - input_port
        auv_control::WorldToAligned.cmd_out - output_port
        auv_control::WorldToAligned - instance_req
        modemdriver::Modem.data_in - input_port
        modemdriver::Modem.data_out - output_port
        modemdriver::Modem.distance - output_port
        modemdriver::Modem.out_modem_substates - output_port
        modemdriver::Modem.state - output_port
        modemdriver::Modem - instance_req
        modemdriver::ModemCanbus.can_in - input_port
        modemdriver::ModemCanbus.can_out - output_port
        modemdriver::ModemCanbus.stats - output_port
        modemdriver::ModemCanbus - instance_req
        modemdriver::ModemSerial - instance_req
  )
  (:init

        (task  root)
        (is-running root)
        (task  auv_rel_pos_controller::Task)
        (task  pipeline_inspection::ColorFilter)
        (task  pipeline_inspection::Inspection)
        (task  pipeline_inspection::LaserSimulation)
        (task  uw_particle_localization::FastFusion)
        (task  uw_particle_localization::MotionModel)
        (task  uw_particle_localization::OrientationCorrection)
        (task  uw_particle_localization::Task)
        (task  gps::BaseTask)
        (task  gps::GPSDTask)
        (task  gps::MB500Task)
        (task  line_scanner::Task)
        (task  message_producer::Task)
        (task  buoy::Detector)
        (task  buoy::Detector2)
        (task  buoy::ServoingOnWall)
        (task  buoy::Survey)
        (task  mars::Altimeter)
        (task  mars::AuvController)
        (task  mars::AuvMotion)
        (task  mars::Camera)
        (task  mars::CameraPlugin)
        (task  mars::DepthCamera)
        (task  mars::ForceApplier)
        (task  mars::ForceTorque6DOF)
        (task  mars::HighResRangeFinder)
        (task  mars::IMU)
        (task  mars::Joints)
        (task  mars::LaserRangeFinder)
        (task  mars::Plugin)
        (task  mars::RotatingLaserRangeFinder)
        (task  mars::Sonar)
        (task  mars::Task)
        (task  mars::Trigger)
        (task  sonar_feature_estimator::Task)
        (task  modem_can::Task)
        (task  taskmon::Task)
        (task  depth_reader::Task)
        (task  raw_control_command_converter::Movement)
        (task  raw_control_command_converter::Position)
        (task  sonar_feature_detector::Task)
        (task  frame_demultiplexer::Task)
        (task  sonar_tritech::Echosounder)
        (task  sonar_tritech::Micron)
        (task  sonar_tritech::Profiling)
        (task  battery_watcher::Task)
        (task  orientation_estimator::BaseEstimator)
        (task  orientation_estimator::IKF)
        (task  orientation_estimator::IKFEstimator)
        (task  orientation_estimator::UKFEstimator)
        (task  hsv_mosaicing::Task)
        (task  sonar_wall_hough::Task)
        (task  pose_estimation::BaseTask)
        (task  pose_estimation::UWPoseEstimator)
        (task  pose_estimation::VehiclePoseEstimator)
        (task  auv_waypoint_navigator::Task)
        (task  camera_aravis::Task)
        (task  gps_helper::GPSFaker)
        (task  gps_helper::MapToGPS)
        (task  gps_helper::WaypointNavigation)
        (task  avalon_simulation::AsvNavigation)
        (task  avalon_simulation::LineLaser)
        (task  avalon_simulation::Modem)
        (task  avalon_simulation::WhiteLight)
        (task  avalon_control::FakeWriter)
        (task  avalon_control::MotionControlTask)
        (task  avalon_control::MotionFeedbackTask)
        (task  avalon_control::PositionControlTask)
        (task  avalon_control::RelFakeWriter)
        (task  avalon_control::TrajectoryFollower)
        (task  hbridge::CommandWriter)
        (task  hbridge::SensorReader)
        (task  sonar_structure_servoing::Task)
        (task  depth_map::LaserToPoints)
        (task  depth_map::Task)
        (task  depth_map::frame_export)
        (task  depth_map::sonar_ground_distance)
        (task  wall_servoing::DualSonarServoing)
        (task  wall_servoing::SingleSonarServoing)
        (task  wall_servoing::SonarServoing)
        (task  wall_servoing::WallDetector)
        (task  wall_servoing::WallServoing)
        (task  camera_unicap::CameraTask)
        (task  task_scheduler::Task)
        (task  sonar_blueview::Task)
        (task  auv_helper::depth_and_orientation_fusion)
        (task  sysmon::Task)
        (task  logger::Logger)
        (task  fog_kvh::Dsp3000Task)
        (task  rear_sonar_distance_estimator::Task)
        (task  transformer::Task)
        (task  camera_base::Task)
        (task  low_level_driver::LowLevelTask)
        (task  wall_orientation_correction::OrientationInMap)
        (task  wall_orientation_correction::Task)
        (task  structure_servoing::Alignment)
        (task  structure_servoing::Task)
        (task  image_preprocessing::BaseTask)
        (task  image_preprocessing::DepthImage2Pointcloud)
        (task  image_preprocessing::HSVSegmentationAndBlur)
        (task  image_preprocessing::MonoTask)
        (task  image_preprocessing::StereoTask)
        (task  offshore_pipeline_detector::SonarDetector)
        (task  offshore_pipeline_detector::Task)
        (task  interfaces::ActuatorCommandWriter)
        (task  interfaces::ActuatorSensorReader)
        (task  interfaces::IMU)
        (task  interfaces::LaserRangeFinder)
        (task  interfaces::Servo)
        (task  canbus::InterfaceTask)
        (task  canbus::Task)
        (task  structured_light::Calibration)
        (task  structured_light::Task)
        (task  xsens_imu::Task)
        (task  camera_prosilica::Task)
        (task  dynamixel::Task)
        (task  pddl_planner::Task)
        (task  video_streamer_vlc::Capturer)
        (task  video_streamer_vlc::Streamer)
        (task  controldev::GenericRawToMotion2D)
        (task  controldev::GenericTask)
        (task  controldev::JoyPadTask)
        (task  controldev::JoystickTask)
        (task  controldev::Mouse3DTask)
        (task  controldev::RawJoystickToMotion2D)
        (task  controldev::RawWheelToMotion2D)
        (task  controldev::Remote)
        (task  controldev::SliderboxTask)
        (task  controldev::SteeringWheelTask)
        (task  structure_reconstruction::Task)
        (task  lights::Lights)
        (task  auv_control::AccelerationController)
        (task  auv_control::AlignedToBody)
        (task  auv_control::Base)
        (task  auv_control::BasePIDController)
        (task  auv_control::ConstantCommand)
        (task  auv_control::ConstantCommandGroundAvoidance)
        (task  auv_control::ConstantCommandGroundFollower)
        (task  auv_control::MotionCommand2DConverter)
        (task  auv_control::OptimalHeadingController)
        (task  auv_control::PIDController)
        (task  auv_control::WaypointNavigator)
        (task  auv_control::WorldToAligned)
        (task  modemdriver::Modem)
        (task  modemdriver::ModemCanbus)
        (task  modemdriver::ModemSerial)

            (depends root  root)

            (has_input auv_rel_pos_controller::Task auv_rel_pos_controller::Task.position_command)
            (has_input auv_rel_pos_controller::Task auv_rel_pos_controller::Task.position_sample)
            (has_input pipeline_inspection::ColorFilter pipeline_inspection::ColorFilter.frame_in)
            (has_input pipeline_inspection::Inspection pipeline_inspection::Inspection.laserSamples)
            (has_input pipeline_inspection::Inspection pipeline_inspection::Inspection.laserPoints)
            (has_input pipeline_inspection::Inspection pipeline_inspection::Inspection.laserPointCloud)
            (has_input pipeline_inspection::Inspection pipeline_inspection::Inspection.pipeline)
            (has_input pipeline_inspection::Inspection pipeline_inspection::Inspection.dead_reckoning)
            (has_input uw_particle_localization::FastFusion uw_particle_localization::FastFusion.position_samples)
            (has_input uw_particle_localization::FastFusion uw_particle_localization::FastFusion.depth_samples)
            (has_input uw_particle_localization::FastFusion uw_particle_localization::FastFusion.orientation_samples)
            (has_input uw_particle_localization::FastFusion uw_particle_localization::FastFusion.velocity_samples)
            (has_input uw_particle_localization::MotionModel uw_particle_localization::MotionModel.thruster_samples)
            (has_input uw_particle_localization::MotionModel uw_particle_localization::MotionModel.orientation_samples)
            (has_input uw_particle_localization::OrientationCorrection uw_particle_localization::OrientationCorrection.orientation_input)
            (has_input uw_particle_localization::OrientationCorrection uw_particle_localization::OrientationCorrection.orientation_offset)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.laser_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.speed_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.pipeline_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.pose_update)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.gps_pose_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.buoy_samples_orange)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.buoy_samples_white)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.thruster_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.orientation_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.echosounder_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.obstacle_samples)
            (has_input uw_particle_localization::Task uw_particle_localization::Task.structur_samples)
            (has_input line_scanner::Task line_scanner::Task.frame)
            (has_input buoy::Detector buoy::Detector.frame)
            (has_input buoy::Detector2 buoy::Detector2.frame)
            (has_input buoy::ServoingOnWall buoy::ServoingOnWall.buoy_samples)
            (has_input buoy::ServoingOnWall buoy::ServoingOnWall.wall_samples)
            (has_input buoy::ServoingOnWall buoy::ServoingOnWall.orientation_samples)
            (has_input buoy::Survey buoy::Survey.orientation_samples)
            (has_input buoy::Survey buoy::Survey.force_cutting)
            (has_input buoy::Survey buoy::Survey.input_buoy)
            (has_input buoy::Survey buoy::Survey.motion_command)
            (has_input buoy::Survey buoy::Survey.light)
            (has_input buoy::Survey buoy::Survey.target_angle_input)
            (has_input mars::ForceApplier mars::ForceApplier.command)
            (has_input mars::Joints mars::Joints.command)
            (has_input mars::Task mars::Task.control_action)
            (has_input sonar_feature_estimator::Task sonar_feature_estimator::Task.sonar_input)
            (has_input sonar_feature_estimator::Task sonar_feature_estimator::Task.orientation_sample)
            (has_input modem_can::Task modem_can::Task.modem_in)
            (has_input modem_can::Task modem_can::Task.canModem)
            (has_input modem_can::Task modem_can::Task.light_value)
            (has_input modem_can::Task modem_can::Task.position_samples)
            (has_input depth_reader::Task depth_reader::Task.canIn)
            (has_input raw_control_command_converter::Movement raw_control_command_converter::Movement.raw_command)
            (has_input raw_control_command_converter::Movement raw_control_command_converter::Movement.orientation_readings)
            (has_input raw_control_command_converter::Movement raw_control_command_converter::Movement.ground_distance)
            (has_input raw_control_command_converter::Position raw_control_command_converter::Position.raw_command)
            (has_input raw_control_command_converter::Position raw_control_command_converter::Position.pose_samples)
            (has_input sonar_feature_detector::Task sonar_feature_detector::Task.grid_maps)
            (has_input sonar_feature_detector::Task sonar_feature_detector::Task.pose_samples)
            (has_input frame_demultiplexer::Task frame_demultiplexer::Task.iframe)
            (has_input orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.imu_orientation)
            (has_input orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.fog_samples)
            (has_input orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.heading_correction)
            (has_input orientation_estimator::IKF orientation_estimator::IKF.imu_samples)
            (has_input orientation_estimator::IKF orientation_estimator::IKF.fog_samples)
            (has_input orientation_estimator::IKF orientation_estimator::IKF.initial_orientation)
            (has_input orientation_estimator::IKF orientation_estimator::IKF.dynamic_transformations)
            (has_input orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.imu_orientation)
            (has_input orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.fog_samples)
            (has_input orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.imu_samples)
            (has_input orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.imu_orientation)
            (has_input orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.fog_samples)
            (has_input orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.imu_samples)
            (has_input hsv_mosaicing::Task hsv_mosaicing::Task.frame)
            (has_input sonar_wall_hough::Task sonar_wall_hough::Task.sonar_samples)
            (has_input sonar_wall_hough::Task sonar_wall_hough::Task.reset)
            (has_input sonar_wall_hough::Task sonar_wall_hough::Task.orientation_samples)
            (has_input sonar_wall_hough::Task sonar_wall_hough::Task.pose_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.orientation_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.depth_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.dvl_velocity_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.model_velocity_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.lbl_position_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.xy_position_samples)
            (has_input pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.dynamic_transformations)
            (has_input pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.orientation_samples)
            (has_input pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.velocity_samples)
            (has_input pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.position_samples)
            (has_input pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.dynamic_transformations)
            (has_input auv_waypoint_navigator::Task auv_waypoint_navigator::Task.trajectory)
            (has_input auv_waypoint_navigator::Task auv_waypoint_navigator::Task.pose_samples)
            (has_input gps_helper::MapToGPS gps_helper::MapToGPS.position_samples)
            (has_input gps_helper::MapToGPS gps_helper::MapToGPS.dynamic_transformations)
            (has_input gps_helper::WaypointNavigation gps_helper::WaypointNavigation.gps_position_samples)
            (has_input gps_helper::WaypointNavigation gps_helper::WaypointNavigation.pose_samples)
            (has_input avalon_simulation::Modem avalon_simulation::Modem.light_value)
            (has_input avalon_simulation::Modem avalon_simulation::Modem.position_samples)
            (has_input avalon_control::MotionControlTask avalon_control::MotionControlTask.dummy_feedback)
            (has_input avalon_control::MotionControlTask avalon_control::MotionControlTask.ground_distance)
            (has_input avalon_control::MotionControlTask avalon_control::MotionControlTask.pose_samples)
            (has_input avalon_control::MotionControlTask avalon_control::MotionControlTask.motion_commands)
            (has_input avalon_control::MotionControlTask avalon_control::MotionControlTask.joints_in)
            (has_input avalon_control::MotionFeedbackTask avalon_control::MotionFeedbackTask.hbridge_feedback)
            (has_input avalon_control::PositionControlTask avalon_control::PositionControlTask.pose_samples)
            (has_input avalon_control::PositionControlTask avalon_control::PositionControlTask.position_commands)
            (has_input avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.pose_samples)
            (has_input hbridge::CommandWriter hbridge::CommandWriter.can_in)
            (has_input hbridge::CommandWriter hbridge::CommandWriter.command)
            (has_input hbridge::SensorReader hbridge::SensorReader.can_in)
            (has_input sonar_structure_servoing::Task sonar_structure_servoing::Task.sonarbeam_feature)
            (has_input sonar_structure_servoing::Task sonar_structure_servoing::Task.odometry_samples)
            (has_input sonar_structure_servoing::Task sonar_structure_servoing::Task.dynamic_transformations)
            (has_input depth_map::LaserToPoints depth_map::LaserToPoints.laser_scan)
            (has_input depth_map::LaserToPoints depth_map::LaserToPoints.rbs)
            (has_input depth_map::Task depth_map::Task.input)
            (has_input depth_map::frame_export depth_map::frame_export.pointcloud)
            (has_input depth_map::sonar_ground_distance depth_map::sonar_ground_distance.sonar_input)
            (has_input wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.sonarbeam_feature_front)
            (has_input wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.sonarbeam_feature_rear)
            (has_input wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.orientation_sample)
            (has_input wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.sonarbeam_feature)
            (has_input wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.orientation_sample)
            (has_input wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.position_sample)
            (has_input wall_servoing::WallDetector wall_servoing::WallDetector.sonarbeam_feature)
            (has_input wall_servoing::WallDetector wall_servoing::WallDetector.orientation_sample)
            (has_input wall_servoing::WallDetector wall_servoing::WallDetector.position_sample)
            (has_input wall_servoing::WallServoing wall_servoing::WallServoing.orientation_sample)
            (has_input wall_servoing::WallServoing wall_servoing::WallServoing.servoing_wall)
            (has_input wall_servoing::WallServoing wall_servoing::WallServoing.obstacle_wall)
            (has_input task_scheduler::Task task_scheduler::Task.trigger)
            (has_input auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.orientation_samples)
            (has_input auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.depth_samples)
            (has_input auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.ground_distance)
            (has_input sysmon::Task sysmon::Task.can_in_system_status)
            (has_input sysmon::Task sysmon::Task.can_in_experiment_markers)
            (has_input sysmon::Task sysmon::Task.in_experiment_markers)
            (has_input sysmon::Task sysmon::Task.in_modem_substates)
            (has_input fog_kvh::Dsp3000Task fog_kvh::Dsp3000Task.config)
            (has_input rear_sonar_distance_estimator::Task rear_sonar_distance_estimator::Task.BaseScan)
            (has_input rear_sonar_distance_estimator::Task rear_sonar_distance_estimator::Task.depth_samples)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.depth_samples)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.ShortExposure)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.LongExposure)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.LightValue)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.DebugLED)
            (has_input low_level_driver::LowLevelTask low_level_driver::LowLevelTask.LaserRate)
            (has_input wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.orientation_in_world)
            (has_input wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.dynamic_transformations)
            (has_input wall_orientation_correction::Task wall_orientation_correction::Task.sonarbeam_feature)
            (has_input wall_orientation_correction::Task wall_orientation_correction::Task.orientation_samples)
            (has_input wall_orientation_correction::Task wall_orientation_correction::Task.dynamic_transformations)
            (has_input structure_servoing::Alignment structure_servoing::Alignment.input)
            (has_input structure_servoing::Task structure_servoing::Task.rbs)
            (has_input structure_servoing::Task structure_servoing::Task.input)
            (has_input image_preprocessing::DepthImage2Pointcloud image_preprocessing::DepthImage2Pointcloud.color_frame)
            (has_input image_preprocessing::DepthImage2Pointcloud image_preprocessing::DepthImage2Pointcloud.frame)
            (has_input image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.frame)
            (has_input image_preprocessing::MonoTask image_preprocessing::MonoTask.frame)
            (has_input image_preprocessing::StereoTask image_preprocessing::StereoTask.frame_left)
            (has_input image_preprocessing::StereoTask image_preprocessing::StereoTask.frame_right)
            (has_input offshore_pipeline_detector::SonarDetector offshore_pipeline_detector::SonarDetector.sonar_beam)
            (has_input offshore_pipeline_detector::Task offshore_pipeline_detector::Task.frame)
            (has_input offshore_pipeline_detector::Task offshore_pipeline_detector::Task.orientation_sample)
            (has_input offshore_pipeline_detector::Task offshore_pipeline_detector::Task.altitude_samples)
            (has_input interfaces::Servo interfaces::Servo.cmd_angle)
            (has_input canbus::InterfaceTask canbus::InterfaceTask.can_in)
            (has_input canbus::Task canbus::Task.in)
            (has_input structured_light::Calibration structured_light::Calibration.laser_scan)
            (has_input structured_light::Calibration structured_light::Calibration.calibration)
            (has_input structured_light::Task structured_light::Task.frame_pair)
            (has_input structured_light::Task structured_light::Task.frame)
            (has_input xsens_imu::Task xsens_imu::Task.hard_timestamps)
            (has_input controldev::GenericRawToMotion2D controldev::GenericRawToMotion2D.raw_command)
            (has_input controldev::RawJoystickToMotion2D controldev::RawJoystickToMotion2D.raw_command)
            (has_input controldev::RawWheelToMotion2D controldev::RawWheelToMotion2D.raw_command)
            (has_input controldev::Remote controldev::Remote.canInputDevice)
            (has_input structure_reconstruction::Task structure_reconstruction::Task.front_camera)
            (has_input structure_reconstruction::Task structure_reconstruction::Task.bottom_camera)
            (has_input structure_reconstruction::Task structure_reconstruction::Task.dynamic_transformations)
            (has_input lights::Lights lights::Lights.int_in)
            (has_input lights::Lights lights::Lights.can_in)
            (has_input auv_control::AlignedToBody auv_control::AlignedToBody.orientation_samples)
            (has_input auv_control::Base auv_control::Base.cmd_in)
            (has_input auv_control::Base auv_control::Base.cmd_cascade)
            (has_input auv_control::BasePIDController auv_control::BasePIDController.pose_samples)
            (has_input auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.altimeter)
            (has_input auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.depth)
            (has_input auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.cmd_in)
            (has_input auv_control::MotionCommand2DConverter auv_control::MotionCommand2DConverter.cmd_in)
            (has_input auv_control::OptimalHeadingController auv_control::OptimalHeadingController.orientation_samples)
            (has_input auv_control::WaypointNavigator auv_control::WaypointNavigator.trajectory)
            (has_input auv_control::WaypointNavigator auv_control::WaypointNavigator.pose_sample)
            (has_input auv_control::WorldToAligned auv_control::WorldToAligned.pose_samples)
            (has_input modemdriver::Modem modemdriver::Modem.data_in)
            (has_input modemdriver::ModemCanbus modemdriver::ModemCanbus.can_in)

            (has_output auv_rel_pos_controller::Task auv_rel_pos_controller::Task.motion_command)
            (has_output auv_rel_pos_controller::Task auv_rel_pos_controller::Task.state)
            (has_output pipeline_inspection::ColorFilter pipeline_inspection::ColorFilter.frame_out)
            (has_output pipeline_inspection::ColorFilter pipeline_inspection::ColorFilter.green_frame)
            (has_output pipeline_inspection::ColorFilter pipeline_inspection::ColorFilter.diff_frame)
            (has_output pipeline_inspection::ColorFilter pipeline_inspection::ColorFilter.state)
            (has_output pipeline_inspection::Inspection pipeline_inspection::Inspection.inspectionStatus)
            (has_output pipeline_inspection::Inspection pipeline_inspection::Inspection.pipePoints)
            (has_output pipeline_inspection::Inspection pipeline_inspection::Inspection.debugFrame)
            (has_output pipeline_inspection::Inspection pipeline_inspection::Inspection.pipeMap)
            (has_output pipeline_inspection::Inspection pipeline_inspection::Inspection.state)
            (has_output pipeline_inspection::LaserSimulation pipeline_inspection::LaserSimulation.laserPoints)
            (has_output pipeline_inspection::LaserSimulation pipeline_inspection::LaserSimulation.laserPointCloud)
            (has_output pipeline_inspection::LaserSimulation pipeline_inspection::LaserSimulation.vehiclePos)
            (has_output pipeline_inspection::LaserSimulation pipeline_inspection::LaserSimulation.state)
            (has_output uw_particle_localization::FastFusion uw_particle_localization::FastFusion.pose_samples)
            (has_output uw_particle_localization::FastFusion uw_particle_localization::FastFusion.state)
            (has_output uw_particle_localization::MotionModel uw_particle_localization::MotionModel.pose_samples)
            (has_output uw_particle_localization::MotionModel uw_particle_localization::MotionModel.stream_aligner_status)
            (has_output uw_particle_localization::MotionModel uw_particle_localization::MotionModel.state)
            (has_output uw_particle_localization::OrientationCorrection uw_particle_localization::OrientationCorrection.orientation_output)
            (has_output uw_particle_localization::OrientationCorrection uw_particle_localization::OrientationCorrection.orientation_offset_corrected)
            (has_output uw_particle_localization::OrientationCorrection uw_particle_localization::OrientationCorrection.state)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.pose_samples)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.environment)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.dead_reckoning_samples)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.full_dead_reckoning)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.particles)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.debug_sonar_beam)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.stats)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.depth_grid)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.grid_map)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.debug_filtered_obstacles)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.stream_aligner_status)
            (has_output uw_particle_localization::Task uw_particle_localization::Task.state)
            (has_output gps::BaseTask gps::BaseTask.solution)
            (has_output gps::BaseTask gps::BaseTask.position_samples)
            (has_output gps::BaseTask gps::BaseTask.state)
            (has_output gps::MB500Task gps::MB500Task.constellation)
            (has_output gps::MB500Task gps::MB500Task.time)
            (has_output line_scanner::Task line_scanner::Task.state)
            (has_output line_scanner::Task line_scanner::Task.pointcloud)
            (has_output line_scanner::Task line_scanner::Task.debug)
            (has_output message_producer::Task message_producer::Task.messages)
            (has_output message_producer::Task message_producer::Task.state)
            (has_output buoy::Detector buoy::Detector.state)
            (has_output buoy::Detector buoy::Detector.buoy)
            (has_output buoy::Detector buoy::Detector.light)
            (has_output buoy::Detector buoy::Detector.h_image)
            (has_output buoy::Detector buoy::Detector.s_image)
            (has_output buoy::Detector buoy::Detector.v_image)
            (has_output buoy::Detector buoy::Detector.binary_debug_image)
            (has_output buoy::Detector buoy::Detector.gray_debug_image)
            (has_output buoy::Detector buoy::Detector.hough_debug_image)
            (has_output buoy::Detector buoy::Detector.other_buoys)
            (has_output buoy::Detector buoy::Detector.debug_image)
            (has_output buoy::Detector2 buoy::Detector2.state)
            (has_output buoy::Detector2 buoy::Detector2.buoy)
            (has_output buoy::Detector2 buoy::Detector2.light)
            (has_output buoy::Detector2 buoy::Detector2.h_image)
            (has_output buoy::Detector2 buoy::Detector2.s_image)
            (has_output buoy::Detector2 buoy::Detector2.v_image)
            (has_output buoy::Detector2 buoy::Detector2.binary_debug_image)
            (has_output buoy::Detector2 buoy::Detector2.gray_debug_image)
            (has_output buoy::Detector2 buoy::Detector2.hough_debug_image)
            (has_output buoy::Detector2 buoy::Detector2.other_buoys)
            (has_output buoy::Detector2 buoy::Detector2.debug_image)
            (has_output buoy::ServoingOnWall buoy::ServoingOnWall.world_cmd)
            (has_output buoy::ServoingOnWall buoy::ServoingOnWall.aligned_position_cmd)
            (has_output buoy::ServoingOnWall buoy::ServoingOnWall.state)
            (has_output buoy::Survey buoy::Survey.state)
            (has_output buoy::Survey buoy::Survey.strafed_angle)
            (has_output buoy::Survey buoy::Survey.relative_position)
            (has_output buoy::Survey buoy::Survey.position)
            (has_output mars::Altimeter mars::Altimeter.ground_distance)
            (has_output mars::Camera mars::Camera.frame)
            (has_output mars::DepthCamera mars::DepthCamera.distance_image)
            (has_output mars::ForceApplier mars::ForceApplier.status)
            (has_output mars::ForceTorque6DOF mars::ForceTorque6DOF.wrenches)
            (has_output mars::HighResRangeFinder mars::HighResRangeFinder.pointcloud)
            (has_output mars::IMU mars::IMU.orientation_samples)
            (has_output mars::IMU mars::IMU.calibrated_sensors)
            (has_output mars::IMU mars::IMU.pose_samples)
            (has_output mars::Joints mars::Joints.status_samples)
            (has_output mars::Joints mars::Joints.current_values)
            (has_output mars::Joints mars::Joints.transforms)
            (has_output mars::LaserRangeFinder mars::LaserRangeFinder.scans)
            (has_output mars::Plugin mars::Plugin.state)
            (has_output mars::RotatingLaserRangeFinder mars::RotatingLaserRangeFinder.pointcloud)
            (has_output mars::Sonar mars::Sonar.sonar_beam)
            (has_output mars::Task mars::Task.time)
            (has_output mars::Task mars::Task.simulated_time)
            (has_output mars::Task mars::Task.state)
            (has_output sonar_feature_estimator::Task sonar_feature_estimator::Task.new_feature)
            (has_output sonar_feature_estimator::Task sonar_feature_estimator::Task.features_out)
            (has_output sonar_feature_estimator::Task sonar_feature_estimator::Task.debug_output)
            (has_output sonar_feature_estimator::Task sonar_feature_estimator::Task.2d_debug_output)
            (has_output sonar_feature_estimator::Task sonar_feature_estimator::Task.state)
            (has_output modem_can::Task modem_can::Task.modem_out)
            (has_output modem_can::Task modem_can::Task.canOut)
            (has_output modem_can::Task modem_can::Task.motion_command)
            (has_output modem_can::Task modem_can::Task.state)
            (has_output taskmon::Task taskmon::Task.stats)
            (has_output taskmon::Task taskmon::Task.state)
            (has_output depth_reader::Task depth_reader::Task.depthOut)
            (has_output depth_reader::Task depth_reader::Task.depth_samples)
            (has_output depth_reader::Task depth_reader::Task.canOut)
            (has_output depth_reader::Task depth_reader::Task.state)
            (has_output raw_control_command_converter::Movement raw_control_command_converter::Movement.motion_command)
            (has_output raw_control_command_converter::Movement raw_control_command_converter::Movement.world_command)
            (has_output raw_control_command_converter::Movement raw_control_command_converter::Movement.world_command_depth)
            (has_output raw_control_command_converter::Movement raw_control_command_converter::Movement.aligned_velocity_command)
            (has_output raw_control_command_converter::Movement raw_control_command_converter::Movement.state)
            (has_output raw_control_command_converter::Position raw_control_command_converter::Position.position_command)
            (has_output raw_control_command_converter::Position raw_control_command_converter::Position.world_command)
            (has_output raw_control_command_converter::Position raw_control_command_converter::Position.state)
            (has_output sonar_feature_detector::Task sonar_feature_detector::Task.features)
            (has_output sonar_feature_detector::Task sonar_feature_detector::Task.next_target)
            (has_output sonar_feature_detector::Task sonar_feature_detector::Task.next_target_feature)
            (has_output sonar_feature_detector::Task sonar_feature_detector::Task.next_target_command)
            (has_output sonar_feature_detector::Task sonar_feature_detector::Task.state)
            (has_output frame_demultiplexer::Task frame_demultiplexer::Task.oframe_pair)
            (has_output frame_demultiplexer::Task frame_demultiplexer::Task.oframe)
            (has_output frame_demultiplexer::Task frame_demultiplexer::Task.state)
            (has_output sonar_tritech::Echosounder sonar_tritech::Echosounder.ground_distance)
            (has_output sonar_tritech::Echosounder sonar_tritech::Echosounder.state)
            (has_output sonar_tritech::Micron sonar_tritech::Micron.ground_distance)
            (has_output sonar_tritech::Micron sonar_tritech::Micron.sonar_beam)
            (has_output sonar_tritech::Micron sonar_tritech::Micron.state)
            (has_output sonar_tritech::Profiling sonar_tritech::Profiling.profiling_scan)
            (has_output sonar_tritech::Profiling sonar_tritech::Profiling.state)
            (has_output battery_watcher::Task battery_watcher::Task.battery_info)
            (has_output orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.stream_aligner_status)
            (has_output orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.attitude_b_g)
            (has_output orientation_estimator::BaseEstimator orientation_estimator::BaseEstimator.state)
            (has_output orientation_estimator::IKF orientation_estimator::IKF.transformer_stream_aligner_status)
            (has_output orientation_estimator::IKF orientation_estimator::IKF.transformer_status)
            (has_output orientation_estimator::IKF orientation_estimator::IKF.attitude_b_g)
            (has_output orientation_estimator::IKF orientation_estimator::IKF.state)
            (has_output orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.stream_aligner_status)
            (has_output orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.attitude_b_g)
            (has_output orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.inputs_backup)
            (has_output orientation_estimator::IKFEstimator orientation_estimator::IKFEstimator.state)
            (has_output orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.stream_aligner_status)
            (has_output orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.attitude_b_g)
            (has_output orientation_estimator::UKFEstimator orientation_estimator::UKFEstimator.state)
            (has_output hsv_mosaicing::Task hsv_mosaicing::Task.result)
            (has_output hsv_mosaicing::Task hsv_mosaicing::Task.state)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.lines)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.peaks)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.houghspace)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.position)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.position_quality)
            (has_output sonar_wall_hough::Task sonar_wall_hough::Task.state)
            (has_output pose_estimation::BaseTask pose_estimation::BaseTask.pose_samples)
            (has_output pose_estimation::BaseTask pose_estimation::BaseTask.state)
            (has_output pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.transformer_stream_aligner_status)
            (has_output pose_estimation::UWPoseEstimator pose_estimation::UWPoseEstimator.transformer_status)
            (has_output pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.transformer_stream_aligner_status)
            (has_output pose_estimation::VehiclePoseEstimator pose_estimation::VehiclePoseEstimator.transformer_status)
            (has_output auv_waypoint_navigator::Task auv_waypoint_navigator::Task.relative_position_command)
            (has_output auv_waypoint_navigator::Task auv_waypoint_navigator::Task.current_delta)
            (has_output auv_waypoint_navigator::Task auv_waypoint_navigator::Task.current_waypoint)
            (has_output auv_waypoint_navigator::Task auv_waypoint_navigator::Task.queue_size)
            (has_output auv_waypoint_navigator::Task auv_waypoint_navigator::Task.state)
            (has_output gps_helper::GPSFaker gps_helper::GPSFaker.position_samples)
            (has_output gps_helper::GPSFaker gps_helper::GPSFaker.state)
            (has_output gps_helper::MapToGPS gps_helper::MapToGPS.gps_position)
            (has_output gps_helper::MapToGPS gps_helper::MapToGPS.transformer_stream_aligner_status)
            (has_output gps_helper::MapToGPS gps_helper::MapToGPS.transformer_status)
            (has_output gps_helper::MapToGPS gps_helper::MapToGPS.state)
            (has_output gps_helper::WaypointNavigation gps_helper::WaypointNavigation.target_waypoint)
            (has_output gps_helper::WaypointNavigation gps_helper::WaypointNavigation.heading_offset)
            (has_output gps_helper::WaypointNavigation gps_helper::WaypointNavigation.distance_delta)
            (has_output gps_helper::WaypointNavigation gps_helper::WaypointNavigation.state)
            (has_output avalon_simulation::AsvNavigation avalon_simulation::AsvNavigation.state)
            (has_output avalon_simulation::Modem avalon_simulation::Modem.motion_command)
            (has_output avalon_simulation::Modem avalon_simulation::Modem.state)
            (has_output avalon_simulation::WhiteLight avalon_simulation::WhiteLight.state)
            (has_output avalon_control::FakeWriter avalon_control::FakeWriter.motion_commands)
            (has_output avalon_control::FakeWriter avalon_control::FakeWriter.state)
            (has_output avalon_control::MotionControlTask avalon_control::MotionControlTask.hbridge_commands)
            (has_output avalon_control::MotionControlTask avalon_control::MotionControlTask.joint_commands)
            (has_output avalon_control::MotionControlTask avalon_control::MotionControlTask.debug)
            (has_output avalon_control::MotionControlTask avalon_control::MotionControlTask.estimated_ground_pos)
            (has_output avalon_control::MotionControlTask avalon_control::MotionControlTask.state)
            (has_output avalon_control::MotionFeedbackTask avalon_control::MotionFeedbackTask.hbridge_status)
            (has_output avalon_control::MotionFeedbackTask avalon_control::MotionFeedbackTask.state)
            (has_output avalon_control::PositionControlTask avalon_control::PositionControlTask.motion_commands)
            (has_output avalon_control::PositionControlTask avalon_control::PositionControlTask.state)
            (has_output avalon_control::RelFakeWriter avalon_control::RelFakeWriter.position_command)
            (has_output avalon_control::RelFakeWriter avalon_control::RelFakeWriter.state)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.next_position)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.position_command)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.next_pos_on_spline)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.last_pos_on_spline)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.segment_dist)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.world_command)
            (has_output avalon_control::TrajectoryFollower avalon_control::TrajectoryFollower.state)
            (has_output hbridge::CommandWriter hbridge::CommandWriter.state)
            (has_output hbridge::CommandWriter hbridge::CommandWriter.can_out)
            (has_output hbridge::CommandWriter hbridge::CommandWriter.speedCtrlDebug)
            (has_output hbridge::CommandWriter hbridge::CommandWriter.fakeReader)
            (has_output hbridge::SensorReader hbridge::SensorReader.state)
            (has_output hbridge::SensorReader hbridge::SensorReader.can_out)
            (has_output hbridge::SensorReader hbridge::SensorReader.status_samples)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.position_command)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.aligned_position_command)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.world_command)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.debug_data)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.transformer_stream_aligner_status)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.transformer_status)
            (has_output sonar_structure_servoing::Task sonar_structure_servoing::Task.state)
            (has_output depth_map::LaserToPoints depth_map::LaserToPoints.pointcloud)
            (has_output depth_map::LaserToPoints depth_map::LaserToPoints.state)
            (has_output depth_map::Task depth_map::Task.output)
            (has_output depth_map::Task depth_map::Task.state)
            (has_output depth_map::frame_export depth_map::frame_export.frames)
            (has_output depth_map::frame_export depth_map::frame_export.minZ)
            (has_output depth_map::frame_export depth_map::frame_export.maxZ)
            (has_output depth_map::frame_export depth_map::frame_export.state)
            (has_output depth_map::sonar_ground_distance depth_map::sonar_ground_distance.new_feature)
            (has_output depth_map::sonar_ground_distance depth_map::sonar_ground_distance.state)
            (has_output wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.position_command)
            (has_output wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.aligned_command)
            (has_output wall_servoing::DualSonarServoing wall_servoing::DualSonarServoing.wall_servoing_debug)
            (has_output wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.position_command)
            (has_output wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.aligned_position_command)
            (has_output wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.world_command)
            (has_output wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.wall_servoing_debug)
            (has_output wall_servoing::SingleSonarServoing wall_servoing::SingleSonarServoing.wall)
            (has_output wall_servoing::SonarServoing wall_servoing::SonarServoing.state)
            (has_output wall_servoing::WallDetector wall_servoing::WallDetector.point_cloud)
            (has_output wall_servoing::WallDetector wall_servoing::WallDetector.wall)
            (has_output wall_servoing::WallDetector wall_servoing::WallDetector.state)
            (has_output wall_servoing::WallServoing wall_servoing::WallServoing.motion_command)
            (has_output wall_servoing::WallServoing wall_servoing::WallServoing.world_command)
            (has_output wall_servoing::WallServoing wall_servoing::WallServoing.aligned_velocity_command)
            (has_output wall_servoing::WallServoing wall_servoing::WallServoing.state)
            (has_output camera_unicap::CameraTask camera_unicap::CameraTask.frame)
            (has_output camera_unicap::CameraTask camera_unicap::CameraTask.state)
            (has_output task_scheduler::Task task_scheduler::Task.state)
            (has_output sonar_blueview::Task sonar_blueview::Task.frame)
            (has_output sonar_blueview::Task sonar_blueview::Task.state)
            (has_output auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.pose_samples)
            (has_output auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.stream_aligner_status)
            (has_output auv_helper::depth_and_orientation_fusion auv_helper::depth_and_orientation_fusion.state)
            (has_output sysmon::Task sysmon::Task.ocu_markers)
            (has_output sysmon::Task sysmon::Task.annotations)
            (has_output sysmon::Task sysmon::Task.system_status)
            (has_output sysmon::Task sysmon::Task.state)
            (has_output logger::Logger logger::Logger.state)
            (has_output fog_kvh::Dsp3000Task fog_kvh::Dsp3000Task.rotation)
            (has_output fog_kvh::Dsp3000Task fog_kvh::Dsp3000Task.orientation_samples)
            (has_output fog_kvh::Dsp3000Task fog_kvh::Dsp3000Task.timestamp_estimator_status)
            (has_output fog_kvh::Dsp3000Task fog_kvh::Dsp3000Task.state)
            (has_output rear_sonar_distance_estimator::Task rear_sonar_distance_estimator::Task.ground_distance)
            (has_output rear_sonar_distance_estimator::Task rear_sonar_distance_estimator::Task.state)
            (has_output transformer::Task transformer::Task.configuration_state)
            (has_output transformer::Task transformer::Task.state)
            (has_output camera_base::Task camera_base::Task.frame)
            (has_output camera_base::Task camera_base::Task.frame_raw)
            (has_output camera_base::Task camera_base::Task.state)
            (has_output low_level_driver::LowLevelTask low_level_driver::LowLevelTask.state)
            (has_output wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.orientation_in_map)
            (has_output wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.transformer_stream_aligner_status)
            (has_output wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.transformer_status)
            (has_output wall_orientation_correction::OrientationInMap wall_orientation_correction::OrientationInMap.state)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.orientation_in_world)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.angle_in_world)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.debug_data)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.transformer_stream_aligner_status)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.transformer_status)
            (has_output wall_orientation_correction::Task wall_orientation_correction::Task.state)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.world_command)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.aligned_speed_command)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.left)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.right)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.top)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.bottom)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.size)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.x)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.y)
            (has_output structure_servoing::Alignment structure_servoing::Alignment.state)
            (has_output structure_servoing::Task structure_servoing::Task.servoed_angle)
            (has_output structure_servoing::Task structure_servoing::Task.angle_speed)
            (has_output structure_servoing::Task structure_servoing::Task.world_command)
            (has_output structure_servoing::Task structure_servoing::Task.aligned_speed_command)
            (has_output structure_servoing::Task structure_servoing::Task.found_structure)
            (has_output structure_servoing::Task structure_servoing::Task.state)
            (has_output structure_servoing::Task structure_servoing::Task.left)
            (has_output structure_servoing::Task structure_servoing::Task.right)
            (has_output structure_servoing::Task structure_servoing::Task.top)
            (has_output structure_servoing::Task structure_servoing::Task.bottom)
            (has_output structure_servoing::Task structure_servoing::Task.size)
            (has_output structure_servoing::Task structure_servoing::Task.heading)
            (has_output structure_servoing::Task structure_servoing::Task.cnt_left)
            (has_output structure_servoing::Task structure_servoing::Task.cnt_right)
            (has_output structure_servoing::Task structure_servoing::Task.cnt_top)
            (has_output structure_servoing::Task structure_servoing::Task.cnt_bottom)
            (has_output image_preprocessing::BaseTask image_preprocessing::BaseTask.state)
            (has_output image_preprocessing::DepthImage2Pointcloud image_preprocessing::DepthImage2Pointcloud.pointcloud)
            (has_output image_preprocessing::DepthImage2Pointcloud image_preprocessing::DepthImage2Pointcloud.stream_aligner_status)
            (has_output image_preprocessing::DepthImage2Pointcloud image_preprocessing::DepthImage2Pointcloud.state)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.oframe)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.binary_result)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.hDebug)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.hDebugGray)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.vDebug)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.vDebugGray)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.sDebug)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.sDebugGray)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.hsv_v_frame)
            (has_output image_preprocessing::HSVSegmentationAndBlur image_preprocessing::HSVSegmentationAndBlur.state)
            (has_output image_preprocessing::MonoTask image_preprocessing::MonoTask.oframe)
            (has_output image_preprocessing::StereoTask image_preprocessing::StereoTask.oframe_pair)
            (has_output offshore_pipeline_detector::SonarDetector offshore_pipeline_detector::SonarDetector.frame)
            (has_output offshore_pipeline_detector::SonarDetector offshore_pipeline_detector::SonarDetector.state)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.state)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.pipeline)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.world_command)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.aligned_position_command)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.position_command)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.debug)
            (has_output offshore_pipeline_detector::Task offshore_pipeline_detector::Task.debug_frame)
            (has_output interfaces::ActuatorCommandWriter interfaces::ActuatorCommandWriter.state)
            (has_output interfaces::ActuatorSensorReader interfaces::ActuatorSensorReader.state)
            (has_output interfaces::IMU interfaces::IMU.orientation_samples)
            (has_output interfaces::IMU interfaces::IMU.calibrated_sensors)
            (has_output interfaces::IMU interfaces::IMU.state)
            (has_output interfaces::LaserRangeFinder interfaces::LaserRangeFinder.scans)
            (has_output interfaces::LaserRangeFinder interfaces::LaserRangeFinder.state)
            (has_output interfaces::Servo interfaces::Servo.upper2lower)
            (has_output interfaces::Servo interfaces::Servo.angle)
            (has_output interfaces::Servo interfaces::Servo.state)
            (has_output canbus::InterfaceTask canbus::InterfaceTask.can_out)
            (has_output canbus::InterfaceTask canbus::InterfaceTask.state)
            (has_output canbus::Task canbus::Task.stats)
            (has_output canbus::Task canbus::Task.can_status)
            (has_output canbus::Task canbus::Task.log_message)
            (has_output canbus::Task canbus::Task.state)
            (has_output structured_light::Calibration structured_light::Calibration.stream_aligner_status)
            (has_output structured_light::Calibration structured_light::Calibration.state)
            (has_output structured_light::Task structured_light::Task.laser_scan)
            (has_output structured_light::Task structured_light::Task.candidates)
            (has_output structured_light::Task structured_light::Task.debug_frame)
            (has_output structured_light::Task structured_light::Task.state)
            (has_output xsens_imu::Task xsens_imu::Task.orientation_samples)
            (has_output xsens_imu::Task xsens_imu::Task.calibrated_sensors)
            (has_output xsens_imu::Task xsens_imu::Task.timestamp_estimator_status)
            (has_output xsens_imu::Task xsens_imu::Task.state)
            (has_output dynamixel::Task dynamixel::Task.lowerDynamixel2UpperDynamixel)
            (has_output pddl_planner::Task pddl_planner::Task.state)
            (has_output video_streamer_vlc::Capturer video_streamer_vlc::Capturer.state)
            (has_output video_streamer_vlc::Streamer video_streamer_vlc::Streamer.state)
            (has_output controldev::GenericRawToMotion2D controldev::GenericRawToMotion2D.motion_command)
            (has_output controldev::GenericRawToMotion2D controldev::GenericRawToMotion2D.state)
            (has_output controldev::GenericTask controldev::GenericTask.raw_command)
            (has_output controldev::GenericTask controldev::GenericTask.state)
            (has_output controldev::RawJoystickToMotion2D controldev::RawJoystickToMotion2D.motion_command)
            (has_output controldev::RawJoystickToMotion2D controldev::RawJoystickToMotion2D.state)
            (has_output controldev::RawWheelToMotion2D controldev::RawWheelToMotion2D.motion_command)
            (has_output controldev::RawWheelToMotion2D controldev::RawWheelToMotion2D.state)
            (has_output structure_reconstruction::Task structure_reconstruction::Task.transformer_stream_aligner_status)
            (has_output structure_reconstruction::Task structure_reconstruction::Task.transformer_status)
            (has_output structure_reconstruction::Task structure_reconstruction::Task.state)
            (has_output lights::Lights lights::Lights.light_value)
            (has_output lights::Lights lights::Lights.state)
            (has_output auv_control::AccelerationController auv_control::AccelerationController.cmd_out)
            (has_output auv_control::AlignedToBody auv_control::AlignedToBody.cmd_out)
            (has_output auv_control::Base auv_control::Base.state)
            (has_output auv_control::BasePIDController auv_control::BasePIDController.cmd_out)
            (has_output auv_control::BasePIDController auv_control::BasePIDController.pid_state)
            (has_output auv_control::ConstantCommand auv_control::ConstantCommand.cmd_out)
            (has_output auv_control::ConstantCommand auv_control::ConstantCommand.state)
            (has_output auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.floor_position)
            (has_output auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.state)
            (has_output auv_control::ConstantCommandGroundFollower auv_control::ConstantCommandGroundFollower.cmd_out)
            (has_output auv_control::MotionCommand2DConverter auv_control::MotionCommand2DConverter.cmd_out)
            (has_output auv_control::MotionCommand2DConverter auv_control::MotionCommand2DConverter.state)
            (has_output auv_control::OptimalHeadingController auv_control::OptimalHeadingController.cmd_out)
            (has_output auv_control::WaypointNavigator auv_control::WaypointNavigator.cmd_out)
            (has_output auv_control::WaypointNavigator auv_control::WaypointNavigator.waypoint_info)
            (has_output auv_control::WaypointNavigator auv_control::WaypointNavigator.state)
            (has_output auv_control::WorldToAligned auv_control::WorldToAligned.cmd_out)
            (has_output modemdriver::Modem modemdriver::Modem.data_out)
            (has_output modemdriver::Modem modemdriver::Modem.distance)
            (has_output modemdriver::Modem modemdriver::Modem.out_modem_substates)
            (has_output modemdriver::Modem modemdriver::Modem.state)
            (has_output modemdriver::ModemCanbus modemdriver::ModemCanbus.can_out)
            (has_output modemdriver::ModemCanbus modemdriver::ModemCanbus.stats)


            (requests root  root)
            (requests root  mars::IMU)

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
