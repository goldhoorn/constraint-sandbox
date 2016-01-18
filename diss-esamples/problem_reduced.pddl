(define (problem network001)
  (:domain network)
  (:objects
        root - instance_req
        Base::ImageProviderSrv - instance_req
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
        Base::OrientationWithZSrv - instance_req
        AuvControl::DepthFusionCmp - instance_req
  )
  (:init
        (composition  root)
        (composition  AuvControl::DepthFusionCmp)
        (fullfills AuvControl::DepthFusionCmp  AuvControl::DepthFusionCmp)
        (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::OrientationWithZSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::ZProviderSrv)
        (fullfills AuvControl::DepthFusionCmp  Base::OrientationSrv)
        (task  OffshorePipelineDetector::Task)
        (data-service  Base::ImageProviderSrv)
        (depends Pipeline::Detector  OffshorePipelineDetector::Task)
        (depends Pipeline::Detector  Base::ImageProviderSrv)
        (depends Pipeline::Detector  Base::OrientationWithZSrv)
        (fullfills CameraProsilica::Task  Base::ImageProviderSrv)

; Begin requirements
      	    (depends root root)
            (depends root  Pipeline::Detector)

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
  
  ))
)
