function [ dimension ] = GetEdgeTypeDimension( EdgeTypeName )


dimension=0;

switch EdgeTypeName 
    case 'PriorPose3_Factor'
        dimension=6;
    case 'PriorPose2_Factor'
        dimension=3;    
    case 'RelativePose3_Factor' 
        dimension=6;
    case 'RelativePose2_Factor' 
        dimension=3;
    case 'LinearVision_Factor'
        dimension=2;
    case 'IMU_Factor'
        dimension=9;
    case 'RGBD_Factor'
        dimension=3;
    case 'RGBD_2D_Factor'
        dimension=2;
    case 'Vision_Factor'
        dimension=2;
    case 'VisionPoseFixed_Factor'
        dimension=2;       
    case 'VisionTest_Factor'
        dimension=3;
    case 'VisionTestPoseFixed_Factor'
        dimension=3;        
    case 'IMUbias_Factor'
        dimension=6;
    case 'PriorVelAndbias_Factor'
        dimension=9;
    case 'VisionNoLandmark_Factor'
        dimension=2;
    case 'LinearVisionNoLandmark_Factor'
        dimension=2;
    case 'VisionPBA_Factor'     
        dimension=2;
    case 'VisionPBA_MainAnchor_Factor'     
        dimension=2;
    case 'VisionPBA_AssAnchor_Factor' 
        dimension=2;
    case 'VisionTestPBA_Factor'     
        dimension=3;
    case 'VisionTestPBA_MainAnchor_Factor'     
        dimension=3;
    case 'VisionTestPBA_AssAnchor_Factor' 
        dimension=3;    
end


if dimension==0
   
    sprintf('Warning: you have not defined the new edge dimension in  GetEdgeTypeDimension.m')
    
end



end

