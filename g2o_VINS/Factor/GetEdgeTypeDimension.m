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
    case 'IMUbias_Factor'
        dimension=6;
    case 'PriorVelAndbias_Factor'
        dimension=9;
    case 'VisionNoLandmark_Factor'
        dimension=2;
    case 'LinearVisionNoLandmark_Factor'
        dimension=2;    
end


if dimension==0
   
    sprintf('Warning: you have to define the new edge dimension in  GetEdgeTypeDimension.m')
    
end



end

