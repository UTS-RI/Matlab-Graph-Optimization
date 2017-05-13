function [ Nodes_array ] = GetFactorX_format( EdgeTypeName )
flag=0;
switch EdgeTypeName 
    case 'PriorPose3_Factor'
        Nodes_array{1}=[]; flag=1;
    case 'PriorPose2_Factor'
        Nodes_array{1}=[]; flag=1;   
    case 'RelativePose3_Factor' 
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'RelativePose2_Factor' 
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;   
    case 'Vision3_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'IMU_Factor'
        Nodes_array=cell(6,1);        flag=1;
   case 'IMUbias_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'RGBD_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'RGBD_2D_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'Vision_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'VisionTest_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'VisionTestPoseFixed_Factor'
        Nodes_array{1}=[]; flag=1;        
    case 'VisionPoseFixed_Factor'
        Nodes_array{1}=[]; flag=1;       
    case 'PriorVelAndbias_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
        case 'VisionNoLandmark_Factor'
            Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'LinearVisionNoLandmark_Factor'     
         Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'VisionPBA_Factor'     
       Nodes_array=cell(4,1) ; flag=1;
    case 'VisionPBA_MainAnchor_Factor'     
       Nodes_array=cell(1,1) ; flag=1;
    case 'VisionPBA_AssAnchor_Factor'     
       Nodes_array=cell(3,1) ; flag=1;
    case 'VisionTestPBA_Factor'     
       Nodes_array=cell(4,1) ; flag=1;
    case 'VisionTestPBA_MainAnchor_Factor'     
       Nodes_array=cell(1,1) ; flag=1;
    case 'VisionTestPBA_AssAnchor_Factor'     
       Nodes_array=cell(3,1) ; flag=1;   
end 

if flag==0
   
    sprintf('Warning: you have not defined the new edge format in GetFactorX_format.m')
    
end


end

