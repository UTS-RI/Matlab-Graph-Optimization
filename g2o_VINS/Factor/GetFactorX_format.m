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
    case 'PriorVelAndbias_Factor'
        Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
        case 'VisionNoLandmark_Factor'
            Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;
    case 'LinearVisionNoLandmark_Factor'     
         Nodes_array{1}=[]; Nodes_array{2}=[]; flag=1;

end 

if flag==0
   
    sprintf('Warning: you have to define the new edge format in GetFactorX_format.m')
    
end


end

