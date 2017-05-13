function [ dimension ] = GetNodeTypeDimension( NodeTypeName )

dimension=0;

switch NodeTypeName 
    case 'Pose3'
        dimension=6;
    case 'LPose3'
        dimension=6;    
    case 'Pose2' 
        dimension=3;
    case 'Landmark3'
        dimension=3;
    case 'Landmark2'
        dimension=2;    
    case 'Velocity3'
        dimension=3;
    case 'IMUbias'
        dimension=6;
    case 'ParallaxPoint'
        dimension=3;
end


if dimension==0
   
    sprintf('Warning: you have to define the new Node dimension in  GetNodeTypeDimension.m')
    
end

end

