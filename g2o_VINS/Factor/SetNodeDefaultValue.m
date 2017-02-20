function [ NodeValue ] = SetNodeDefaultValue( NodeTypeName )

flag=0;

switch NodeTypeName 
    case 'Pose3'
        NodeValue=[eye(3 ) zeros(3,1)]; flag=1;
    case 'Pose2' 
        NodeValue=[eye(2) zeros(2,1)]; flag=1;
    case 'Landmark3'
        NodeValue=zeros(3,1); flag=1;
    case 'Velocity3'
        NodeValue=zeros(3,1); flag=1;
    case 'IMUbias'
        NodeValue=zeros(6,1); flag=1;
    case 'Landmark2'
        NodeValue=zeros(2,1); flag=1;
end

if flag==0
    sprintf('Warning: you have to define the default value for the new Node in  SetNodeDefaultValue.m')
end




end

