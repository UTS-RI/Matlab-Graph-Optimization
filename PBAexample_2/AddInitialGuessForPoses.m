function [Graph] = AddInitialGuessForPoses(Graph, filename)

load (filename);

all_Pose_name = fields(Graph.Nodes.LPose3.Values);
num_poses = size( all_Pose_name,1);

for i = 1: num_poses

    thisPose_name = all_Pose_name{i};
    SS =regexp(thisPose_name,'e','split');
    index_pose = SS{2};
    pose_No = str2num( index_pose );
    
    
    pose =  RobotState{pose_No+1}.pose;
    
    if pose_No~=0 && pose_No~=1
    R = pose(1:3,1:3)*expm(skew( randn(3,1)*0.1));
    t = pose(1:3,4)+randn(3,1)*0.1;
    pose = [R t];
    end
    
    Graph.Nodes.LPose3.Values.(thisPose_name)=pose;
    
end


end

