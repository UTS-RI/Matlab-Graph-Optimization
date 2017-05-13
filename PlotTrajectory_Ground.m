function [ y ] = PlotTrajectory_Ground( RobotState, num_poses )

XX=[];
for i=1:num_poses

    pose = RobotState{i}.pose;
    p = pose(1:3,4);
    
    XX=[XX p];
    
end


plot3( XX(1,:), XX(2,:),XX(3,:),'b');

title('ground truth');
xlabel('X unit,: m');
ylabel('Y unit,: m');
zlabel('Z unit,: m');
axis equal

y=0;

end

