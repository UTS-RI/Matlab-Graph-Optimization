function y = PlotTrajectory( Graph )

Landmarks_es = Graph.Nodes.Landmark3.Values; 
Robot_es = Graph.Nodes.Pose3.Values;

PoseArray = fields(Robot_es);
sizePose = size( PoseArray,1 );

LandmarkArray = fields(Landmarks_es);
sizeLandmark = size( LandmarkArray,1 );


Position_es =[];

F_es = [];

for i=1:sizePose
pose_i = Robot_es.( PoseArray{i} );
position_i = pose_i(1:3,4);    
    
Position_es=[Position_es position_i];
end

for i=1:sizeLandmark
f_i = Landmarks_es.( LandmarkArray{i} );    
F_es=[F_es f_i];
end


plot3(Position_es(1,:), Position_es(2,:),Position_es(3,:),'b');
hold on;
plot3(F_es(1,:), F_es(2,:),F_es(3,:),'go');
xlabel('X unit,: m');
ylabel('Y unit,: m');
zlabel('Z unit,: m');


axis equal



y=0;

end