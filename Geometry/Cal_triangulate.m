function [ Landmarks_values ] = Cal_triangulate( Pose3_values , LandmarkManager )

landmarks_names = fields( LandmarkManager );
num_landmarks = size( landmarks_names, 1);

for i=1:num_landmarks
    landmark_id = landmarks_names{i};
    this_landmark_allobservations = LandmarkManager.(landmark_id);
    
    posearray = fields( this_landmark_allobservations );
    num_poses = size( posearray, 1 );
    if num_poses>1
    landmark_position = Fn_triangulate( Pose3_values,  this_landmark_allobservations );     
    Landmarks_values.(landmark_id)=landmark_position;
    end
end



end

function landmark_position = Fn_triangulate( Pose3_values,  this_landmark_allobservations )

posearray = fields( this_landmark_allobservations );
num_poses = size( posearray, 1 );

A = zeros(2*num_poses, 3 );
B = zeros(2*num_poses, 1 );

for i=1:num_poses
pose_id = posearray{i};
pose_i  = Pose3_values.(pose_id);
Ri = pose_i(1:3,1:3); pi = pose_i(1:3,4);    
uv = this_landmark_allobservations.(pose_id);
    
    fx = 525.0;
    fy = 525.0;
    cx0 = 639.5;
    cy0 = 479.5;
K_inv = [ 1/fx 0; 0  1/fy];
A_i = [ eye(2) -K_inv*(uv - [cx0;cy0] ) ];

A(2*i-1:2*i,1:3) =A_i*Ri'; 
B(2*i-1:2*i,:) = A_i*Ri'*pi;

end

landmark_position = pinv(A)*B;


end