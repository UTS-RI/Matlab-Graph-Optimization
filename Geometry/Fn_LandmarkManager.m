function [ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision )

sizem = size( Measurement_Vision, 1);

for i = 1:sizem

    landmark_id = ['landmark' num2str(Measurement_Vision(i,2) ) ];
    pose_id     = ['pose' num2str(Measurement_Vision(i,1) ) ];   
    uv = Measurement_Vision(i,3:4)';
    LandmarkManager.(landmark_id).(pose_id)= uv;

end


end

