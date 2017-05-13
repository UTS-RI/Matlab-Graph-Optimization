function [ NewLandmarkManager ] = GeneratePBALandmarkManager( filename , num_poses)


load (filename);



S_max= find(Measurement_Vision(:,1)== num_poses , 1, 'last' );
[ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision(1: S_max, :) );

all_landmarkNames = fields(LandmarkManager);
num_landmarks =size(all_landmarkNames,1);

% delete all landmarks observed only once
NewLandmarkManager = struct;
for i=1:num_landmarks
    name_i = all_landmarkNames{i};
    
    numOBs_thisLandmark = size(fields(LandmarkManager.(name_i)),1);
   
    if numOBs_thisLandmark>2
        NewLandmarkManager.(name_i) = LandmarkManager.(name_i);
    end    
end



end

