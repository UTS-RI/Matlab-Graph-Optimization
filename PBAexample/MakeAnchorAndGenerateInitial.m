function NewLandmarkManager = MakeAnchorAndGenerateInitial( LandmarkManager, filename )


all_landmarkNames = fields(LandmarkManager);
num_landmarks =size(all_landmarkNames,1);


for i = 1: num_landmarks
    
    name_i = all_landmarkNames{i};
    landmark_i_info = LandmarkManager.(name_i);
    Newlandmark_i_info = MakeAnchorAndGenerateInitial_ForEachLandmark(landmark_i_info, filename);
    
    NewLandmarkManager.(name_i) = Newlandmark_i_info;
     
end



end

function  landmark_i_info =  MakeAnchorAndGenerateInitial_ForEachLandmark(landmark_i_info, filename)

load (filename);

all_PosesNames= fields(landmark_i_info);
num_Poses = size(all_PosesNames,1);

main_Anchor = all_PosesNames{1};
landmark_i_info.MainAnchor = main_Anchor;



SS =regexp(main_Anchor,'e','split');
index_pose = SS{2};
index_pose = str2num( index_pose );
pose_m = RobotState{index_pose+1}.pose;
uvArray{1} = landmark_i_info.(main_Anchor);
tempuvArray{1} = landmark_i_info.(main_Anchor);



Value = [ 0 ;0;0; -1 ];

j=2;
while ( j<=num_Poses && Value(4) <= 0.5 )
  
  tempAss_Anchor = all_PosesNames{j};
  SS =regexp(tempAss_Anchor,'e','split');
  index_Asspose = SS{2};
  index_Asspose = str2num(index_Asspose);
  Robot_temp = RobotState{index_Asspose+1};
  pose_temp = Robot_temp.pose;

  tempuvArray{2} = landmark_i_info.(tempAss_Anchor);
  
  tempValue =  GetParallexFromTwoPoses( pose_m, pose_temp, tempuvArray );
    
  if tempValue(4)> Value(4)
     Value = tempValue;
     uvArray =  tempuvArray;  
     Ass_Anchor = tempAss_Anchor;
  end
      
 j = j +1 ;    
end

landmark_i_info.AssAnchor = Ass_Anchor;
landmark_i_info.Value = Value;


end