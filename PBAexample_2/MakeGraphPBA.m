function [ Graph ] = MakeGraphPBA( NewLandmarkManager )
[ Graph ] = InitializeGraph;
 Graph.Fixed.IDname.pose0 = 1;
 Graph.Fixed.IDname.pose1 = 1;
 
 All_landmarks_names = fields(NewLandmarkManager);
 num_landmarks = size(All_landmarks_names,1);
 
 for i = 1: num_landmarks
     
     thislandmark_name = All_landmarks_names{i};
     thislandmark_info = NewLandmarkManager.(thislandmark_name);
     
     Graph = MakeGraphForOneLandmark( Graph, thislandmark_info, thislandmark_name);
 end


end


function Graph = MakeGraphForOneLandmark( Graph, thislandmark_info,thislandmark_name)

isTestEdge =1;



all_PosesNames= fields(thislandmark_info);
num_Poses = size(all_PosesNames,1);

MainAnchor_name = thislandmark_info.MainAnchor; 
 AssAnchor_name = thislandmark_info.AssAnchor;
 
 
for  i = 1:num_Poses
   
    current_posename = all_PosesNames{i};
    uv = thislandmark_info.(current_posename);
    Measurement=[];
    Measurement.value.uv = uv ;
    Measurement.value.Tcb = [eye(3) zeros(3,1)];
    
  if   isTestEdge
        Measurement.inf   = eye(3);
  else  Measurement.inf   = eye(2);
  end
  
    if strcmpi( current_posename, MainAnchor_name)
       NodeArray = []; 
       %NodeArray{1,1} = 'LPose3'; NodeArray{1,2} = MainAnchor_name;
       %NodeArray{2,1} = 'LPose3'; NodeArray{2,2} = AssAnchor_name;
       NodeArray{1,1} = 'ParallaxPoint'; NodeArray{1,2} = thislandmark_name;
      if   isTestEdge
       [ Graph ] = AddComplexEdge( Graph, 'VisionTestPBA_MainAnchor_Factor', NodeArray, Measurement );
      else
       [ Graph ] = AddComplexEdge( Graph, 'VisionPBA_MainAnchor_Factor', NodeArray, Measurement ); 
      end
    elseif strcmpi( current_posename, AssAnchor_name)
       NodeArray = []; 
       NodeArray{1,1} = 'LPose3'; NodeArray{1,2} = MainAnchor_name;
       NodeArray{2,1} = 'LPose3'; NodeArray{2,2} = AssAnchor_name;
       NodeArray{3,1} = 'ParallaxPoint'; NodeArray{3,2} = thislandmark_name; 
       if isTestEdge
       [ Graph ] = AddComplexEdge( Graph, 'VisionTestPBA_AssAnchor_Factor', NodeArray, Measurement );
       else    
       [ Graph ] = AddComplexEdge( Graph, 'VisionPBA_AssAnchor_Factor', NodeArray, Measurement ); 
       end
    elseif ~(strcmpi( current_posename, 'Value') ||  strcmpi( current_posename, 'MainAnchor') || strcmpi( current_posename, 'AssAnchor'))
       NodeArray = []; 
       NodeArray{1,1} = 'LPose3'; NodeArray{1,2} = MainAnchor_name;
       NodeArray{2,1} = 'LPose3'; NodeArray{2,2} = AssAnchor_name;
       NodeArray{3,1} = 'LPose3'; NodeArray{3,2} = current_posename;
       NodeArray{4,1} = 'ParallaxPoint'; NodeArray{4,2} = thislandmark_name;
    if isTestEdge
       [ Graph ] = AddComplexEdge( Graph, 'VisionTestPBA_Factor', NodeArray, Measurement );
    else
       [ Graph ] = AddComplexEdge( Graph, 'VisionPBA_Factor', NodeArray, Measurement ); 
    end
    end
end

Graph.Nodes.ParallaxPoint.Values.(thislandmark_name) = thislandmark_info.Value;
end

