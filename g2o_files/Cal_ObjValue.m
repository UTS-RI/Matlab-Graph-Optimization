function [ Objvalue ] = Cal_ObjValue( Graph )
Objvalue= 0;
NumberofEdges=size(Graph.Edges, 2);
for iter_edge=1:NumberofEdges      
    edge =Graph.Edges{iter_edge};
    edgeType=edge.EdgeType;   
     num_nodes_ThisEdge= size( fieldnames(edge.withNodes),1);
     Nodes_array=cell(num_nodes_ThisEdge, 1 );
     nodes_id = fields(edge.withNodes);
for i = 1: num_nodes_ThisEdge
    node_id  = nodes_id{i};
    node_type= edge.withNodes.(node_id).NodeTypeName;    
    Nodes_array{i}=Graph.Nodes.(node_type).Values.(node_id); 
end
     FuncFactor=str2func(edgeType);
   [ ErrorVector, ~ ]=FuncFactor( Nodes_array, edge.Measurement.value  );  
   Objvalue  =  Objvalue + ErrorVector'*edge.Measurement.inf*ErrorVector;
end

% Total_ErrorVector=zeros( Graph.TotalDimensionOfEdges, 1 );       %%%%%  Initialize the Error Vector 
% Number_NodeTypes=size( fields(Graph.Nodes) ,1 );
% Total_Jacobian=struct;
% NodeTypeNamesArray = fields(Graph.Nodes);
% 
% num_update=0;
% for i=1:Number_NodeTypes 
%     Total_Jacobian.( NodeTypeNamesArray{i} )=[];
%     dimension= Graph.Nodes.( NodeTypeNamesArray{i}  ).Dimension;
%     num_node_thisKind = size( fields (Graph.Nodes.( NodeTypeNamesArray{i}  ).Values) , 1) ;
%     
%     if isfield(Graph.Fixed, NodeTypeNamesArray{i})
%         num_node_thisKind = num_node_thisKind - Graph.Fixed.(NodeTypeNamesArray{i});
%     end    
%     
%     Graph.Nodes.( NodeTypeNamesArray{i}  ).updateIndex = num_update+ [1:1: dimension* num_node_thisKind ]'; 
%     num_update=num_update+ dimension* num_node_thisKind;
% end
% 
% NumberofEdges=size(Graph.Edges, 2);
% 
% 
% for iter_edge=1:NumberofEdges    
%     
%     edge =Graph.Edges{iter_edge};
%     edgeType=edge.EdgeType;
%     
%      num_nodes_ThisEdge= size( fieldnames(edge.withNodes),1);
%      Nodes_array=cell(num_nodes_ThisEdge, 1 );
%      nodes_id = fields(edge.withNodes);
% for i = 1: num_nodes_ThisEdge
%     node_id  = nodes_id{i};
%     node_type= edge.withNodes.(node_id).NodeTypeName;    
%     Nodes_array{i}=Graph.Nodes.(node_type).Values.(node_id); 
% end
%      FuncFactor=str2func(edgeType);
%    [ ErrorVector, ~ ]=FuncFactor( Nodes_array, edge.Measurement.value  );
%     
%    Total_ErrorVector( edge.ErrorVectorIndex  )=   edge.Measurement.inf_sqrt* ErrorVector;
%    
% end
% 
%    Objvalue = Total_ErrorVector'*Total_ErrorVector;

end

