function [ Objvalue ] = Cal_ObjValue_New( Graph )
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


end

