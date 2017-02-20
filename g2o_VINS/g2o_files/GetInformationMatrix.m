function [ Graph ] = GetInformationMatrix( Graph, TypeName_this )
%%%%%%%%%%%%%% generate the sparse Jaocbian matrix and the error vecotr
Total_ErrorVector=zeros( Graph.TotalDimensionOfEdges, 1 );       %%%%%  Initialize the Error Vector 
Number_NodeTypes=size( fields(Graph.Nodes) ,1 );
Total_Jacobian=struct;
NodeTypeNamesArray = fields(Graph.Nodes);

num_update=0;
for i=1:Number_NodeTypes 
    Total_Jacobian.( NodeTypeNamesArray{i} )=[];
    dimension= Graph.Nodes.( NodeTypeNamesArray{i}  ).Dimension;
    num_node_thisKind = size( fields (Graph.Nodes.( NodeTypeNamesArray{i}  ).Values) , 1) ;
    Graph.Nodes.( NodeTypeNamesArray{i}  ).updateIndex = num_update+ [1:1: dimension* num_node_thisKind ]'; 
    num_update=num_update+ dimension* num_node_thisKind;
end

NumberofEdges=size(Graph.Edges, 2);
NeedOptimization=true;
iter_optimization=0;

while ( NeedOptimization  && iter_optimization < Graph.Parameters.iter   )
for iter_edge=1:NumberofEdges
    NeedOptimization=false;
    
    
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
   [ ErrorVector, Jacobian_Node ]=FuncFactor( Nodes_array, edge.Measurement.value  );
    
   Total_ErrorVector( edge.ErrorVectorIndex  )=   edge.Measurement.inf_sqrt* ErrorVector;

   %% debug
   error_s =  edge.Measurement.inf_sqrt* ErrorVector;
   Error_s = error_s'*error_s;
   
   
for i = 1: num_nodes_ThisEdge
    node_id  = nodes_id{i};
    node_type= edge.withNodes.(node_id).NodeTypeName;    
     
    Jacobian_Node_i_infsqrt=edge.Measurement.inf_sqrt*Jacobian_Node{i};
    Jacobianindex_thisEdge_thisNode=edge.withNodes.(node_id).JacobianIndex;
    Graph.Nodes.(node_type).Jacobian.ValueVector( Jacobianindex_thisEdge_thisNode )=Jacobian_Node_i_infsqrt(:);
    
end      
   
end


sprintf('cost =  %f\n',Total_ErrorVector'*Total_ErrorVector )



Whole_Jacobian=[];
for i=1:Number_NodeTypes 
    ThisTypeName=NodeTypeNamesArray{i};
    all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
    num_ids_thisType= size ( all_ids_thisType, 1 );
    num_cols=num_ids_thisType*Graph.Nodes.(ThisTypeName).Dimension;
    num_rows=Graph.TotalDimensionOfEdges;
    
    Total_Jacobian.( NodeTypeNamesArray{i} )=sparse( Graph.Nodes.(ThisTypeName).Jacobian.RowVector, Graph.Nodes.(ThisTypeName).Jacobian.ColVector, Graph.Nodes.(ThisTypeName).Jacobian.ValueVector, num_rows, num_cols   );
    Whole_Jacobian=[Whole_Jacobian  Total_Jacobian.( NodeTypeNamesArray{i} )];
end




if strcmp(Graph.Parameters.OptimizationMethod,'GN')
dX= -  (Whole_Jacobian'*Whole_Jacobian)\(Whole_Jacobian'*Total_ErrorVector);  % this is the incremental vector
else
     H = Whole_Jacobian'*Whole_Jacobian;  mm=size(H,1); 
     %lambda = 0.2;
     dX = (speye(mm)*lambda-H)\(Whole_Jacobian'*Total_ErrorVector);
end    
    




%%%%% update_state
for i=1:Number_NodeTypes 
    ThisTypeName=NodeTypeNamesArray{i};
    dx = dX (Graph.Nodes.( ThisTypeName ).updateIndex);
    dimension_thisType= Graph.Nodes.( ThisTypeName ).Dimension;
    
    all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
    num_ids_thisType= size ( all_ids_thisType, 1 );
    
    for k=1:num_ids_thisType
    thisNode_id = all_ids_thisType{k};
    increment =  dx ( dimension_thisType*(k-1)+1  : dimension_thisType*k  );
    
    Z= abs( increment )-Graph.Parameters.LinearizedError.(ThisTypeName);
    if any(Z>0)
       NeedOptimization=true;
    end   
    
    Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) = update_state( ThisTypeName, Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) , increment);   
    end
   
end

iter_optimization=iter_optimization+1;
end


end
