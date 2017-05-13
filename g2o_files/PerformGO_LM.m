function [ Graph ] = PerformGO_LM( Graph )
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
    
      if isfield(Graph.Fixed, NodeTypeNamesArray{i})
        num_node_thisKind = num_node_thisKind - Graph.Fixed.(NodeTypeNamesArray{i});
    end
    
    
    Graph.Nodes.( NodeTypeNamesArray{i}  ).updateIndex = num_update+ [1:1: dimension* num_node_thisKind ]'; 
    num_update=num_update+ dimension* num_node_thisKind;
end

NumberofEdges=size(Graph.Edges, 2);
NeedOptimization=true;
iter_optimization=0;

v=2;

Graph.Edge_Error = zeros( NumberofEdges,1 );

while ( NeedOptimization  && iter_optimization < Graph.Parameters.iter   )
NeedOptimization_possible = false; 
    
for iter_edge=1:NumberofEdges
    NeedOptimization=true;   
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
   Graph.Edge_Error( iter_edge )  = (edge.Measurement.inf_sqrt* ErrorVector)'*(edge.Measurement.inf_sqrt* ErrorVector);
   
for i = 1: num_nodes_ThisEdge
    node_id  = nodes_id{i};
    node_type= edge.withNodes.(node_id).NodeTypeName;    
    
     if ~isfield( Graph.Fixed.IDname,  node_id)
    Jacobian_Node_i_infsqrt=edge.Measurement.inf_sqrt*Jacobian_Node{i};
    Jacobianindex_thisEdge_thisNode=edge.withNodes.(node_id).JacobianIndex;
    Graph.Nodes.(node_type).Jacobian.ValueVector( Jacobianindex_thisEdge_thisNode )=Jacobian_Node_i_infsqrt(:);
    end

    
end      
   
end

F_0 = Total_ErrorVector'*Total_ErrorVector;
L_0 = F_0;
sprintf('cost =  %f\n',Total_ErrorVector'*Total_ErrorVector )



Whole_Jacobian=[];
for i=1:Number_NodeTypes 
    ThisTypeName=NodeTypeNamesArray{i};
    all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
    num_ids_thisType= size ( all_ids_thisType, 1 );
    
    num_ids_thisType  =  num_ids_thisType - Graph.Fixed.(ThisTypeName);    % new

    
    num_cols=num_ids_thisType*Graph.Nodes.(ThisTypeName).Dimension;
    num_rows=Graph.TotalDimensionOfEdges;
    
    Total_Jacobian.( NodeTypeNamesArray{i} )=sparse( Graph.Nodes.(ThisTypeName).Jacobian.RowVector, Graph.Nodes.(ThisTypeName).Jacobian.ColVector, Graph.Nodes.(ThisTypeName).Jacobian.ValueVector, num_rows, num_cols   );
    Whole_Jacobian=[Whole_Jacobian  Total_Jacobian.( NodeTypeNamesArray{i} )];
end

G = Whole_Jacobian'*Total_ErrorVector;

if iter_optimization == 0
    AAA= Whole_Jacobian'*Whole_Jacobian;
    u = max(diag( AAA ))* Graph.Parameters.tau ;  
end   


dX=   (- u*speye( num_update  )-Whole_Jacobian'*Whole_Jacobian)\(Whole_Jacobian'*Total_ErrorVector);  % this is the incremental vector

L_new =    (Total_ErrorVector+Whole_Jacobian*dX)'*(Total_ErrorVector+Whole_Jacobian*dX);

previousNodes = Graph.Nodes;


%%%%% update_state
for i=1:Number_NodeTypes 
    ThisTypeName=NodeTypeNamesArray{i};
    dx = dX (Graph.Nodes.( ThisTypeName ).updateIndex);
    dimension_thisType= Graph.Nodes.( ThisTypeName ).Dimension;
    
    all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
    num_ids_thisType= size ( all_ids_thisType, 1 );
    
        PreFixed_num = 0;
    for k=1:num_ids_thisType
    thisNode_id = all_ids_thisType{k};
    
    if ~isfield(Graph.Fixed.IDname,  thisNode_id)  
    %%increment =  dx ( dimension_thisType*(k-1)+1  : dimension_thisType*k  );    %%%       
    increment =  dx ( dimension_thisType*(k-1-PreFixed_num)+1  : dimension_thisType*(k-PreFixed_num)  );           

    Z= abs( increment )-Graph.Parameters.LinearizedError.(ThisTypeName);
    if any(Z>0)
       NeedOptimization_possible=true;
    end       
    
    Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) = update_state( ThisTypeName, Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) , increment); 
    else
    PreFixed_num = PreFixed_num+1;
    end    
    
    
    
     end   
end

 F_new  = Cal_ObjValue( Graph );
 
 
 %p2 = dX'*(u*dX+max(abs(G)));
 %p=(F_0 - F_new)/p2;
 
 p=(F_0 - F_new)/(L_0 - L_new);
 if p>0
    u= u* max(1/3, 1-(2*p-1)^3);
    v=2;
      if abs(F_0-F_new) < Graph.Parameters.eplison*F_0
      NeedOptimization = false; Graph.stop_reason = 'obj is small';
      end
%       if NeedOptimization_possible
%       else NeedOptimization = false; Graph.stop_reason = 'dX is small';
%       end
    iter_optimization=iter_optimization+1;
 else
    Graph.Nodes=previousNodes;
    u = u*v; 
    v=2*v;
    NeedOptimization = true;
 end
end

Graph.iter_actual = iter_optimization;

end
