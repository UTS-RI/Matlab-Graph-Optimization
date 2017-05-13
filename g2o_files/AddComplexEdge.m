function [ Graph ] = AddComplexEdge( Graph, EdgeTypeName, NodeArray_TypeIncluded,  Measurement )


    Measurement.inf_sqrt=(Measurement.inf)^(1/2) ;
    dimension_edge  = GetEdgeTypeDimension( EdgeTypeName );
    SizeEdges=size(Graph.Edges,2);
    ThisEdgeOrder=SizeEdges+1;
    Graph.Edges{ThisEdgeOrder}.EdgeType=EdgeTypeName;
    Graph.Edges{ThisEdgeOrder}.ErrorVectorIndex=  Graph.TotalDimensionOfEdges + [1 : 1: dimension_edge   ]';
    Graph.Edges{ThisEdgeOrder}.Measurement=Measurement;
    Graph.Edges{ThisEdgeOrder}.OrderInEdges=SizeEdges+1;

    
    
CurrentTotalDimensionOfEdges=Graph.TotalDimensionOfEdges;    
num_Node=size( NodeArray_TypeIncluded, 1 );

num_fixNodes_inthisedge_now = 0;
for i=1: num_Node
   
    
    Node_typename_i=NodeArray_TypeIncluded{i,1}; 
      Node_id_i=NodeArray_TypeIncluded{i,2};
     [ Graph ] = AddOneNode( Graph, Node_typename_i, Node_id_i );    
     Graph.Edges{ThisEdgeOrder}.withNodes.(Node_id_i).NodeTypeName=Node_typename_i; 
    
   field_array_i=fields(Graph.Nodes.(Node_typename_i).Values);
   NodeID_order_i= find(strcmp(field_array_i, Node_id_i));   
   %NodeID_order_i = NodeID_order_i - Graph.Fixed.(Node_typename_i); %%  the order in this edge   
    
   FixedNumberInthisEdgeInThisNodeType = GetFixedNumber(Graph.Fixed.IDname, field_array_i, NodeID_order_i );
   NodeID_order_i = NodeID_order_i - FixedNumberInthisEdgeInThisNodeType;
   
   %Graph.Edges{ThisEdgeOrder}.withNodes.(Node_id_i).OrderInThisType=NodeID_order_i;
   
   
   
   if ~isfield(Graph.Fixed.IDname,Node_id_i)
   
   dimension_node_i  = GetNodeTypeDimension( Node_typename_i );
   
    [ RowVecotr_Node_i, ColVector_Node_i ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node_i, NodeID_order_i );
   
    if min(RowVecotr_Node_i)<0 || min(ColVector_Node_i)<0
        
       fprintf('warning! RowVecotr_Node_i or ColVector_Node_i < 0 ');
        
    end
    
    
    Graph.Nodes.(Node_typename_i).Jacobian.RowVector=[ Graph.Nodes.(Node_typename_i).Jacobian.RowVector;  RowVecotr_Node_i];
    Graph.Nodes.(Node_typename_i).Jacobian.ColVector=[Graph.Nodes.(Node_typename_i).Jacobian.ColVector;  ColVector_Node_i ];        
    num= Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian;
    index_i = num+[1:1: dimension_edge*dimension_node_i ]';    
    Graph.Edges{SizeEdges+1}.withNodes.(Node_id_i).JacobianIndex=index_i;    
    Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian=Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian+dimension_edge*dimension_node_i;
    
   end
end




Graph.TotalDimensionOfEdges=Graph.TotalDimensionOfEdges+dimension_edge;

end

function    FixedNumberInthisEdgeInThisNodeType = GetFixedNumber(GraphFixedIDname, field_array_i, NodeID_order_i )

if NodeID_order_i == 1
    FixedNumberInthisEdgeInThisNodeType = 0;
else    
    FixedNumberInthisEdgeInThisNodeType = 0;
    for  j = 1: NodeID_order_i-1
        
        if isfield( GraphFixedIDname, field_array_i{j})
            FixedNumberInthisEdgeInThisNodeType = FixedNumberInthisEdgeInThisNodeType+1;
        end
    end
end

end


