function [ Graph ] = AddUnaryEdge( Graph, EdgeTypeName, NodeTypeName, NodeID,  Measurement )
    Measurement.inf_sqrt=(Measurement.inf)^(1/2) ;
   
    dimension_edge  = GetEdgeTypeDimension( EdgeTypeName );

    
    

    SizeEdges=size(Graph.Edges,2);
    ThisEdgeOrder=SizeEdges+1;
    Graph.Edges{ThisEdgeOrder}.EdgeType=EdgeTypeName;
    Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID).NodeTypeName=NodeTypeName;
    Graph.Edges{ThisEdgeOrder}.ErrorVectorIndex=  Graph.TotalDimensionOfEdges + [1 : 1: dimension_edge   ]';
    
    
    
    [ Graph ] = AddOneNode( Graph, NodeTypeName, NodeID );
    field_array=fields(Graph.Nodes.(NodeTypeName).Values);
    NodeID_order= find(strcmp(field_array, NodeID));   
    Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID).OrderInThisType=NodeID_order;
   
    
    Graph.Edges{ThisEdgeOrder}.Measurement=Measurement;
    Graph.Edges{ThisEdgeOrder}.OrderInEdges=SizeEdges+1;

    dimension_node  = GetNodeTypeDimension( NodeTypeName );
    
    
    CurrentTotalDimensionOfEdges=Graph.TotalDimensionOfEdges;
    [ RowVecotr_ThisNode, ColVector_ThisNode ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node, NodeID_order );
    Graph.Nodes.(NodeTypeName).Jacobian.RowVector=[ Graph.Nodes.(NodeTypeName).Jacobian.RowVector;  RowVecotr_ThisNode];
    Graph.Nodes.(NodeTypeName).Jacobian.ColVector=[Graph.Nodes.(NodeTypeName).Jacobian.ColVector;  ColVector_ThisNode ];
    num= Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian;
    index= num+[1:1: dimension_edge*dimension_node ]';    
    %Graph.Edges{SizeEdges+1}.Jacobian.(NodeTypeName).JacobianIndex=index;
    
    % 
    Graph.Edges{SizeEdges+1}.withNodes.(NodeID).JacobianIndex=index;
    
    Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian=Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian+dimension_edge*dimension_node;
    Graph.TotalDimensionOfEdges=Graph.TotalDimensionOfEdges+dimension_edge;
    
    
    

end

