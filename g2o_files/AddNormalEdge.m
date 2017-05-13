function [ Graph ] = AddNormalEdge( Graph, EdgeTypeName, NodeTypeName_A, NodeID_A, NodeTypeName_B, NodeID_B ,  Measurement )
    Measurement.inf_sqrt=(Measurement.inf)^(1/2) ;

    dimension_edge  = GetEdgeTypeDimension( EdgeTypeName );

    SizeEdges=size(Graph.Edges,2);
    ThisEdgeOrder=SizeEdges+1;
    Graph.Edges{ThisEdgeOrder}.EdgeType=EdgeTypeName;
    Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID_A).NodeTypeName=NodeTypeName_A;
    Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID_B).NodeTypeName=NodeTypeName_B;
    Graph.Edges{ThisEdgeOrder}.ErrorVectorIndex=  Graph.TotalDimensionOfEdges + [1 : 1: dimension_edge   ]';


     [ Graph ] = AddOneNode( Graph, NodeTypeName_A, NodeID_A );    
     field_array_A=fields(Graph.Nodes.(NodeTypeName_A).Values);
     NodeID_order_A= find(strcmp(field_array_A, NodeID_A));   
     Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID_A).OrderInThisType=NodeID_order_A;

     [ Graph ] = AddOneNode( Graph, NodeTypeName_B, NodeID_B );    
     field_array_B=fields(Graph.Nodes.(NodeTypeName_B).Values);
     NodeID_order_B= find(strcmp(field_array_B, NodeID_B));   
     Graph.Edges{ThisEdgeOrder}.withNodes.(NodeID_B).OrderInThisType=NodeID_order_B;
     
     
     
     Graph.Edges{ThisEdgeOrder}.Measurement=Measurement;
     Graph.Edges{ThisEdgeOrder}.OrderInEdges=SizeEdges+1;
     
     
     
     dimension_node_A  = GetNodeTypeDimension( NodeTypeName_A ); 
     dimension_node_B  = GetNodeTypeDimension( NodeTypeName_B );
     
     
    CurrentTotalDimensionOfEdges=Graph.TotalDimensionOfEdges;
    
    [ RowVecotr_Node_A, ColVector_Node_A ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node_A, NodeID_order_A );
    Graph.Nodes.(NodeTypeName_A).Jacobian.RowVector=[ Graph.Nodes.(NodeTypeName_A).Jacobian.RowVector;  RowVecotr_Node_A];
    Graph.Nodes.(NodeTypeName_A).Jacobian.ColVector=[Graph.Nodes.(NodeTypeName_A).Jacobian.ColVector;  ColVector_Node_A ];        
    num= Graph.Nodes.(NodeTypeName_A).Jacobian.currentNumber_AllElementsInJacobian;
    index_A= num+[1:1: dimension_edge*dimension_node_A ]';    
    %Graph.Edges{SizeEdges+1}.Jacobian.(NodeTypeName_A).JacobianIndex=index_A;
    %%%
    Graph.Edges{SizeEdges+1}.withNodes.(NodeID_A).JacobianIndex=index_A;
    
    Graph.Nodes.(NodeTypeName_A).Jacobian.currentNumber_AllElementsInJacobian=Graph.Nodes.(NodeTypeName_A).Jacobian.currentNumber_AllElementsInJacobian+dimension_edge*dimension_node_A;
    
 
    
    
    [ RowVecotr_Node_B, ColVector_Node_B ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node_B, NodeID_order_B );
    Graph.Nodes.(NodeTypeName_B).Jacobian.RowVector=[ Graph.Nodes.(NodeTypeName_B).Jacobian.RowVector;  RowVecotr_Node_B];
    Graph.Nodes.(NodeTypeName_B).Jacobian.ColVector=[Graph.Nodes.(NodeTypeName_B).Jacobian.ColVector;  ColVector_Node_B ];        
    num= Graph.Nodes.(NodeTypeName_B).Jacobian.currentNumber_AllElementsInJacobian;
    index_B= num+[1:1: dimension_edge*dimension_node_B ]';    
    
    Graph.Edges{SizeEdges+1}.withNodes.(NodeID_B).JacobianIndex=index_B;  %%% add
    
    Graph.Nodes.(NodeTypeName_B).Jacobian.currentNumber_AllElementsInJacobian=Graph.Nodes.(NodeTypeName_B).Jacobian.currentNumber_AllElementsInJacobian+dimension_edge*dimension_node_B;
    
    
    
    
    Graph.TotalDimensionOfEdges=Graph.TotalDimensionOfEdges+dimension_edge;
     

end

