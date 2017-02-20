function [ RowVecotr_ThisNode, ColVector_ThisNode ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node, NodeID_order )


 A= 1:1:dimension_edge;
 A= A';

 B= 1:1:dimension_node;
 
 RowMatrix=  repmat(A, 1, dimension_node )+ CurrentTotalDimensionOfEdges ;
 VectorMatrix = repmat( B, dimension_edge,1 ) +   (NodeID_order-1)*dimension_node;
 
 
 RowVecotr_ThisNode=RowMatrix(:);
 ColVector_ThisNode=VectorMatrix(:);
 
end

