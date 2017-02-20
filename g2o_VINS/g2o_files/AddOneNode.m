function [ Graph ] = AddOneNode( Graph, NodeTypeName, NodeID )



if  isfield(Graph.Nodes, NodeTypeName)     
    if  isfield(Graph.Nodes.(NodeTypeName).Values, NodeID)        
    else
        Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue( NodeTypeName );
    end
else
    Graph.Nodes.(NodeTypeName).Dimension=GetNodeTypeDimension( NodeTypeName );
    Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian=0;
    Graph.Nodes.(NodeTypeName).Jacobian.ValueVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.RowVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.ColVector=[];    
    Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue( NodeTypeName );
end


end

