function [ NodeTable ] = classifyNodeType( NodeTypeNamesArray, schur )

Number_NodeTypes=size( NodeTypeNamesArray ,1 );

for i = 1:Number_NodeTypes
    NodeType_i  = NodeTypeNamesArray{i};
    dimension_i =  GetNodeTypeDimension(NodeType_i);
    
    if isfield( schur, NodeType_i)    
    %NodeTable.schur.(NodeType_i)=[];
    NodeTable.schur.(NodeType_i).dimension=dimension_i;
    else     
    %NodeTable.normal.(NodeType_i)=[];
    NodeTable.normal.(NodeType_i).dimension=dimension_i;
    end
end


end

