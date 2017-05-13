function invHss = calculate_inverse_new( Hss,  NodeTable)
%load matlab.mat;

types = fields(NodeTable.schur);
NodeTable = Fn_processNodeTable( NodeTable );

[ index_r, index_c, values ] = find( Hss );

num_values = length( index_r );


index_type = 1;
type_i = types{index_type};
dimension = NodeTable.schur.(type_i).dimension;


i=1;

while i<=num_values
   
   while i > max( NodeTable.schur.(types{index_type}).num   ) 
       index_type = index_type+1;
       dimension = NodeTable.schur.(type_i).dimension;
   end
   
   aa = reshape(values(i:i+dimension^2-1),dimension,dimension);
   aa = inv(aa);

   values( i:i+dimension^2-1 ) = reshape( aa, dimension^2 , 1 );
   i=i+dimension^2;
end

invHss = sparse( index_r, index_c, values   );

inv(Hss);


end 


