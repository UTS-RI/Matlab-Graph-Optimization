function invHss = calculate_inverse( Hss,  NodeTable)
invHss = Hss;
types = fields(NodeTable.schur);

num_types = size(types, 1);

for i = 1:num_types
   type_i = types{i};
   dimension = NodeTable.schur.(type_i).dimension;
   upindex =   NodeTable.schur.(type_i).updateIndex;
   this_block  =  upindex(1):dimension:upindex(end);
   
   num_i  =  (upindex(end)+1 - upindex(1) )/dimension;

   for j = 1:num_i
      index_ij = this_block(j):(this_block(j)+dimension-1);
      A = Hss( index_ij ,index_ij ); 
      invHss( index_ij, index_ij ) = inv(A);
   end     

end



end

