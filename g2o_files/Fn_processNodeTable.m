function NodeTable = Fn_processNodeTable( NodeTable )
types = fields(NodeTable.schur);
num_types = size(types, 1);

a = 0;
for i=1:num_types

   type_i = types{i};
   dimension = NodeTable.schur.(type_i).dimension;
   upindex =   NodeTable.schur.(type_i).updateIndex;
   num_i  =  (upindex(end)+1 - upindex(1) )/dimension;
   a = a+ num_i*dimension^2;
   NodeTable.schur.(type_i).num = a; 
   
end



end
