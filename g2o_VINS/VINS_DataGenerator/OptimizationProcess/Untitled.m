

num.robotstate=Camera.NumOfFrames;
num.feature=Feature.num;
num.observation=size(Camera.Data,1);
FrameTime=Camera.timestep;

[ SparseJacobian_index_row,   SparseJacobian_index_col, SparseJacobian_values  ]= CreatSparseJacobian_rowcol_index( Camera,Feature   );
FullErrorVector = ones( 6 + 9*(num.robotstate-1)+2*num.observation,1 );


% Prior Factor


iteration_max=10;
for iteration=1:iteration_max

    
[ SparseJacobian_values, FullErrorVector ] = Fun_GetJacobianError( LinearizedPoint, IMUfactorMeasurement, Camera,  SparseJacobian_values , FullErrorVector, Feature );
SparseJacobian_matrix=sparse(  SparseJacobian_index_row,   SparseJacobian_index_col, SparseJacobian_values    );

A=SparseJacobian_matrix'*Inf_Matrix*SparseJacobian_matrix;

dx=A\FullErrorVector;

LinearizedPoint=Update_fullstate(LinearizedPoint, dx, num.robotstate, num.feature );


end
