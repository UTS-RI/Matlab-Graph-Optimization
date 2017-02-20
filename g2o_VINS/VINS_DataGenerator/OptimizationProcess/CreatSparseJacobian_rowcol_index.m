function  [ SparseJacobian_index_row,   SparseJacobian_index_col, SparseJacobian_values  ]= CreatSparseJacobian_rowcol_index( Camera,Feature   )

num.robotstate=Camera.NumOfFrames;
num.feature=Feature.num;
num.observation=size(Camera.Data,1);

SparseJacobian_index_row=ones( 6*9+9^2*2*(num.robotstate-1)+24*num.observation,1);
SparseJacobian_index_col=ones( 6*9+9^2*2*(num.robotstate-1)+24*num.observation,1);
SparseJacobian_values=ones( 6*9+9^2*2*(num.robotstate-1)+24*num.observation,1);

% Prior Factor 
s=[1:6]';
ss=[1:9];
Mpr_r= repmat(s,1,9);
Mpr_c= repmat(ss,6, 1 );
SparseJacobian_index_row(1:54)=Mpr_r(:);
SparseJacobian_index_col(1:54)=Mpr_c(:);

% IMU Factor
IndexMatrix1=[1:9]';
IndexMatrix1=repmat(IndexMatrix1,1,18 );
IndexMatrix2=[1:18];
IndexMatrix2=repmat(IndexMatrix2,9,1);
for i=0:num.robotstate-2   
    SparseJacobian_index_row(54+ 2*i*9^2+1 : 54+ 2*i*9^2+81*2)= [IndexMatrix1(:)+6+9*i];
    SparseJacobian_index_col(54+ 2*i*9^2+1 : 54+ 2*i*9^2+81*2)= [IndexMatrix2(:)+9*i];
end

A1= [ones(1,9); 2*ones(1,9)]; B1=[1 1 1; 2 2 2];
C1=[1:9;1:9]; D1=[1 2 3; 1 2 3];

%Vision Factor
for index_ob=1:num.observation

    Ri_index=Camera.Data(index_ob,1);
    fk_index=Camera.Data(index_ob,2);
    
    row_r= 6+9*(num.robotstate-1)+A1+ 2*(index_ob-1) ;
    row_f= 6+9*(num.robotstate-1)+B1+ 2*(index_ob-1) ;
    
    col_r=9*(Ri_index )+C1;
    col_f=9*(num.robotstate)+3*(fk_index-1)+D1;
    
    SparseJacobian_index_row(54+ 2*( num.robotstate-1 )*9^2+24*(index_ob-1)+1 : 54+ 2*(num.robotstate-1)*9^2+24*(index_ob-1)+24)=[row_r(:); row_f(:)];
    SparseJacobian_index_col(54+ 2*( num.robotstate-1 )*9^2+24*(index_ob-1)+1 : 54+ 2*(num.robotstate-1)*9^2+24*(index_ob-1)+24)=[ col_r(:); col_f(:)   ];
    
end


end

