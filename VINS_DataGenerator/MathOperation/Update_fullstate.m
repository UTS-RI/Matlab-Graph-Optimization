function [ newLinearizedPoint ]=Update_fullstate(LinearizedPoint, dx , num_robot,num_feature )
newLinearizedPoint=LinearizedPoint;

for ii=1:num_robot
    index_ii= 1 + 3*( ii-1 ) :  3+3*( ii-1 );
    index_robot= 1+9*(ii-1)  : 9 *(ii-1)+9;
    newLinearizedPoint.robot( index_ii  , 1:5    )= update_robotstate(LinearizedPoint.robot( index_ii , 1:5  ), dx( index_robot )  );    
end

feature_matrix=LinearizedPoint.feature(1:3,:);
feature_vector=feature_matrix(:);
feature_vector=feature_vector+dx( 9*num_robot+1:end);

feature_matrix=reshape(feature_vector, 3, num_feature );


newLinearizedPoint.feature( 1:3, : )=feature_matrix;


end

