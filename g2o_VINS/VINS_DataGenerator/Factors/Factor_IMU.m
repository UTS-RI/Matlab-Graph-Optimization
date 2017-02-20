function [ Jimu_ri, Jimu_rj, ErrorVector_imu_rirj ] = Factor_IMU( xi, xj, IMU_preintegrationIJ, FrameTime )

J1=IMU_preintegrationIJ(1:3,1:3);
J2=IMU_preintegrationIJ(1:3,4);
J3=IMU_preintegrationIJ(1:3,5);

hatXj=prediction(xi,J1,J2,J3, FrameTime);

[ Error , ErrorVector_imu_rirj]=Special_minus( hatXj , fun_x2X(xj)  );

% to do Jacobians
[ Jreinvserse ] = Jr_inverse( ErrorVector_imu_rirj );
J1t=J1';
%K= [ J1t zeros(3,6); J1t*skew(J2)  J1t zeros(3,3);  J1t*skew(J3)  FrameTime*J1t zeros(3,3)   ];
K= [ J1t zeros(3,6); -J1t*skew(J2)  J1t zeros(3,3);  -J1t*skew(J3)  FrameTime*J1t   J1t   ];


Ri=xi(1:3,1:3);
vi=xi(1:3,4);
pi=xi(1:3,5);

Rj=xj(1:3,1:3);
vj=xj(1:3,4);
pj=xj(1:3,5);



T= [ Ri' zeros(3,6); -skew(vi)*Ri  eye(3) zeros(3,3); -Ri'*skew(pi) zeros(3,3) Ri' ];
Tj=[ Rj' zeros(3,6); -skew(vj)*Rj  eye(3) zeros(3,3); -Rj'*skew(pj) zeros(3,3) Rj' ];
adjj=ad_G_inverse(Error);

Jimu_ri=Jreinvserse*K*T;
Jimu_rj=-Jreinvserse*adjj*Tj;
end

function hatXj=prediction(xi,J1,J2,J3, FrameTime)
g=[0 ; 0; -9.8];
Ri=xi(1:3,1:3);
hatRj=Ri*J1;

vi_G=Ri*xi(1:3,4);
hatvj_G=vi_G+Ri*J2+FrameTime*g;

pi=xi(1:3,5);
hatpj=pi+FrameTime^2/2*g+FrameTime*vi_G+Ri*J3; 

hatXj=[ hatRj hatvj_G   hatpj ];
end

function [Error,error]=Special_minus( X1 , X2  )
   
R2=X2(1:3,1:3);
R1=X1(1:3,1:3);

theta1=so3_log( R2'*R1 );

dv_G=R2'*( X1(1:3,4 )-X2(1:3,4 )   );
dp=R2'*( X1(1:3,5 )-X2(1:3,5 )   );

theta2=jacor_inverse(-theta1)*dv_G;
theta3=jacor_inverse(-theta1)*dp;

error=[theta1; theta2;theta3  ];

Error= [R2'*R1  dv_G dp ];

end