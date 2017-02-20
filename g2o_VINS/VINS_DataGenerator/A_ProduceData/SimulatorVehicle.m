function [ Vehicle ] = SimulatorVehicle( t )


% Vehicle.position= [ 5*cos(0.3*t); 4*sin(0.2*t); 2*sin(0.2*t+1) ];
% dp=[-1.5*sin(0.3*t); 0.8*cos(0.2*t); 0.4*cos(0.2*t+1)   ];
% ddp=[-0.45*cos(0.3*t); -0.16*sin(0.2*t); -0.08*sin(0.2*t+1)];
k=2*pi/20; 

Vehicle.position= [ 5*cos(k*t); 4*sin(k*t); 0.4*sin(k*t) ];
dp=[-5*sin(k*t); 4*cos(k*t); 0.4*cos(k*t) ]*k;
ddp=[-5*cos(k*t);-4*sin(k*t);-0.4*sin(k*t)  ]*k^2;


Vehicle.OneLoopTime = 2*pi/k;



theta=[ 0.2*cos(t) ; 0.3*sin(t) ; k*t ];%-0.3*t ];
dtheta=[- 0.2*sin(t) ; 0.3*cos(t);  k ];

R0 = [0 0 -1; -1 0 0; 0 1 0 ];


Vehicle.orientation=EularAngle( theta )* R0 ;
Vehicle.lv=Vehicle.orientation'* dp;
Vehicle.dp=dp;

Vehicle.av=R0' *( J_rota(theta)*dtheta );
Vehicle.accLocal= Vehicle.orientation'* (ddp -[0;0; -9.8]  );



 

end

