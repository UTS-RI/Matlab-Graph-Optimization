F = full(Total_Jacobian.Pose3);
E = full(Total_Jacobian.Landmark3);
C = full(Total_ErrorVector);   

dx = rand(24,1);
dy = zeros(30,1);

norm(F*dx+E*dy+C)



nullE = null(E');
norm( nullE'*(F*dx+E*dy+C))

