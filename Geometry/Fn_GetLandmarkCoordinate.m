function [ landmark_coordinate ] = Fn_GetLandmarkCoordinate( ancor_main, ancor_au, landmark_para  )

Rm = ancor_main(1:3,1:3); pm= ancor_main(1:3,4); 
Ra = ancor_au(1:3,1:3); pa= ancor_au(1:3,4); 


n_m = landmark_para(1:3);
theta = landmark_para(4);

n = Rm*n_m;

 t =  pm-pa;
norm_t = norm(t);
 if norm_t > 0.000001
 alpha = acos((t'*n)/norm_t);
 else 
 alpha = 0;
 end


 d = norm_t*sin(alpha-theta)/sin(theta); % alpha> theta 
 
 
 
 landmark_coordinate = d*n + pm;
 
 
end

