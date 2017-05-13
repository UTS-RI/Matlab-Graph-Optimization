function h_dl=compute_DogLeg( h_gn, h_sd , delta  )

norm_hgn = norm(h_gn);
norm_hsd = norm(h_sd);
if norm_hgn <= delta
    h_dl = h_gn;
else if norm_hsd>delta
        h_dl = ( delta/norm(h_sd)  ) * h_sd;
    else
        beta = compute_beta( h_gn , h_sd , delta  );
        h_dl = h_sd + beta*(h_gn - h_sd);
    end



end

end

function  beta =compute_beta( h_gn , h_sd , delta  )

a =  h_sd;
b =  h_gn;
c =  a'*(b-a);

A2 = a'*a;
BA2 = (b-a)'*(b-a);


if c<= 0
beta = (-c+sqrt( c^2 + BA2*( delta^2 - A2  ) ))/BA2;
    
else
beta = (delta^2 - A2)/( c+ sqrt(  c^2 + BA2*( delta^2 - A2  )      )   );
    
end


end