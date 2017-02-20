function delta = update_radius(p, h_dl, delta )

if p>0.75
    delta = max(delta, 3*norm(h_dl));
else if p<0.25
    delta = delta/2;
    end

end