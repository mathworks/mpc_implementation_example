function xyq0q3_MAT = convert_theta_to_q_vec(xyth_MAT)

initial_flag = false;
q0q3_pre = zeros(1, 2, 'like', xyth_MAT);
xyq0q3_MAT = zeros(size(xyth_MAT, 1), 4);

for i = 1:size(xyth_MAT, 1)
    THETA = 0.5 * xyth_MAT(i, 3);
    q0q3 = [cos(THETA), sin(THETA)];
    
    if (~initial_flag)
        q0q3_pre = q0q3;
        initial_flag = true;
    else
        diff = sum(abs(q0q3 - q0q3_pre));
        
        if (diff > 0.5)
            THETA_2 = 0.5 * (xyth_MAT(i, 3) + 2 * pi);
            q0q3 = [cos(THETA_2), sin(THETA_2)];
            q0q3_pre = q0q3;
        else
            q0q3_pre = q0q3;
        end
        
    end
    
    xyq0q3_MAT(i, :) = [xyth_MAT(i, 1:2), q0q3_pre];
end

end