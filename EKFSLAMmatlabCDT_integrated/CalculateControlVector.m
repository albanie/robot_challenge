function [ u ] = CalculateControlVector( delta_left, delta_right, L )



    R = (delta_right + delta_left) / 2;
    theta = (delta_left-delta_right) / (2*L);
    x = R * cos(theta);
    y = R * sin(theta);

    u = [x;
        y;
        theta];
end
