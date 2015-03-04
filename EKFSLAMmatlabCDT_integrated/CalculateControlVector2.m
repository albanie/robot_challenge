function [u] = CalculateControlVector2(d_l, d_r)
	
    L = 0.55;
    
	R = (d_l + d_r)/2;
	theta = -(d_l-d_r)/(2*L);
    
    x = R*sin(theta);
    y = R*cos(theta);
	plot(x,y);

u = [x;y;theta];