L = 55;  % The width of the vehicle

x_pos = [];
y_pos = [];
theta_pos = [];

for idx = 1:length(logged_wheel_data)
    % retrieve the logged data for each wheel
    left = logged_wheel_data{idx}.m_l;
    right = logged_wheel_data{idx}.m_r;
    
    % calculate the change in x, y, theta
    dx = 0;
    dy = (left + right) / 2;
    dtheta = ( -2 * ( left - right ) )/ L;
    
    % store these changes
    x_pos(end + 1) = dx;
    y_pos(end + 1) = dy;
    theta_pos(end + 1) = dtheta;
end

    