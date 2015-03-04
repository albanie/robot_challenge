function basicSLAMRobot
close all;
clear;

addpath(genpath('../pb-parsers/'));
addpath('../pb-parsers/mrg');
addpath('../PoleDetector');


% Set up MOOS channel names
laser_channel = 'LMS1xx_14280166_laser2d';
pose_channel = 'vo_pose_pb';
wheel_odom_channel = 'husky_differential_motion';

% Initialise mex-moos and register channels
clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.11','MOOSNAME','ExampleName');
mexmoos('REGISTER',laser_channel,0.0);
mexmoos('REGISTER',pose_channel,0.0);
mexmoos('REGISTER',wheel_odom_channel,0.0);

% init
x = [0; 0; 0;];
P = 0.0005 * diag([1,1,0.01]);

mailbox = mexmoos('FETCH');
pause(1)

wheel_odom = GetWheelOdometry(mailbox, wheel_odom_channel, true);
if ~isempty(wheel_odom)
    initial_left = wheel_odom.m_l;
    initial_right = wheel_odom.m_r;
else
    pause(1)
    mailbox = mexmoos('FETCH');
    pause(0.2)
    wheel_odom = GetWheelOdometry(mailbox, wheel_odom_channel, true);
    if ~isempty(wheel_odom)
        initial_left = wheel_odom.m_l;
        initial_right = wheel_odom.m_r;
    else
        disp('No wheel odometry')
        return
    end
end

car_measurements = Car_measurements(initial_left, initial_right);
figure;

loop_idx = 1;
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
%     scan = GetLaserScans(mailbox, laser_channel, true);
    wheel_odom = GetWheelOdometry(mailbox, wheel_odom_channel, true);
    

    if ~isempty(scan) && ~isempty(wheel_odom)
        disp(['Start loop ', num2str(loop_idx)]);
        
        
        % Estimate car pose using wheel odometry readings
        add_wheel_reading(car_measurements, wheel_odom);
        [d_l, d_r] = get_last_wheel_reading(car_measurements); 
        % estimated_pose: x-forward, y-right, theta-clockwise
        estimated_pose = estimate_pose(car_measurements, d_l, d_r);
        
        [xPred, PPred] = SLAMPrediction(estimated_pose, x, P);
        x = xPred;
        P = PPred;

        % do data associations
        z = SLAMDataAssociations(x, pole_polar);

        [x P] = SLAMMeasurement(z, x, P);
        
        % Plot [x,y] pose estimations
        hold on;
        [ logged_x, logged_y ] = car_measurements.logged_poses(1:2, end);
        plot( logged_x, logged_y);
%         set(0, 'currentfigure', fig3);
%         plot(u_history);
%         legend('x','y','t');
        
        
%         set(0, 'currentfigure',fig4);
%         a = axis;
%         clf;
%         axis(a);
%         hold on;
%         n  = length(x);
%         nF = (n-3)/2;
% 
%         x_vis = vis_transform * x(1:3);
%         P_vis = P(1:3,1:3)';
%         DoVehicleGraphics(x_vis,P_vis,3,[0 1]);
% 
%         for i=1:size(z,1)
%             if z(i, 3) > 0
%                 iF = 3 + 2*z(i,3) -1;
%                 xFeature = [x(iF), x(iF+1)];
%                 xFeature = vis_transform(1:2,1:2) * xFeature';
% 
%                 h = line([x_vis(1),xFeature(1)],[x_vis(2),xFeature(2)]);
%                 set(h,'linestyle',':');
%             end
%         end

        loop_idx = loop_idx + 1;
    end % end scan not empty


end