addpath('../pb-parsers/mrg')

close all;
fig1 = figure;
fig2 = figure;
fig3 = figure;

laser_channel = 'LMS1xx_14280166_laser2d';
wheel_odom_channel = 'husky_differential_motion';


clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.11','MOOSNAME','PoleDetectorDemo');
mexmoos('REGISTER',laser_channel,0.0);
mexmoos('REGISTER',wheel_odom_channel,0.0);

logged_scan_data = {};
logged_wheel_data = {};

while true
    mailbox = mexmoos('FETCH');
    
    % log the laser scanned data
    scan = GetLaserScans(mailbox, laser_channel, true);    
    if ~isempty(scan)
        logged_scan_data{end + 1} = scan;
    end
    
    % log the wheel odometry data
    wheel_odom = GetWheelOdometry(mailbox, wheel_odom_channel, true);
    if ~isempty(wheel_odom)
        logged_wheel_data{end + 1} = wheel_odom;
    end
end


