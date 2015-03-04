ls
%% CDT Husky configuration

% Add MRG helper functions
addpath('/Users/samuelalbanie/aims_course/Hilary_2015/robotics/sample_code/pb-parsers/mrg');
% addpath('../planner');
% addpath('../control');

    
% Set up MOOS channel names
control_channel = 'robot_motion_state_topic';

% Initialise mex-moos and register subscribers
mooshost = '192.168.0.100';

clear mexmoos;
mexmoos('init', 'SERVERHOST',mooshost,'MOOSNAME','cdt-husky3','SERVERPORT','9000');
pause(1) % give mexmoos a chance to connect (important!)

SendSpeedCommand(0, 0, control_channel)

while true
    disp('Send command');
    mailbox = mexmoos('FETCH');
    
    SendSpeedCommand(0.3, 0, control_channel);
    
    pause(0.5)
end

