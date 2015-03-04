% Example file for CDT Husky configuration

% Add MRG helper functions
addpath('mrg');
javaaddpath('/Users/samuelalbanie/aims_course/Hilary_2015/robotics/sample_code/pb-parsers/lib/protobuf-java.jar')
javaaddpath('/Users/samuelalbanie/aims_course/Hilary_2015/robotics/sample_code/pb-parsers/lib/datatypes_java.jar')
addpath(genpath('/Users/samuelalbanie/aims_course/Hilary_2015/robotics/sample_code/pb-parsers'))

% Load camera model
if ~exist('camera_model', 'var')
    load('BB2-14200211.mat');
end
    
% Set up MOOS channel names
laser_channel = 'LMS1xx_14280166_laser2d';
stereo_channel = 'BUMBLEBEE2_IMAGES';
pose_channel = 'vo_pose_pb';

% Initialise mex-moos and register channels
clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.11','MOOSNAME','SAS');
mexmoos('REGISTER',laser_channel,0.0);
mexmoos('REGISTER',stereo_channel,0.0);
mexmoos('REGISTER',pose_channel,0.0);

figure;

% Main loop
while true
    
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, laser_channel, true);
    stereo_image = GetStereoImages(mailbox, stereo_channel, true);
    relative_poses = GetRelativePoses(mailbox, pose_channel);
    
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    
    
    
    
    
    
    
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Display laser scan
    subplot(1,2,1);
    xlim(gca(), [-1,1]);
    ylim(gca(), [-1,1]);
    ShowLaserScan(scan);

    % Display stereo image
%      subplot(1,3,2);
%      ShowStereoImage(stereo_image)
    
    % Display undistorted stereo image
     subplot(1,2,2);
     ShowStereoImage(UndistortStereoImage(stereo_image, camera_model));

    % Display relative poses
    disp(relative_poses);
    
    drawnow
    
end