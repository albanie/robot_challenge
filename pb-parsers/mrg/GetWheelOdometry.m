function [ odom ] = GetWheelOdometry( varargin )
%
% [ odom ] = GetWheelOdometry( moos_mailbox,
%                              moos_channel,
%                              only_newest [=FALSE])
%
% INPUTS:
%   moos_mailbox: MOOS mailbox structure array returned from calling
%      mexmoos('FETCH').
%   moos_channel: String containing the name of the MOOS channel (variable)
%      for which to return the messages (if one exists).
%   only_newest: Only return the latest message to be received on the
%      channel specified by moos_channel (default = FALSE).
%
% OUTPUTS:
%   odom: Struct array of odometry messages (or EMPTY if there are no
%      odometry messages available).
%

% Will Maddern
% February 2015
% Mobile Robotics Group, Oxford University.

% parse inputs
arg_idx = 1;
if (~(isempty(varargin{arg_idx}) || ...
        (isstruct(varargin{arg_idx}) && ...
        all(isfield(varargin{arg_idx}, {'KEY', 'TIME', 'BIN'})) )))
    error(['%s - moos_mailbox must be a struct array output from ' ...
        'mexmoos(''FETCH''), or empty.'], mfilename);
end
moos_mailbox = varargin{arg_idx};
arg_idx = arg_idx + 1;

if (~(ischar(varargin{arg_idx}) && ~isempty(varargin{arg_idx})))
    error('%s - moos_channel must be a non-empty string.', mfilename);
end
moos_channel = varargin{arg_idx};
arg_idx = arg_idx + 1;

% parse optional inputs
only_newest = false;
if ( nargin >= arg_idx )
    if ( ~islogical(varargin{arg_idx}) )
        error('%s - only_newest must be a logical value (true or false)',...
            mfilename);
    end
    only_newest = varargin{arg_idx};
end

% extract images from mailbox
odom = {};

if ~isempty(moos_mailbox)
    
    % find messages containing images from moos_channel
    odom_idx = find(strcmp({moos_mailbox.KEY}, moos_channel));
    
    if (only_newest && ~isempty(odom_idx))
        % return only index to the latest message.
        [~, newest_odom_idx] = max([moos_mailbox(odom_idx).TIME]);
        odom_idx = odom_idx(newest_odom_idx);
    end
    
    for idx = odom_idx
        
        % parse s from binary payload
        pbDifferentialMotion = javaMethod('parseFrom',...
            ['mrg.datatypes.protobuf.motion.PbDifferentialMotion'...
            '$pbDifferentialMotion'],typecast(moos_mailbox(idx).BIN,...
            'uint8'));
        
        odometry = struct;
        odometry.timestamp = int64(...
            pbDifferentialMotion.getTimestamp);
        odometry.m_l = pbDifferentialMotion.getThetaPosL;
        odometry.m_r = pbDifferentialMotion.getThetaPosR;
        
        odom{end+1} = odometry;
    end
    
end

odom = cell2mat(odom);

end