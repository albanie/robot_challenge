function [ scans ] = GetLaserScans( varargin )
%
% [ scans ] = GetLaserScans( moos_mailbox, 
%                            moos_channel, 
%                            only_newest [=FALSE])
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
%   scans: Struct array of laser scans (or EMPTY if there are no
%      scans available).
%

% Geoff Pascoe
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
scans = {};

if ~isempty(moos_mailbox)
    
    % find messages containing images from moos_channel
    scan_idx = find(strcmp({moos_mailbox.KEY}, moos_channel));
    
    if (only_newest && ~isempty(scan_idx))
        % return only index to the latest message.
        [~, newest_scan_idx] = max([moos_mailbox(scan_idx).TIME]);
        scan_idx = scan_idx(newest_scan_idx);
    end
    
    for idx = scan_idx
        
        % parse s from binary payload
        pbLaserScan2DPolar = javaMethod('parseFrom',...
            ['mrg.datatypes.protobuf.laser2D.PbLaser2DScanPolar'...
            '$pbLaser2DScanPolar'],typecast(moos_mailbox(idx).BIN,...
            'uint8'));
        
        scan = struct;
        scan.timestamp = int64(pbLaserScan2DPolar.getTimestamp);
        
        scan.ranges = cell2mat(cell(...
            pbLaserScan2DPolar.getRangesList.toArray));
        scan.reflectances = cell2mat(cell(...
            pbLaserScan2DPolar.getReflectanceList.toArray));
        
        scan.duration = pbLaserScan2DPolar.getScanDuration;
        scan.start_angle = pbLaserScan2DPolar.getStartAngle;
        scan.step_size = pbLaserScan2DPolar.getStepSize;
        
        scans{end+1} = scan;
    end
    
end

scans = cell2mat(scans);

end