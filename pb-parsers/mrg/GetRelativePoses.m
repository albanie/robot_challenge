function [ poses ] = GetRelativePoses( varargin )
%
% [ poses ] = GetRelativePoses( moos_mailbox,
%                               moos_channel,
%                               only_newest [=FALSE])
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
%   poses: Struct array of relative poses (or EMPTY if there are no
%      relative poses available).
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
poses = {};

if ~isempty(moos_mailbox)
    
    % find messages containing images from moos_channel
    pose_idx = find(strcmp({moos_mailbox.KEY}, moos_channel));
    
    if (only_newest && ~isempty(pose_idx))
        % return only index to the latest message.
        [~, newest_pose_idx] = max([moos_mailbox(pose_idx).TIME]);
        pose_idx = pose_idx(newest_pose_idx);
    end
    
    for idx = pose_idx
        
        % parse s from binary payload
        pbSerialisedTransform = javaMethod('parseFrom',...
            ['mrg.datatypes.protobuf.transform.PbSerialisedTransform'...
            '$pbSerialisedTransform'],typecast(moos_mailbox(idx).BIN,...
            'uint8'));
        pbTransform = javaMethod('parseFrom', ...
            'mrg.datatypes.protobuf.transform.PbTransform$pbTransform', ...
            typecast(pbSerialisedTransform.getSerialisedTransform ...
            .toByteArray,'uint8'));
        
        pose = struct;
        pose.src_timestamp = int64(pbTransform.getSourceTimestamp);
        pose.dst_timestamp = int64(pbTransform.getDestinationTimestamp);
        pose.xyzrpy = [reshape(cell2mat(...
            cell(pbTransform.getLinXyzList.toArray)),1,3), ...
            reshape(cell2mat(...
            cell(pbTransform.getAngRpyList.toArray)),1,3)];
        
        poses{end+1} = pose;
    end
    
end

poses = cell2mat(poses);

end