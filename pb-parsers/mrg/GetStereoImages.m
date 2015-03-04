function [ stereo_images ] = GetStereoImages( varargin )
%
% [ stereo_images ] = GetStereoImages( moos_mailbox, 
%                                      moos_channel,
%                                      only_newest [=FALSE])
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
%   stereo_images: Struct array of stereo images (or EMPTY if there are no
%      stereo images available).
%
% SEE ALSO:
%   ShowStereoImage
%   UndistortStereoImage
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
stereo_images = {};

if ~isempty(moos_mailbox)
    
    % find messages containing images from moos_channel
    image_idx = find(strcmp({moos_mailbox.KEY}, moos_channel));
    
    if (only_newest && ~isempty(image_idx))
        % return only index to the latest message.
        [~, newest_image_idx] = max([moos_mailbox(image_idx).TIME]);
        image_idx = image_idx(newest_image_idx);
    end
    
    for idx = image_idx
        
        % parse image from binary payload
        pbImageArray = javaMethod('parseFrom',...
            'mrg.datatypes.protobuf.image.PbImageArray$pbImageArray',...
            typecast(moos_mailbox(idx).BIN, 'uint8'));
        
        stereo_image = struct;
        stereo_image.timestamp = int64(...
            pbImageArray.getImages(0).getTimestamp);
        
        stereo_image.left = pbImageToImage(pbImageArray.getImages(0));
        stereo_image.right = pbImageToImage(pbImageArray.getImages(1));
        
        stereo_images{end+1} = stereo_image;
    end
    
end

stereo_images = cell2mat(stereo_images);

end

function [ image ] = pbImageToImage( pbImage )

image.timestamp = int64(pbImage.getTimestamp);
pixeldata = typecast([pbImage.getPixelData.toByteArray],'uint8');
image.rgb = permute(reshape(reshape(pixeldata,3,numel(pixeldata)/3).',...
    [pbImage.getWidth pbImage.getHeight 3]),[2 1 3]);

end

