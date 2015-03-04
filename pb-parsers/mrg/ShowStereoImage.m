function [ ] = ShowStereoImage( stereo_image )
%
% ShowStereoImage( stereo_image )
%
% INPUTS:
%   stereo_image: Struct containing stereo image as returned by
%      GetStereoImages().
%
% SEE ALSO:
%   GetStereoImage
%   UndistortStereoImage
%

% Will Maddern
% February 2015
% Mobile Robotics Group, Oxford University.

% verify inputs
if (~(isempty(stereo_image) || ...
        (isstruct(stereo_image) && ...
        (all(isfield(stereo_image, {'timestamp', 'left', 'right'}))))))
    error(['%s - stereo_image must be a struct output from ' ...
        'GetStereoImages().'], mfilename);
end

if ~isempty(stereo_image)
    
    % display stereo image
    imshow(cat(2,stereo_image.left.rgb, stereo_image.right.rgb));
    title(num2str(int64(stereo_image.timestamp)));
    
else
    warning('%s - stereo_image is empty.', mfilename);
end

end