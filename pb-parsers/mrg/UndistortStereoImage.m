function [ undistorted_stereo_image ] = UndistortStereoImage( ...
    stereo_image, camera_model )
%
% [ undistorted_image ] = UndistortStereoImage( stereo_image, camera_model )
%
% INPUTS:
%   stereo_image: Struct containing stereo image as returned by
%      GetStereoImages().
%   camera_model: Bilinear LUT camera model for stereo camera.
%
% OUTPUTS:
%   undistorted_image: Struct containing undistorted and rectified stereo 
%      image along with stereo baseline and camera intrinsics.
%
% SEE ALSO:
%   GetStereoImage
%   ShowStereoImage
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
if (~(isempty(camera_model) || ...
        (isstruct(camera_model) && ...
        (all(isfield(camera_model, {'baseline', 'left', 'right'}))))))
    error(['%s - camera_model must be a struct containing bilinear' ...
        'undistortion LUTs.'], mfilename);
end

if ~isempty(stereo_image)
    
    % build output image
    undistorted_stereo_image = struct;
    undistorted_stereo_image.timestamp = stereo_image.timestamp;
    undistorted_stereo_image.baseline = camera_model.baseline;
    
    undistorted_stereo_image.left = UndistortImage(...
        stereo_image.left, camera_model.left);
    undistorted_stereo_image.right = UndistortImage(...
        stereo_image.right, camera_model.right);
    
else
    warning('%s - stereo_image is empty, returning no output', mfilename);
    undistorted_stereo_image = [];
end

end