function [ undistorted_image ] = UndistortImage(image, model)

undistorted_image = struct;
undistorted_image.timestamp = image.timestamp;
undistorted_image.fx = model.fx;
undistorted_image.fy = model.fy;
undistorted_image.cx = model.cx;
undistorted_image.cy = model.cy;

for k = 1:size(image.rgb, 3)
    undistorted_image.rgb(:,:,k) = cast(reshape(interp2(cast(...
        image.rgb(:,:,k), 'single'),  model.lut(:,1)+1,  model.lut(:,2)+1, ...
        'bilinear'),  [size(image.rgb, 2), size(image.rgb, 1)]).', ...
        'like', image.rgb);
end

end