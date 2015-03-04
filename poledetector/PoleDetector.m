function [ pole_coords_xy, pole_range_bearing ] = PoleDetector( scan, refl_threshold, min_dist_between_poles )

    % Hardcoded settings you probably wont want to change
    % ---------------------------------------------------
    pole_diam = 0.1; % meters
    min_num_points_per_pole = 3;
    exclude_outer_region_of_scan = 30; % deg
    laser_dist = 0.44;
    % --------------------0------------------------------

    % Smooths the data with a ten point moving average
    scan.reflectances = smooth(scan.reflectances,10);

    % Find local maxima in reflectances
    maxima_refl_loc = diff(scan.reflectances(2:end))<0 & diff(scan.reflectances(1:(end-1)))>0;
    maxima_refl_loc = [0; maxima_refl_loc; 0];

    % Get the values of those local maxima
    maxima_refl_val = zeros(size(maxima_refl_loc,1),1);
    maxima_refl_val(maxima_refl_loc==1) = scan.reflectances(maxima_refl_loc==1);

    % Calculate coordinates of scan readings in cartesian frame
    angles = -((0:size(scan.ranges, 1)-1).'*-scan.step_size - scan.start_angle + 90) * pi/180;
    [ranges angles] = TranslateLaserScan(scan.ranges, angles, laser_dist);
    coords = [ranges.*cos(angles) ranges.*sin(angles)];


    % Do the reflectance maxima look like poles in cartesian frame?
    % exclude the outer region of the laser scan if desired
    start_ind = exclude_outer_region_of_scan/scan.step_size+1;
    stop_ind = size(ranges,1) - exclude_outer_region_of_scan/scan.step_size;
    pole_detected = zeros(size(ranges,1),1);
    for i=start_ind:stop_ind
        % 1) Must be a local reflectance maxima and pass reflectance threshold
        if maxima_refl_val(i) > refl_threshold
            % horrible way of calculating distance to particular coordinate
            rel_coords(:,1) = coords(:,1) - coords(i,1);
            rel_coords(:,2) = coords(:,2) - coords(i,2);
            dist_to_i = (rel_coords(:,1).*rel_coords(:,1) + rel_coords(:,2).*rel_coords(:,2)).^0.5;

            % 2) Must have minimum number of points on pole
            % 3) In cartesian frame, must be a cluster of points surrounded by
            % nothing
            pole_detected(i) = sum(dist_to_i < 0.2) >= min_num_points_per_pole ...
                && sum(dist_to_i < pole_diam*2) == sum(dist_to_i < pole_diam*2+min_dist_between_poles);
        end
    end

    % return pole coordinates as the centre point of the detected pole
    % (extend laser range of local reflectance maxima by pole_diam/2)
    pole_coords_xy = [(ranges(pole_detected==1)+pole_diam/2).*cos(angles(pole_detected==1)) (ranges(pole_detected==1)+pole_diam/2).*sin(angles(pole_detected==1))];
    pole_range_bearing = [ranges(pole_detected==1)+pole_diam/2 (angles(pole_detected==1))];
%
%     pole_coords_xy = [];
%     pole_range_bearing = [];
%
%
%     % do nearest neighbour search to remove double detections of the same
%     % pole
%     for i=1:size(pole_coords_xy_temp,1)
%         xy = pole_coords_xy_temp(i,:);
%         dist = pole_coords_xy_temp - repmat(xy,size(pole_coords_xy_temp,1),1);
%         dist = (dist(:,1).^2 + dist(:,2).^2).^0.5;
%         dist = dist(dist~=0);
%         if min(dist) > pole_diam
%             pole_coords_xy = [pole_coords_xy; pole_coords_xy_temp(i,:)];
%             pole_range_bearing = [pole_range_bearing; pole_range_bearing_temp(i,:)];
%         end
%     end



end
