function [z] = SLAMDataAssociations(x, z_raw)
  %%
  % SLAMDataAssociation
  %
  % INPUTS:
  % - x: state vector
  % - z_raw: observations, each in the form [rane, bearing]
  %
  % OUTPUTS:
  % - z: observations, each in the form [range, bearing, ID]
  %%
  
  if size(x,1) < 4
    z = [z_raw, zeros(size(z_raw,1))];
    return;
  end
  xVehicle = x(1:3);
  xFeatures = x(4:end);
  
  %global xy
  xFeaturesXY = [xFeatures(1:2:end), xFeatures(2:2:end)];
  
  
  % convert range and bearing to XY in vehicle frame
  z_rawXY = [z_raw(:,1).*cos(z_raw(:,2)) z_raw(:,1).*sin(z_raw(:,2))];
  feature_ids = zeros(size(z_raw,1),1);
  
  for i=1:size(z_raw,1)

      % calculate distance between map features and z_raw(i)
      disp(z_raw(i,:))
      
      disp(xVehicle);
      
      rot_mx = [cos(xVehicle(3)) sin(xVehicle(3));
                -sin(xVehicle(3)) cos(xVehicle(3))];
          
      xFeaturesXY_vehicle_frame = xFeaturesXY-repmat(xVehicle(1:2)',size(xFeaturesXY,1),1);
      disp(xFeaturesXY_vehicle_frame);
      
      dist_mapfeature_to_z_i = xFeaturesXY_vehicle_frame - repmat((rot_mx' * z_rawXY(i,:)')',size(xFeaturesXY,1),1);
      
      disp(dist_mapfeature_to_z_i);
      
      %dist_mapfeature_to_z_i = xFeaturesXY-repmat(xVehicle(1:2)',size(xFeaturesXY,1),1) - repmat(z_rawXY(i,:),size(xFeaturesXY,1),1);
      
      dist_mapfeature_to_z_i = (dist_mapfeature_to_z_i(:,1).^2 + dist_mapfeature_to_z_i(:,2).^2).^0.5;
      
      closest_distance = min(dist_mapfeature_to_z_i);
      disp(closest_distance)
      if norm(closest_distance) < 1
          nearest_neighbour_index = find(dist_mapfeature_to_z_i == closest_distance);
          feature_ids(i) = nearest_neighbour_index(1); % use first element found (there might be more than one of them)     
      end
      
  end
  
  z = [z_raw, feature_ids];
end
