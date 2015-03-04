function [range_out bearing_out] = TranslateLaserScan(ranges, bearings, d)

  x = ranges .* cos(bearings);
  y = ranges .* sin(bearings);

  range_out = sqrt((x+d).^2 + y.^2);
  bearing_out = atan2(y, x + d);

end
