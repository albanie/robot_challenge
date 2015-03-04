function [omega,vel] = BasicController(vehiclepose,target)

% Given a vehiclepose (x,y,theta) and a target (x',y'), derive the angular
% velocity to feed in before moving forward.

if target(2)-vehiclepose(2)>0 | target(1)-vehiclepose(1)>0
    omega = atan((target(2)-vehiclepose(2))/(target(1)-vehiclepose(1)))-vehiclepose(3);
else
    omega = pi + atan((target(2)-vehiclepose(2))/(target(1)-vehiclepose(1)))-vehiclepose(3);

%vel = 0.1;

% using the distance between vehiclepose and target, derive the velocity.

n = norm(target-vehiclepose(1:2));

switch n
    case n < 1
        vel = 0.1
    case n=>1 & n<3
        vel = 0.2
    case n => 3
        vel = 0.4
end

    