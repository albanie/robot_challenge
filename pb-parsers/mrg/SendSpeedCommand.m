function [ ] = SendSpeedCommand( vel, omega, moos_channel )
%
% SendSpeedCommand( vel, omega, moos_channel )
%
% INPUTS:
%   vel: Desired linear velocity (m/s).
%   omega: Desired angular velocity (rad/s).
%   moos_channel: String containing the name of the MOOS channel (variable)
%      for which to return the messages (if one exists).
%
% ########################## WARNING ##############################
%
%   This function will cause the Husky to move - do not use unless
%   the Husky is raised off the ground or in a safe environment.
%
% #################################################################
%

% Will Maddern
% February 2015
% Mobile Robotics Group, Oxford University.

% set constants (replicated in Husky driver)
max_vel = 0.5;
max_omega = 0.5;

% verify inputs
if (~(isnumeric(vel) && isscalar(vel) &&~isempty(vel)))
    error('%s - vel must be a non-empty scalar double.', mfilename);
end
if (~(isnumeric(omega) && isscalar(vel) && ~isempty(omega)))
    error('%s - omega must be a non-empty scalar double.', mfilename);
end
if (~(ischar(moos_channel) && ~isempty(moos_channel)))
    error('%s - moos_channel must be a non-empty string.', mfilename);
end

% warn if limits exceeded
if abs(vel) > max_vel
    new_vel = min(max(vel, -max_vel), max_vel);
    warning(['%s - commanded velocity of %0.2f m/s exceeds maximum of '...
        '%0.2f m/s - clamping to %0.2f m/s'], mfilename, vel, ...
        sign(vel)*max_vel, new_vel);
    vel = new_vel;
end
if abs(omega) > max_omega
    new_omega = min(max(omega, -max_omega), max_omega);
    warning(['%s - commanded angular velocity of %0.2f rad/s exceeds '...
        'maximum of %0.2f rad/s - clamping to %0.2f rad/s'], mfilename, ...
        omega, sign(omega)*max_omega, new_omega);
    omega = new_omega;
end

% generate current unix time
timestamp = int64(round((now - datenum( 1970,01,01,00,00,00 )) .* ...
    (24 * 60 * 60)  .* 1.0e6));

% build speed command message
command_builder = ...
    javaMethod('newBuilder', ...
    'mrg.datatypes.protobuf.motion.PbBicycleMotion$pbBicycleMotion');

command_builder = command_builder.setTimestamp(timestamp);
command_builder = command_builder.setLinVelX(vel);
command_builder = command_builder.setAngRateZ(omega);

command = command_builder.build();

% send speed command over mex-moos
mexmoos('NOTIFY', moos_channel, ...
        typecast(command.toByteArray(), 'uint8'));






