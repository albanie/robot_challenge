function [xPred, PPred] = SLAMPrediction(u, x, P)
  %%
  % SLAMPrediction
  %
  % INPUTS:
  % - u: motion estimate (from odometry or control inputs)
  % - x: state vector
  % - P: covariance matrix
  %
  % OUTPUTS:
  % - xPred: state vector updated with motion prediction
  % - PPred: covariance updated with motion prediction
  %%

  xVehicle = x(1:3);
  xMap = x(4:end);

  xVehiclePred = tcomp(xVehicle, u);

  % TODO: Update to make UTrue TIME VARYING - at the moment uncertainty
  % remains constant when the robot is still.
  
  % Covariances
  % PPredvv: covariance vehicle pose to vehicle pose
  UTrue = diag([0.01,0.01,1.5*pi/180]).^2;
  UEst = 0.5*UTrue;
  PPredvv = J1(xVehicle, u) * P(1:3, 1:3) * J1(xVehicle, u)' + ...
      J2(xVehicle, u) * UEst * J2(xVehicle, u)';

  % PPredvm: covariance vehicle pose to map
  PPredvm = J1(xVehicle, u) * P(1:3, 4:end);

  % PPredmm: covariance map to map
  PPredmm = P(4:end, 4:end);

  xPred = [xVehiclePred; xMap];
  PPred = [PPredvv PPredvm; PPredvm' PPredmm];

end
