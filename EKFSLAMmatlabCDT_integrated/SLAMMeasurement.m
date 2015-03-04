function [xEst, PEst] = SLAMMeasurement(z, x, P)
  %%
  % SLAMMeasurement
  %
  % INPUTS:
  % - z: observations, each in the form [range, bearing, ID]
  % - x: state vector
  % - P: covariance matrix
  %
  % OUTPUTS:
  % - xEst: updated state vector
  % - PEst: update covariance matrix

  REst = 0.25 * diag([1.1, 5 * pi/180.0]).^0.2;
  xEst = x;
  
  for i = 1:size(z,1)
    FeatureIndex = GetFeatureIndex(z(i,3));
    xVehicle = xEst(1:3);
    if FeatureIndex > 3
      xFeature = xEst(FeatureIndex:FeatureIndex+1);
    
      % Transform feature coordinates into range/bearing
      zPred = DoObservation(xVehicle, xFeature);

      [jHxv, jHxf] = GetObsJacs(xVehicle, xFeature);

      jH = zeros(2, length(xEst));
      jH(:, FeatureIndex:FeatureIndex+1) = jHxf;
      jH(:, 1:3) = jHxv;
            
      Innov =  z(i,1:2)'-zPred; 
      Innov(2) = AngleWrap(Innov(2));

      S = jH*P*jH'+REst;
      W = P*jH'*inv(S);
      xEst = xEst + W * Innov;
      
      P = P-W*S*W';

      P = 0.5 * (P + P');

    else
      disp('Adding new landmark');
      nStates = length(xEst);
      xFeature = xVehicle(1:2) + ...
          [z(i, 1) * cos(z(i, 2) + xVehicle(3)); z(i, 1) * sin(z(i, 2) + xVehicle(3))];
      xEst = [xEst; xFeature];
      [jGxv, jGz] = GetNewFeatureJacs(xVehicle, z(i,1:2));

      M = [eye(nStates), zeros(nStates,2);
          jGxv zeros(2, nStates-3), jGz];

      P = M * blkdiag(P, REst) * M';

    end

  end
  
  PEst = P;
end

function featureIndex = GetFeatureIndex(featureID)
  featureIndex = 3 + 2 * featureID - 1;
end

function z = DoObservation(xVeh, xFeature)
  Delta = xFeature-xVeh(1:2);
  z = [norm(Delta);
      atan2(Delta(2),Delta(1))-xVeh(3)];
  z(2) = AngleWrap(z(2));
end

function [jHxv,jHxf] = GetObsJacs(xPred, xFeature)
  jHxv = zeros(2,3);jHxf = zeros(2,2);
  Delta = (xFeature-xPred(1:2));
  r = norm(Delta);
  jHxv(1,1) = -Delta(1) / r;
  jHxv(1,2) = -Delta(2) / r;
  jHxv(2,1) = Delta(2) / (r^2);
  jHxv(2,2) = -Delta(1) / (r^2);
  jHxv(2,3) = -1;
  jHxf(1:2,1:2) = -jHxv(1:2,1:2);
end

function [jGx,jGz] = GetNewFeatureJacs(Xv, z)
  x = Xv(1,1);
  y = Xv(2,1);
  theta = Xv(3,1);
  r = z(1);
  bearing = z(2);
  jGx = [ 1   0   -r*sin(theta + bearing);
      0   1   r*cos(theta + bearing)];
  jGz = [ cos(theta + bearing) -r*sin(theta + bearing);
      sin(theta + bearing) r*cos(theta + bearing)];

end
