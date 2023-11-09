function [tem,tam] = helperPassiveRangingErrorMetrics(ownship,isMSC)
% This is a helper function for passive ranging example using a single sensor
% and may be removed in a future release without notice.

% Copyright 2018 The MathWorks, Inc.

tem = trackErrorMetrics;
tem.ErrorFunctionFormat = 'custom';

% The ownship is a handle object and refers to the same object as does 
% scenario. This helps the function to calculate metrics, which are a
% function of ownship.
if ~isMSC
    tem.EstimationErrorFcn = @(track,truth)rangeAndRateErrorCart(track,truth,ownship);
else
    tem.EstimationErrorFcn = @(track,truth)rangeAndRateErrorMSC(track,truth,ownship);
end
tem.EstimationErrorLabels = {'deltaR','deltaRSigmaP','deltaRSigmaM','deltaRR','deltaRRSigmaP','deltaRRSigmaM'};

tam = trackAssignmentMetrics;
tam.DistanceFunctionFormat = 'custom';
tam.AssignmentDistanceFcn = @assignmentByAttribute;

end

function [rError, rSigmaPlusError, rSigmaMinusError, ...
          rrError,rrSigmaPlusError,rrSigmaMinusError] = rangeAndRateErrorCart(track,truth,ownship)

% sigma values
sigmaValues = [0 -1 1];
n = numel(track.State); % State-size.
trackStateSigmas = track.State + chol(track.StateCovariance)*repmat(sigmaValues,[n 1]);
trackPos = trackStateSigmas(1:2:end,:);
trackVel = trackStateSigmas(2:2:end,:);

% truth
truthPos = truth.Position(:);
truthVel = truth.Velocity(:);

ownPose = pose(ownship);

% Error in range 
rAct = norm(truthPos - ownPose.Position(:));
rEst = sqrt(dot(trackPos - ownPose.Position(:),trackPos - ownPose.Position(:),1));

rError = rEst(1) - rAct;
rSigmaMinusError = rEst(2) - rAct;
rSigmaPlusError = rEst(3) - rAct;

% Error in range-rate
rTruth = truthPos - ownPose.Position(:);
rrTruth = dot(truthVel - ownPose.Velocity(:),rTruth)/rAct;

rTracks = trackPos - ownPose.Position(:);
vTracks = trackVel - ownPose.Velocity(:);
rrTracks = dot(vTracks,rTracks,1);
rrError = rrTracks(1)/rEst(1) - rrTruth;
rrSigmaMinusError = rrTracks(2)/rEst(2) - rrTruth;
rrSigmaPlusError = rrTracks(3)/rEst(3) - rrTruth;

end

function [rError, rSigmaPlusError, rSigmaMinusError, ...
          rrError,rrSigmaPlusError,rrSigmaMinusError] = rangeAndRateErrorMSC(track,truth,ownship)
% range is available as 5th state and range-rate is a function of 6th
% state.
[rAct,rrAct] = trueRangeAndRate(truth,ownship);

% 1 std-dev of states.
deltainvR = sqrt(track.StateCovariance(5,5));
deltadRByR = sqrt(track.StateCovariance(6,6));

% Range and range-rate estimate
range = 1/track.State(5);
rangeRate = track.State(6)/track.State(5);

% Convert 1 std-dev of state to 1 std-dev of range and range-rate using
% linearization
deltaStateValues = [0 -deltainvR deltainvR;0 -deltadRByR deltadRByR];
H = [-range^2 0;-rangeRate*range range];
deltaRandRR = H*deltaStateValues;
deltaR = deltaRandRR(1,:);
deltaRR = deltaRandRR(2,:);

% Formulate outputs
rError = range - rAct;
rSigmaPlusError = range + deltaR(3) - rAct;
rSigmaMinusError = range + deltaR(2) - rAct;

rrError = rangeRate - rrAct;
rrSigmaPlusError = rangeRate + deltaRR(3) - rrAct;
rrSigmaMinusError = rangeRate + deltaRR(2) - rrAct;

end


function [range,rangeRate] = trueRangeAndRate(truth,ownship)
    ownPose = pose(ownship);
    r = truth.Position(:) - ownPose.Position(:);
    range = norm(r);
    v = truth.Velocity(:) - ownPose.Velocity(:);
    rangeRate = dot(v,r)/range;
end

function distance = assignmentByAttribute(track,truth)
    trackOriginationID = track.ObjectAttributes.TargetIndex;
    if trackOriginationID == truth.PlatformID
        distance = 0;
    else
        distance = inf;
    end
end