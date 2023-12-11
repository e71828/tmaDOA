% Setup
[scene,ownship,theaterDisplay] = helperCreatePassiveRangingScenario;
showScenario(theaterDisplay);
%% Track Using a Range-Parameterized MSC-EKF
% The MSC-EKF approach uses linearization techniques to project covariances
% in the prediction step. The covariance in range at initialization is
% typically high and the state transition dynamics is highly non-linear,
% which can cause issues with filter convergence.
%
% This section demonstrates the use of a Gaussian-sum filter,
% |trackingGSF|, to describe the state with a bank of filters, each
% initialized at different range assumptions. This technique is commonly
% termed as range-parameterization [3].

% Set random seed
rng(2015);

% restart the scene
restart(scene);
release(theaterDisplay);
theaterDisplay.GrabFigureFcn = @(fig,scene)helperGrabPassiveRangingDisplay(fig,scene,'MSCRPEKF');

% Create error and assignment metrics.
[tem, tam] = helperPassiveRangingErrorMetrics(ownship,true);
theaterDisplay.ErrorMetrics = tem;

% Create a tracker using |trackerGNN| and range-parameterized MSC-EKF
% filter initialization.
tracker = trackerGNN('FilterInitializationFcn',@initMSCRPEKF,...
    'AssignmentThreshold',50);

theaterDisplay.HasNonLinearState = true;
theaterDisplay.NonLinearTrackPositionFcn = @getTrackPositionsMSC;

% Initialization for MSC-RPEKF
prevPose = pose(ownship,'true');
lastCorrectionTime = 0;
allTracks = [];

% Advance scenario, simulate detections and track
while advance(scene)
    
    time = scene.SimulationTime;
    truths = platformPoses(scene);
    
    % Generate detections from ownship
    detections = detect(ownship,time);
    
    % Update the input from the ownship i.e. it's maneuver since last
    % correction time.
    currentPose = pose(ownship,'true');
    dT = time - lastCorrectionTime;
    observerManeuver = calculateManeuver(currentPose,prevPose,dT);
    
    % Get each filter from the trackingGSF property TrackingFilters using
    % the |getTrackFilterProperties| function of the tracker.
    for i = 1:numel(allTracks)
        trackingFilters = getTrackFilterProperties(tracker,allTracks(i).TrackID,'TrackingFilters');
        % Set the ObserverInput for each tracking filter
        for m = 1:numel(trackingFilters{1})
            trackingFilters{1}{m}.ObserverInput = observerManeuver;
        end
    end
    
    % Pass detections to tracker
    if ~isempty(detections)
        lastCorrectionTime = time;
        % Store the previous pose to calculate maneuver
        prevPose = currentPose;
        [tracks,~,allTracks] = tracker(detections,time);
    elseif isLocked(tracker)
        tracks = predictTracksToTime(tracker,'confirmed',time);
    end
    
    % Update error and assignment metrics
    tam(tracks,truths);
    [trackIDs, truthIDs] = currentAssignment(tam);
    tem(tracks,trackIDs,truths,truthIDs);
    
    % Update display
    theaterDisplay.NonLinearStateInput = currentPose.Position(:);
    theaterDisplay(detections,tracks,tracker);
end


%%
% *|initMSCRPEKF|*
% Initialize a range-parameterized MSC-EKF from an angle-only detection
function filter = initMSCRPEKF(detection)
% A range-parameterized MSC-EKF can be defined using a Gaussian-sum filter
% (trackingGSF) containing multiple |trackingMSCEKF| as TrackingFilters.

% Range-parametrization constants
rMin = 3e3;
rMax = 8e3;
numFilters = 10;
rho = (rMax/rMin)^(1/numFilters);
Cr = 2*(rho - 1)/(rho + 1)/sqrt(12);
indFilters = cell(numFilters,1);
for i = 1:numFilters
    range = rMin/2*(rho^i + rho^(i-1));
    rangeSigma = Cr*range;
    % Use initcvmscekf function to create a trackingMSCEKF with provided
    % range and rangeSigma.
    indFilters{i} = initcvmscekf(detection,[range rangeSigma]);
    % Update the velocity covariance of each filter.
    indFilters{i}.StateCovariance(2:2:end,2:2:end) = 400*indFilters{i}.StateCovariance(2:2:end,2:2:end);
end
filter = trackingGSF(indFilters);

end

%% 
% *Utility functions*
% *|calculateManeuver|*
function maneuver = calculateManeuver(currentPose,prevPose,dT)
% Calculate maneuver i.e. 1st order+ motion of the observer. This is
% typically obtained using sensors operating at much higher rate.
v = prevPose.Velocity;
prevPos = prevPose.Position;
prevVel = prevPose.Velocity;
currentPos = currentPose.Position;
currentVel = currentPose.Velocity;

% position change apart from constant velocity motion
deltaP = currentPos - prevPos - v*dT;
% Velocity change
deltaV = currentVel - prevVel;

maneuver = zeros(6,1);
maneuver(1:2:end) = deltaP;
maneuver(2:2:end) = deltaV;
end

%%
% *|getTrackPositionsMSC|*
function [pos,cov] = getTrackPositionsMSC(tracks,observerPosition)

if isstruct(tracks) || isa(tracks,'objectTrack')
    % Track struct
    state = [tracks.State];
    stateCov = cat(3,tracks.StateCovariance);
elseif isa(tracks,'trackingMSCEKF')
    % Tracking Filter
    state = tracks.State;
    stateCov = tracks.StateCovariance;
end

% Get relative position using measurement function.
relPos = cvmeasmsc(state,'rectangular');

% Add observer position
pos = relPos + observerPosition;
pos = pos';

if nargout > 1
    cov = zeros(3,3,numel(tracks));
    
    for i = 1:numel(tracks)
        % Jacobian of position measurement
        jac = cvmeasmscjac(state(:,i),'rectangular');
        cov(:,:,i) = jac*stateCov(:,:,i)*jac';
    end
end

end