function helperRunPassiveRangingSimulation(scene,theaterDisplay,filterInitializationFcn,plotSigmaBounds)
% This is a helper function and may be removed in a future release without
% notice. This function runs the passive simulation by using a
% |trackerGNN| with filterInitializationFcn.

% Copyright 2018 The MathWorks, Inc.

rng(0);

% restart the scene.
restart(scene);
release(theaterDisplay);

if nargin < 4
    plotSigmaBounds = true;
end
theaterDisplay.PlotSigmaBounds = plotSigmaBounds;

% Check if the filter is in Modified Spherical Coordinates
funcName = func2str(filterInitializationFcn);
isMSC = strcmpi(funcName,'initMSCEKF')  || strcmpi(funcName,'initMSCPF') || strcmpi(funcName,'initcvmscekf') || strcmpi(funcName,'initMSCRPEKF');
theaterDisplay.HasNonLinearState = isMSC;

if isMSC
    theaterDisplay.NonLinearTrackPositionFcn = @getTrackPositionsMSC;
end

% Check if the filter is a Gaussian-sum filter.
isGSF = strcmpi(funcName,'initMSCRPEKF');

tracker = trackerGNN('FilterInitializationFcn',filterInitializationFcn,'AssignmentThreshold',50,'MaxNumTracks',5);
ownship = scene.Platforms{1};
prevPose = pose(ownship,'true');

[tem, tam] = helperPassiveRangingErrorMetrics(ownship,isMSC);

theaterDisplay.ErrorMetrics = tem;

% Initialize allTracks and last time of correction
allTracks = [];
lastCorrectionTime = 0;

while advance(scene)
    % Current simulation time
    time = scene.SimulationTime;
    truths = platformPoses(scene);
    
    % Generate detections from ownship
    detections = detect(ownship,time);
    
    if isMSC
        % The State is defined in a relative coordinate frame in the frame of
        % observer. To update the state of the track, we must specify the
        % maneuver performed by the observer. The maneuever is defined as
        % motion of the observer higher than first order.
        currentPose = pose(ownship,'true');
        dT = time - lastCorrectionTime;
        observerManuever = calculateManeuver(currentPose,prevPose,dT);
        % Set the ObserverInput using setTrackFilterProperties function of the
        % tracker.
        for i = 1:numel(allTracks)
            if ~isGSF
                % For a MSCEKF, you can set the property directly
                % using the ObserverInput.
                setTrackFilterProperties(tracker,allTracks(i).TrackID,'ObserverInput',observerManuever);
            else
                % For a Gaussian-sum filter, first get the trackingFilter
                % objects from the filter property.
                trackingFilters = getTrackFilterProperties(tracker,allTracks(i).TrackID,'TrackingFilters');
                % Set the observer input for each tracking filter
                for m = 1:numel(trackingFilters{1})
                    trackingFilters{1}{m}.ObserverInput = observerManuever;
                end
            end
        end
    end
    
    % Track objects using the tracker
    if ~isempty(detections)
        [tracks,~,allTracks] = tracker(detections,time);
        if isMSC
            prevPose = currentPose;
        end
        lastCorrectionTime = time;
    elseif isLocked(tracker)
        tracks = predictTracksToTime(tracker,'all',time);
    end
    
    tracksToPlot = tracks;
    
    % Update error and assignment metrics
    tam(tracks,truths);
    [trackIDs, truthIDs] = currentAssignment(tam);
    tem(tracks,trackIDs,truths,truthIDs);
    
    % Store off previous pose of the observer to calculate the maneuever
    if isMSC
        theaterDisplay.NonLinearStateInput = currentPose.Position(:);
    end
    theaterDisplay(detections,tracksToPlot,tracker);
end
end
%%
% *|calculateManeuver|*
function manuever = calculateManeuver(currentPose,prevPose,dT)
% Calculate maneuver i.e. 2nd order plus motion of the observer.
    v = prevPose.Velocity;
    prevPos = prevPose.Position;
    prevVel = prevPose.Velocity;
    currentPos = currentPose.Position;
    currentVel = currentPose.Velocity;

    deltaP = currentPos - prevPos - v*dT;
    deltaV = currentVel - prevVel;

    manuever = zeros(6,1);
    manuever(1:2:end) = deltaP;
    manuever(2:2:end) = deltaV;
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

