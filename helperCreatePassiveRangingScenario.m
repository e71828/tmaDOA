function [scene,ownship,theaterDisplay] = helperCreatePassiveRangingScenario(numTargets)
% This is a helper function for generating a scenario for passive ranging
% using a single manuevering sensor. It may be changed or removed in a
% future release without notice.

% Copyright 2018 The MathWorks, Inc.

if nargin
    assert(numTargets == 2,'Only 2 targets allowed');
else
    numTargets = 1;
end

scene = trackingScenario;
scene.StopTime = 75; % Scene duration (seconds)
scene.UpdateRate = 2; % Update rate (Hz)

% Create a platform. Sensors/Emitters lives on a platform.
% Therefore each target is a platform and each ownship is a platform.
ownship = platform(scene);
target = platform(scene);
if numTargets == 2
    target2 = platform(scene);
end


% Define motion for ownship and target.
% The ownship is manuevering. The waypoints are defined in a MAT file.
load ('PassiveSensorManeuveringTrajectory.mat','wp','time');

% Use |waypointTrajectory| to mount the trajectory on the ownship
ownship.Trajectory = waypointTrajectory(wp,time,'Orientation',...
    repmat(quaternion([0 0 0],'rotvecd'),[901 1]));

% The target is moving at a constant velocity and the trajectory can be
% defined using |kinematicTrajectory|
target.Trajectory = kinematicTrajectory('Position',[7e4 2e3 -10e3],...
    'Velocity',[-500/3 -50/3 0]);

if numTargets == 2
    target2.Trajectory = kinematicTrajectory('Position',[5e4 -15e3 -10e3],...
        'Velocity',[-500/3 100/3 0]);
end

% Create an IR signature for the target.
target.Signatures{2} = irSignature('Pattern',500*ones(2,2));

if numTargets == 2
    target2.Signatures{2} = irSignature('Pattern',500*ones(2,2));
end

% Mount an infrared sensor using |irSensor| on the observer to provide 
% angle-only measurements
infraredSensor =  irSensor('SensorIndex',1,...
    'ScanMode', 'No scanning',...
    'NumDetectors',[800 800],....
    'HasElevation',false,...
    'UpdateRate', 2,...
    'HasFalseAlarms',false,...
    'HasINS',true,...
    'FocalLength',500,...
    'HasNoise',true,...
    'MountingAngles',[0 0 0]);

ownship.Sensors{1} = infraredSensor;

if numTargets == 2
    str = 'multiTarget';
else
    str = 'CartEKF';
end
grabFigureFcn = @(fig,scene)helperGrabPassiveRangingDisplay(fig,scene,str);

% Create a theater display
theaterDisplay = helperPassiveRangingDisplay(scene,'XLim',[0 100],'YLim',[-50 50],...
    'PlotErrorMetrics',true,'PlotSigmaBounds',true,'GrabFigureFcn',grabFigureFcn);

end