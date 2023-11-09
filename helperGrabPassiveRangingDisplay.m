function figures = helperGrabPassiveRangingDisplay(scene, fig, filterType)
% This is a helper function for capturing figures during publishing of 
% Passive ranging example and may be removed in a future release without
% notice.

% Copyright 2018 The MathWorks, Inc.

figures = {};

if ~ishandle(fig)
    return;
end

simTime = scene.SimulationTime;
stopTime = scene.StopTime;
dT = 1/scene.UpdateRate;

switch filterType
    case 'CartEKF'
        snapTimes = [1.5 stopTime];
        snapNum = find(abs(simTime - snapTimes) < dT/2, 1);
        if isempty(snapNum)
            return;
        end
    case 'CartEKF2'
        snapTimes = [5 stopTime];
        snapNum = find(abs(simTime - snapTimes) < dT/2);
        if isempty(snapNum)
            return;
        else
            snapNum = snapNum + 2;
        end
    case 'MSCEKF'
        snapTimes = [15 stopTime];
        snapNum = find(abs(simTime - snapTimes) < dT/2);
        if isempty(snapNum)
            return;
        else
            snapNum = snapNum + 4;
        end
    case 'MSCRPEKF'
        snapTimes = [2 stopTime];
        snapNum = find(abs(simTime - snapTimes) < dT/2);
        if isempty(snapNum)
            return;
        else
            snapNum = snapNum + 6;
        end
    case 'multiTarget'
        % hit otherwise.
        snapTimes = stopTime;
        snapNum = find(abs(simTime - snapTimes) < dT/2);
        if isempty(snapNum)
            return;
        else
            snapNum = snapNum + 9;
        end
end

[fig, clnup1, clnup2] = copyFigure(fig); %#ok<ASGLU>

switch snapNum
    case 1
        % High range covariance in CartEKF.
        snapshot1 = copy(fig.Children(5));
        xlim(snapshot1,[0 10]);
        ylim(snapshot1,[0 0.3]);
        snapshot1.DataAspectRatioMode = 'auto';
        snapshot1.Position = [0.1300    0.1100    0.7750    0.8150];
        figures = {snapshot1};
    case 3
        % Covariance collapse
        annotation(fig,'textbox',...
            [0.6500    0.7500    0.1085    0.0611],...
            'String',{'High', 'initial','range','covariance'},'FitBoxToText',...
            'on','FontSize',8,'FontWeight','bold');
        
        fig.Children(3).XLim = [0.5 3];
        fig.Children(1).XLim = [0.5 3];
        
        annotation(fig,'textbox',...
            [0.8400    0.65    0.1302    0.0611],...
            'String',{'Covariance collapsed','before maneuver'},...
            'FitBoxToText','on','FontWeight','bold','FontSize',8);
        annPane = findall(fig,'Type','AnnotationPane');
        fig.Children(5).XLim = [0 10];
        fig.Children(5).YLim = [-5 5];
        fig.Children(5).DataAspectRatioMode = 'auto';
        toCopyObjects = [fig.Children;annPane];

        snapshot1 = copy(toCopyObjects);
        figures = {snapshot1};
    
     case 4
        % Slow convergence
        fig.Children(1).XLim = [0 stopTime];
        fig.Children(1).YLimMode = 'auto';
        fig.Children(1).Children(1).YPositiveDelta = zeros(0,1);
        fig.Children(1).Children(1).YNegativeDelta = zeros(0,1);
        fig.Children(3).Children(1).YPositiveDelta = zeros(0,1);
        fig.Children(3).Children(1).YNegativeDelta = zeros(0,1);
        
        fig.Children(3).XLim = [0 stopTime];
        fig.Children(3).YLimMode = 'auto';
        l = fig.Children(3).Legend;
        l.AutoUpdate = 'off';
        text(fig.Children(3),4.5,-0.5,'Slow convergence','FontWeight','bold');
        snapshot = copy(fig.Children);
        figures = {snapshot};
        
    case 5
        % No covariance collapse
        annotation(fig,'textbox',...
            [0.65    0.55    0.1302    0.0611],...
            'String',{'Covariance maintenance', 'before maneuver'},...
            'FitBoxToText','on','FontSize',8,'FontWeight','bold');
        annPane = findall(fig,'Type','AnnotationPane');
        fig.Children(3).YLim = [-9 9];
        toCopyObjects = [annPane;fig.Children];

        snapshot1 = copy(toCopyObjects);
        figures = {snapshot1};
        
    case 7
        % Range-parameterization
        fig.Children(5).Position = [0.1300 0.1100 0.7750 0.8150];
        fig.Children(4).Location = 'northeast';
        fig.Children(5).Title.String = 'Range-Parameterized Representation of State';
        snapshot1 = copy([fig.Children(5);fig.Children(4)]);
        figures = {snapshot1};
        
    otherwise
        % Final results in second half of scenario
        fig.Children(1).XLim = [25 stopTime];
        fig.Children(3).XLim = [25 stopTime];
        
        snapshots = copy(fig.Children);
        figures = {snapshots};
end

end

function [newFig, clnup1, clnup2] = copyFigure(fig)
state = get(0,'DefaultFigureVisible');
clnup1 = onCleanup(@()set(0,'DefaultFigureVisible',state));
set(0,'DefaultFigureVisible','off');

newFig = figure;
clnup2 = onCleanup(@()close(newFig));
copyobj(fig.Children,newFig);
end

