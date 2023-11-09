classdef (Hidden) helperPassiveRangingDisplay < matlab.System
    % helperPassiveRangingDisplay Creates and updates a theaterPlot and
    % range-metrics plot display for passive ranging using a single sensor
    % example.
    %
    % This is a helper class for example purposes and may be removed or
    % modified in the future without notice.
    
    % Copyright 2018 The MathWorks, Inc.
    
    properties(Dependent, SetAccess=private)
        TheaterPlot
    end
    
    properties(Nontunable)
        %XLim Minimum and maximum distance in the x-axis
        XLim = [-100 100]
        
        %YLim Minimum and maximum distance in the y-axis
        YLim = [-100 100]
        
        %ZLim Minimum and maximum distance in the z-axis
        ZLim = [-50 1]
        
        %Movie Filename for movie to be written by writeMovie
        Movie = ''
    end

    properties(Nontunable)
        DistanceUnits = 'km'
    end    
    properties(Constant, Hidden)
        DistanceUnitsSet = matlab.system.StringSet({'m','km'});
    end
    
    properties (Nontunable)
        %TrackPositionSelector Select x and y from the state vector
        %  For example, if the state is [x;vx;y;vy;z;vz], [x;y;z] = H * state
        %  Where H = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]
        TrackPositionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; 
        
        %TrackVelocitySelector Select vx and vy from the state vector
        %  For example, if the state is [x;vx;y;vy;z;vz], [vx;vy;vz] = H * state
        %  Where H = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1]
        TrackVelocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 1];
    end
    
    properties
        NonLinearTrackPositionFcn;
        NonLinearTrackVelocityFcn;
        NonLinearStateInput;
    end
    
    properties
        AssignmentMetrics
        ErrorMetrics
    end
    
    properties (Nontunable)
        PlotErrorMetrics (1, 1) logical;
        PlotSigmaBounds (1, 1) logical;
    
        plotAssignmentMetrics (1, 1) logical;
    end
    
    properties 
        HasNonLinearState (1, 1) logical = false;
    end
    
    properties (Hidden, Nontunable)
        %GrabFigureFcn  Set to a function handle accepting the current
        %scenario and returning a figure's children or a cell array of
        %children from multiple figures
        GrabFigureFcn
    
        %UseGetFrame Set to true to capture frames using getframe(h),
        %otherwise set to false to capture frames using print -RGBImage
        UseGetFrame (1, 1) logical = true
    end
    
    properties(Access=private)
        pScale = 1
        pScenario
        pFig = NaN
        pTheaterPlot
        pPlatformPlotters
        pTargetPlotter
        pPathPlotter
        pTxBeamPlotters = {}
        pRxBeamPlotters = {}
        pDetectionPlotters
        pDetectionPlottersRay
        pTrackPlotter
        pSimulinkUIToolbar
        pBlockName
        pIsLegendOn
        pWriteMovie
        pFrames
        pNumFrames
        pGrabs = {}
        pShowGrabFigs
        pShowLegendGrab
        pFilterType
        pFilterPlotter
        pRangeAxes;
        pRangeRateAxes;
        pRangePlotter;
        pRangeSigmaPlotter;
        pRangeRatePlotter;
        pRangeRateSigmaPlotter;
        pRangePlatformIDs;
    end
    
    properties(Constant, Access=private)
        % BufferSize
        BufferSize = 500
    end
    
    methods
        function val = get.TheaterPlot(obj)
            val = obj.pTheaterPlot;
        end
        
        function obj = helperPassiveRangingDisplay(scenario, varargin)
            % Constructor
            setProperties(obj,numel(varargin),varargin{:});
            
            obj.pScenario = scenario;
            
            % Create a new figure
            % Hide the figure while we build the theater plot
            hfig = figure('Visible','off');
            hfig.Units = 'normalized';
            hfig.Position = [0.1 0.1 0.6 0.6];
                
            obj.pIsLegendOn = true;
            
            obj.pFig = hfig;
            
            if strcmpi(obj.DistanceUnits,'km')
                obj.pScale = 1000;
            end
            
            initTP(obj);
            plotPlatforms(obj);
            
            grabLegend(obj);
            if ~isPublishing()
                hfig.Visible = 'on';
            end
        end
        
        function flag = isOpen(obj)
            flag = ishghandle(obj.pFig);
        end
        
        function varargout = writeMovie(obj,loopCount)
            if obj.pWriteMovie
                rate = obj.pScenario.UpdateRate;
                
                % Assemble file name w/ extension
                [dname,fname,ext] = fileparts(obj.Movie);
                if isempty(ext)
                    fmt = 'mp4';
                else
                    fmt = ext(2:end);
                end
                fname = fullfile(dname,fname);
                
                if strcmpi(fmt,'gif')
                    % Write animated GIF
                    if nargin<2
                        loopCount = inf;
                    end
                    
                    fname = [fname '.' fmt];
                    
                    writeAnimatedGIF(fname, obj.pFrames, obj.pNumFrames, rate, loopCount);
                    
                    % If publishing, close the figure, since the user is probably setting
                    % the animated GIF as the published figure.
                    if isPublishing()
                        close(obj.pFig);
                    end
                else
                    % Write video
                    if nargin<2
                        loopCount = 'MPEG-4';
                    end
                    profile = loopCount;
                    
                    localWriteVideo(fname, obj.pFrames, obj.pNumFrames, rate, profile);
                end
                
                if nargout
                    varargout = {fname};
                else
                    varargout = {};
                end
            else
                if nargout
                    varargout = {''};
                else
                    varargout = {};
                end
            end
        end
        
        function showGrabs(obj, grabNums, showLegend)
            if ~isempty(obj.pShowGrabFigs)
                close(obj.pShowGrabFigs);
            end
            
            if nargin<3
                showLegend = true;
            end
            figs = [];
            
            for m = 1:numel(grabNums)
                thisGrab = obj.pGrabs{grabNums(m)};
                for n = 1:numel(thisGrab)
                    fig = figure;
                    fig.Units = 'normalized';
                    fig.Position = [0.1 0.1 0.5 0.5];
                    copyobj(thisGrab{n},fig);
                    
                    if ~showLegend
                        l = findobj(fig,'Type','Legend');
                        for i = 1:numel(l)
                            l(i).Visible = 'off';
                        end
                    end
                end
                
                if isempty(figs)
                    figs = fig;
                else
                    figs = [figs fig]; %#ok<AGROW>
                end
            end
            
            obj.pShowGrabFigs = figs;
        end
        
        function grabLegend(obj)
            fig = obj.pFig;
            if ~isempty(fig)
                snapshot = obj.pTheaterPlot.Parent;
                obj.pShowLegendGrab = [snapshot;snapshot.Legend];
            end
        end
        
        function showLegend(obj)
            if ~isempty(obj.pShowGrabFigs)
                close(obj.pShowGrabFigs);
            end
            
            fig = [];
            
            thisGrab = obj.pShowLegendGrab;
            if ~isempty(thisGrab)
                fig = figure;
                h = copyobj(thisGrab,fig);
                
                % Hide main axes (only show legend)
                iLeg = strcmpi('legend',get(h,'Type'));
                hLeg = h(iLeg);
                hLeg.Location = 'none';
                hAx = h(~iLeg);
                posOrig = hAx.Position;
                pos = posOrig;
                pos(1:2) = 2;
                hAx.Position = pos;
                
                hLeg.Units = 'normalized';
                fig.Units = 'normalized';
                % Resize figure to size of legend
                pos = hLeg.Position;
                aspectRatio = pos(4)/pos(3);
                width = 1.1*pos(3);
                fig.Position = [0.1 0.1 width aspectRatio*width];
                hLeg.Position = [0.1 0.1 0.8 aspectRatio*0.8];
            end
            
            obj.pShowGrabFigs = fig;
        end
        
        function showScenario(obj)
            if isPublishing()
                f = figure;
                f.Units = 'normalized';
                f.Position = [0.1 0.1 0.6 0.6];
                copyobj(obj.pShowLegendGrab,f);
                f.Children(2).Position = [0.1 0.1 0.8 0.8];
                hLeg = f.Children(1);
                hLeg.Location = 'best';
                hLeg.AutoUpdate = 'on';
                for i = 1:numel(f.Children(2).Children)
                    h = f.Children(2).Children(i);
                    if strcmpi(h.DisplayName,'Detections 1')
                        h.delete;
                        break;
                    end
                end
                f.Children(2).Title.String = 'Sensor and Target Trajectories';
                obj.pShowGrabFigs = f;
            end
        end
    end
    
    methods(Access=protected)
        function setupImpl(obj,varargin)
            % Update in case anything has changed since display was
            % constructed
            initTP(obj);
            plotPlatforms(obj);
            
            hfig = obj.pFig;
            hax = obj.pTheaterPlot.Parent;
            
            [~, dets, ~, tracks,tracker] = parseStepInputs(obj, varargin{:});
            
            % Plot tracks
            if isstruct(tracks) || isa(tracks,'objectTrack')
                trkClr = lines(2); trkClr = trkClr(end,:);
                if isempty(obj.pTrackPlotter) || ~isvalid(obj.pTrackPlotter)
                    obj.pTrackPlotter = trackPlotter(obj.pTheaterPlot,'DisplayName','Tracks','MarkerFaceColor',trkClr);
                end
            end
            if isempty(dets) && ~isempty(tracker)
                error('Must pass a detection on setup of display to plot filters');
            elseif ~isempty(tracker)
                filt = tracker.FilterInitializationFcn(dets{1});
                if isa(filt,'trackingPF')
                    obj.pFilterType = 'trackingPF';
                elseif isa(filt,'trackingGSF')
                    obj.pFilterType = 'trackingGSF';
                elseif isa(filt,'trackingIMM')
                    obj.pFilterType = 'trackingIMM';
                else
                    obj.pFilterType = '';
                end
            end
            
            if strcmpi(obj.pFilterType,'helperTrackingMSCPF')
                if ~isempty(obj.pFilterPlotter)
                    obj.pFilterPlotter.delete;
                end
                wasHeld = ishold(hax);
                hold(hax,'on');
                l = legend;
                l.AutoUpdate = 'on';
                obj.pFilterPlotter = plot3(hax,NaN,NaN,NaN,'b.');
                l.String{end} = 'Particles';
                if ~wasHeld
                    hold(hax,'off');
                end
            elseif strcmpi(obj.pFilterType,'trackingGSF') || strcmpi(obj.pFilterType,'trackingIMM')
                if ~isempty(obj.pFilterPlotter)
                    obj.pFilterPlotter.delete();
                end
                obj.pFilterPlotter = trackPlotter(obj.pTheaterPlot,...
                    'DisplayName','Individual Filters', ...
                    'MarkerEdgeColor', trkClr, ...
                    'MarkerFaceColor', 'none','Marker','o');
            elseif ~isempty(obj.pFilterPlotter) && isvalid(obj.pFilterPlotter)
                obj.pFilterPlotter.delete;
            end
            
            if ~isempty(obj.pRangePlotter)
               for i = 1:numel(obj.pRangePlotter)
                   obj.pRangePlotter(i).XData = nan;
                   obj.pRangePlotter(i).YData = nan;
                   obj.pRangePlotter(i).YPositiveDelta = nan;
                   obj.pRangePlotter(i).YNegativeDelta = nan;
                   obj.pRangeRatePlotter(i).XData = nan;
                   obj.pRangeRatePlotter(i).YData = nan;
                   obj.pRangeRatePlotter(i).YPositiveDelta = nan;
                   obj.pRangeRatePlotter(i).YNegativeDelta = nan;
               end
            end
            
            legend(hax,'AutoUpdate','off')
            
            if isPublishing
                obj.pFig.Visible = 'off';
            else
                hfig.Visible = 'on';
            end
            
            obj.pWriteMovie = ~isempty(obj.Movie);
            obj.pNumFrames = 0;
            grabLegend(obj);
        end

        function stepImpl(obj,varargin)
            [signals, dets, configs, tracks, tracker] = parseStepInputs(obj, varargin{:});
            
            plotPlatforms(obj);
            
            if isa(signals,'fusion.internal.interfaces.BaseEmission')
                plotTXBeams(obj, signals);
            end
            
            if isstruct(configs)
                plotRXBeams(obj, configs);
            end
            
            if iscell(dets)
                plotDetections(obj, dets);
            end
            
            if isstruct(tracks) && isfield(tracks,'TrackID') || isa(tracks,'objectTrack') && isprop(tracks(1),'TrackID')
                plotTracks(obj, tracks);
            end
            
            if ~isempty(tracker) && isfield(tracks,'TrackID') || isa(tracks,'objectTrack')
                plotFilters(obj,tracker,tracks)
            end
            
            if ~isempty(tracks) && isfield(tracks,'TrackID') || isa(tracks,'objectTrack')
                plotRangeMetrics(obj,tracks);
            end
            
            if obj.pWriteMovie
                % Capture current frame to write out movie later.
                iFrame = obj.pNumFrames+1;
                if obj.UseGetFrame
                    f = getframe(obj.pFig);
                    
                    % Allocate blocks of memory at a time
                    if obj.pNumFrames==0
                        obj.pFrames = repmat(f,1,obj.BufferSize);
                    elseif iFrame>numel(obj.pFrames)
                        obj.pFrames = [obj.pFrames repmat(f,1,obj.BufferSize)];
                    end
                    
                    obj.pFrames(iFrame) = f;
                else
                    im = print('-RGBImage','-opengl','-r0');

                    % Allocate blocks of memory at a time
                    if obj.pNumFrames==0
                        obj.pFrames = zeros([size(im) obj.BufferSize],'like',im);
                    elseif iFrame>size(obj.pFrames,4)
                        obj.pFrames = cat(obj.pFrames,zeros([size(im) obj.BufferSize],'like',im),4);
                    end
                    
                    obj.pFrames(:,:,:,iFrame) = im;
                end
                obj.pNumFrames = iFrame;
            end
            
            if ~isempty(obj.GrabFigureFcn) && ishghandle(obj.pFig)
                grabbed = feval(obj.GrabFigureFcn, obj.pScenario, obj.pFig);
                if iscell(grabbed)
                    if ~isempty(grabbed)
                        obj.pGrabs = {obj.pGrabs{:} grabbed}; %#ok<CCAT>
                    end
                else
                    obj.pGrabs = {obj.pGrabs{:} grabbed}; %#ok<CCAT>
                end
            end
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.pShowLegendGrab = obj.pShowLegendGrab;
            
            if isLocked(obj)
                s.pFig = obj.pFig;
                s.pTheaterPlot = obj.pTheaterPlot;
                s.pPlatformPlotters         = obj.pPlatformPlotters;
                s.pTargetPlotter         = obj.pTargetPlotter;
                s.pTxBeamPlotters        = obj.pTxBeamPlotters;
                s.pRxBeamPlotters        = obj.pRxBeamPlotters;
                s.pDetectionPlotters     = obj.pDetectionPlotters;
                s.pDetectionPlottersRay     = obj.pDetectionPlottersRay;
                s.pTrackPlotter         = obj.pTrackPlotter;
                s.pIsLegendOn           = obj.pIsLegendOn;
                s.pSimulinkUIToolbar    = saveobj(obj.pSimulinkUIToolbar);
                s.pWriteMovie = obj.pWriteMovie;
                s.pGrabs = obj.pGrabs;
                s.pShowGrabFigs = obj.pShowGrabFigs;
            end
        end
        
        function loadObjectImpl(obj,s,wasLocked)
            obj.pShowLegendGrab = s.pShowLegendGrab;
            if wasLocked
                obj.pFig = s.pFig;
                obj.pTheaterPlot = s.pTheaterPlot;
                obj.pPlatformPlotters           = s.pPlatformPlotters;
                obj.pTargetPlotter         = s.pTargetPlotter;
                obj.pTxBeamPlotters          = s.pTxBeamPlotters;
                obj.pRxBeamPlotters          = s.pRxBeamPlotters;
                obj.pDetectionPlotters       = s.pDetectionPlotters;
                obj.pDetectionPlottersRay       = s.pDetectionPlottersRay;
                obj.pTrackPlotter           = s.pTrackPlotter;
                obj.pIsLegendOn             = s.pIsLegendOn;
                obj.pSimulinkUIToolbar      = loadobj(s.pSimulinkUIToolbar);
                obj.pWriteMovie = s.pWriteMovie;
                obj.pGrabs = s.pGrabs;
                obj.pShowGrabFigs = s.pShowGrabFigs;
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    methods (Access = private)
        function initTP(obj)
            wasFigureClosed = (isempty(obj.pFig) || ~ishghandle(obj.pFig));
            if wasFigureClosed
                % Create a new figure
                hfig = figure;
                hfig.Units = 'normalized';
                hfig.Position = [0.1 0.1 1 1];
                % Hide the figure while we build the theater plot
                set(hfig,'Visible','off')
                obj.pIsLegendOn = true;

                obj.pFig = hfig;
            end
            
            % Create theaterPlot before the toolbar because clf clears toolbars
            isTPNeeded = (isempty(obj.pTheaterPlot) || wasFigureClosed);
            if isTPNeeded
                clf(obj.pFig);
                hax = axes(obj.pFig);
                set(hax,'YDir','reverse','ZDir','reverse');
                view(hax,-90,90);

                % Add ground
                xx = 2*obj.XLim;
                yy = 2*obj.YLim;
                h = patch(xx([1 end end 1 1]),yy([1 1 end end 1]),zeros(1,5),'k');
                h.FaceColor = 0.8*[1 1 1];

                tp = theaterPlot('Parent',hax,'XLimits',obj.XLim,'YLimits',obj.YLim,'ZLimits',obj.ZLim);
                obj.pTheaterPlot = tp;
                h = legend;
                h.String{1} = 'Ground';
                
                % Find the platforms defined in this scenario
                [iPlats,iTgts] = findPlatforms(obj.pScenario);
                
                clrs = lines(numel(iPlats)+1); % +1 for track colors
                clrs(2,:) = []; % This color is reserved for tracks

                wasHeld = ishold(hax);
                if ~wasHeld
                    hold(hax,'on');
                end
                for m = 1:numel(iPlats)
                    idx = iPlats(m);
                    thisPlatform = obj.pScenario.Platforms{idx};
                    thisClr = clrs(m,:);
                    
                    % Create plotter for this platform's ground truth
                    [str,type] = platformName(thisPlatform);
                    pltr = platformPlotter(tp,type,str,'Marker','d','MarkerFaceColor',thisClr);
                    obj.pPlatformPlotters = [obj.pPlatformPlotters pltr];
                    
                    % Create plotter for detections generated by this
                    % platform
                    [str,type] = detectionsName(thisPlatform);
                    pltr = trackPlotter(tp,type,str,'Marker','o','MarkerSize',6,'MarkerFaceColor',thisClr);
                    obj.pDetectionPlotters = [obj.pDetectionPlotters pltr];
                    
                    pltr = plot3(hax,NaN,NaN,NaN,'-','Color',thisClr,'Marker','none');
                    obj.pDetectionPlottersRay = [obj.pDetectionPlottersRay pltr];
                    
                    % Create plotter for beams generated by this platform
                    [beamPlotterTx, beamPlotterRx] = initBeamPlotter(hax,thisClr);
                    obj.pTxBeamPlotters{m} = beamPlotterTx;
                    obj.pRxBeamPlotters{m} = beamPlotterRx;
                end
                if ~wasHeld
                    hold(hax,'off');
                end
                
                % Add target plotter
                obj.pTargetPlotter = platformPlotter(tp,'DisplayName','Targets','MarkerFaceColor','k');
                
                obj.pPathPlotter = trajectoryPlotter(tp,'LineStyle','-','Tag','All paths');

                % Plot platform paths
                ts = obj.pScenario;
                updateRate = ts.UpdateRate;
                ts.UpdateRate = 1;
                r = record(ts);
                restart(ts);
                ts.UpdateRate = updateRate;
                
                numSamps = numel(r);
                numPlats = numel(r(1).Poses);
                pos = repmat({zeros(numSamps,3)},numPlats,1);
                for m = 1:numPlats
                    thisPos = pos{m};
                    for n = 1:numSamps
                        thisPos(n,:) = r(n).Poses(m).Position;
                    end
                    pos{m} = thisPos/obj.pScale;
                end
                plotTrajectory(obj.pPathPlotter,pos);
                
                wasHeld = ishold(hax);
                hold(hax,'on');
                xlabel(sprintf('X (%s)',obj.DistanceUnits));
                ylabel(sprintf('Y (%s)',obj.DistanceUnits));
                zlabel(sprintf('Z (%s)',obj.DistanceUnits));
                if ~wasHeld
                    hold(hax,'off');
                end
                
                legend(hax,'AutoUpdate','off');
                
                if obj.PlotErrorMetrics
                    if ~wasHeld
                        hold(hax,'on');
                    end
                    obj.pRangeAxes = axes(obj.pFig);
                    obj.pRangeAxes.XGrid = 'on';
                    obj.pRangeAxes.YGrid = 'on';
                    obj.pRangeAxes.Position = [0.65 0.55 0.3 0.35];
                    ylabel(sprintf('Error in Estimated range (%s)', obj.DistanceUnits));
                    xlabel('Simulation time (s)');
                    hold(obj.pRangeAxes,'on');
                    obj.pRangePlotter = errorbar(obj.pRangeAxes,nan,nan,nan,'DisplayName',sprintf('Platform ID %d',iTgts(1)),'Tag',sprintf('Platform ID %d',iTgts(1)),'LineWidth',1.5);
                    for i = 2:numel(iTgts)
                        obj.pRangePlotter(i) = errorbar(obj.pRangeAxes,nan,nan,nan,'DisplayName',sprintf('Platform ID %d',iTgts(i)),'Tag',sprintf('Platform ID %d',iTgts(i)),'LineWidth',1.5);
                    end
                    % Common legend for both axes
                    l = legend(obj.pRangeAxes,'NumColumns',3,'Orientation','horizontal','Location','northoutside');
                    l.Position(2) = 0.91;
                    obj.pRangePlatformIDs = iTgts;
                    obj.pRangeRateAxes = axes(obj.pFig);
                    obj.pRangeRateAxes.XGrid = 'on';
                    obj.pRangeRateAxes.YGrid = 'on';
                    obj.pRangeRateAxes.Position = [0.65 0.1 0.3 0.35];
                    hold(obj.pRangeRateAxes,'on');
                    ylabel(sprintf('Error in Estimated range-rate (m/s)'));
                    xlabel('Simulation time');
                    obj.pRangeRatePlotter = errorbar(obj.pRangeRateAxes,nan,nan,nan,'DisplayName',sprintf('Platform ID %d',iTgts(1)),'Tag',sprintf('Platform ID %d',iTgts(1)),'LineWidth',1.5);
                    for i = 2:numel(iTgts)
                        obj.pRangeRatePlotter(i) = errorbar(obj.pRangeRateAxes,nan,nan,nan,'DisplayName',sprintf('Platform ID %d',iTgts(i)),'Tag',sprintf('Platform ID %d',iTgts(i)),'LineWidth',1.5);
                    end
                    hax.Position = [0.05 0.1 0.5 0.8];
                    l = legend(hax);
                    l.Location = 'best';
                    if ~wasHeld
                        hold(hax,'off');
                    end
                end
            end
        end
        
        function [signals,dets,configs,tracks,tracker] = parseStepInputs(~,varargin)
            
            hasSignals = false;
            hasDets = false;
            hasConfigs = false;
            hasTracks = false;
            hasTracker = false;
            for m = 1:numel(varargin)
                thisInput = varargin{m};
                if iscell(thisInput) && (isempty(thisInput) || isa(thisInput{1},'objectDetection'))
                    hasDets = true;
                    dets = thisInput;
                else
                    if isa(thisInput,'fusion.internal.interfaces.BaseEmission')
                        hasSignals = true;
                        signals = thisInput;
                    elseif isfield(thisInput,'MeasurementParameters')
                        hasConfigs = true;
                        configs = thisInput;
                    elseif (isfield(thisInput,'TrackID') && isfield(thisInput,'State')) || isa(thisInput,'objectTrack')
                        hasTracks = true;
                        if isempty(thisInput) || iscell(thisInput) && isempty(thisInput{1})
                            tracks = struct;
                        else
                            tracks = thisInput;
                        end
                    elseif isa(thisInput,'trackerGNN') || isa(thisInput,'trackerTOMHT')
                        tracker = thisInput;
                        hasTracker = true;
                    end
                end
            end
            
            if ~hasSignals
                signals = [];
            end
            if ~hasDets
                dets = [];
            end
            if ~hasConfigs
                configs = [];
            end
            if ~hasTracks
                tracks = [];
            end
            if ~hasTracker
                tracker = [];
            end
        end
        
        function plotPlatforms(obj)
            % Update postions of platforms and targets
            plats = platformPoses(obj.pScenario);
            pos = reshape([plats.Position],3,[])';
            vel = reshape([plats.Velocity],3,[])';
            
            [iPlats, iTgts] = findPlatforms(obj.pScenario);
            for m = 1:numel(iPlats)
                idx = iPlats(m);
                plotPlatform(obj.pPlatformPlotters(m),pos(idx,:)/obj.pScale,vel(idx,:)/obj.pScale);
            end
            
            plotPlatform(obj.pTargetPlotter,pos(iTgts,:)/obj.pScale,vel(iTgts,:)/obj.pScale);
        end
        
        function plotTXBeams(obj, signals)
            
            % Only plot emitted signals, not reflections
            txSignals = getTXSignals(signals);
            
            beamPlotters = obj.pTxBeamPlotters;
            iPlats = findPlatforms(obj.pScenario);
            iPlatsSigs = findEmittingPlatform(signals);
            for iiPlat = 1:numel(iPlats)
                iPlat = iPlats(iiPlat);
                
                % Find the signals that use this plat
                iSigs = find(iPlatsSigs == iPlat);
                
                beamList = {};
                for iiSigs = 1:numel(iSigs)
                    iSig = iSigs(iiSigs);
                    thisSig = txSignals(iSig);
                    
                    if isempty(beamList)
                        beamList = getBeam(thisSig);
                    else
                        % Append beams from additional sensors mounted on
                        % this platform to the beam list
                        thisBeam = getBeam(thisSig);
                        for iFace = 1:numel(beamList)
                            beamList{iFace} = [beamList{iFace};NaN(1,3);thisBeam{iFace}]; %#ok<AGROW>
                        end
                    end
                end

                % Find the beam plotter for this platform
                platID = obj.pScenario.Platforms{iPlat}.PlatformID;
                idx = iPlats==platID;
                beamPlotter = beamPlotters{idx};
                
                % Plot the beam
                for iFace = 1:numel(beamList)
                    thisPlotter = beamPlotter{iFace};
                    thisFace = beamList{iFace}/obj.pScale;
                    set(thisPlotter,'XData',thisFace(:,1),'YData',thisFace(:,2),'ZData',thisFace(:,3));
                end
            end
        end
        
        function plotRXBeams(obj, configs)
            
            beamPlotters = obj.pRxBeamPlotters;
            iPlats = findPlatforms(obj.pScenario);
            iPlatConfigs = findDetectingPlatform(obj.pScenario, configs);
            for iiPlat = 1:numel(iPlats)
                iPlat = iPlats(iiPlat);
                
                % Find the configs that use this plat
                iConfigs = find(iPlatConfigs == iPlat);
                
                beamList = {};
                for iiConfig = 1:numel(iConfigs)
                    iConfig = iConfigs(iiConfig);
                    thisConfig = configs(iConfig);
                    
                    if isempty(beamList)
                        beamList = getBeam(thisConfig);
                    else
                        % Append beams from additional sensors mounted on
                        % this platform to the beam list
                        thisBeam = getBeam(thisConfig);
                        for iFace = 1:numel(beamList)
                            beamList{iFace} = [beamList{iFace};thisBeam{iFace}]; %#ok<AGROW>
                        end
                    end
                end

                % Find the beam plotter for this platform
                platID = obj.pScenario.Platforms{iPlat}.PlatformID;
                idx = iPlats==platID;
                beamPlotter = beamPlotters{idx};
                
                % Plot the beam
                for iFace = 1:numel(beamList)
                    thisPlotter = beamPlotter{iFace};
                    thisFace = beamList{iFace}/obj.pScale;
                    set(thisPlotter,'XData',thisFace(:,1),'YData',thisFace(:,2),'ZData',thisFace(:,3));
                end
            end
        end
        
        function plotDetections(obj, dets)
            
            isAngleOnly = findAngleOnly(dets);
            
            % Transform detections to scenario frame
            theseDets = dets(~isAngleOnly);
            pos = zeros(3,0);
            cov = zeros(3,3,0);
            for iDet = 1:numel(theseDets)
                thisDet = theseDets{iDet};
                [state, thisCov] = transformDet(obj,thisDet);
                thisPos = state(1:3,:);
                pos = [pos, thisPos]; %#ok<AGROW>
                cov = cat(3,cov,thisCov);
            end
            pos = pos/obj.pScale;
            cov = cov/obj.pScale^2;
            
            iPlats = findPlatforms(obj.pScenario);
            iPlatsDet = findDetectingPlatform(obj.pScenario, theseDets);
            uPlats = unique(iPlatsDet);
            for m = 1:numel(uPlats)
                idx = uPlats(m) == iPlats;
                thisPltr = obj.pDetectionPlotters(idx);
                iFnd = iPlatsDet == uPlats(m);
                plotTrack(thisPltr, pos(:,iFnd)', cov(:,:,iFnd));
            end
            
            % Plot angle only detections
            theseDets = dets(isAngleOnly);
            iPlatsDet = findDetectingPlatform(obj.pScenario, theseDets);
            uPlats = unique(iPlatsDet);
            for m = 1:numel(uPlats)
                idx = uPlats(m) == iPlats;
                thisPltr = obj.pDetectionPlottersRay(idx);

                iDets = find(iPlatsDet == uPlats(m));
                pos = zeros(3,0);
                for iDet = 1:numel(iDets)
                    thisDet = theseDets{iDets(iDet)};
                    state = transformDet(obj,thisDet);
                    thisPos = state(1:3,:);
                    pos = [pos, NaN(3,1), thisPos]; %#ok<AGROW>
                end
                pos = pos/obj.pScale;
                
                set(thisPltr,...
                    'XData',pos(1,:),...
                    'YData',pos(2,:),...
                    'ZData',pos(3,:));
            end
        end
        
        function plotTracks(obj, tracks)
            if ~(obj.HasNonLinearState)
                [trPos, trCov] = getTrackPositions(tracks, obj.TrackPositionSelector);
                trVel = getTrackVelocities(tracks, obj.TrackVelocitySelector);
            else
                [trPos, trCov] = obj.NonLinearTrackPositionFcn(tracks,obj.NonLinearStateInput);
                trVel = zeros(numel(tracks),3);
            end
            tIDs = double([tracks.TrackID]);
            pIDs = cellfun(@(attribs)mode([attribs.TargetIndex]),{tracks.ObjectAttributes});
            pIDs = max(pIDs,-1);
            labels = strrep(strsplit(sprintf('T%02i,P%02i;',[tIDs;pIDs]),';'),'P-1','FA');
            plotTrack(obj.pTrackPlotter, trPos/obj.pScale, trVel/obj.pScale,labels(1:end-1), trCov/obj.pScale^2);
        end
        
        function plotFilters(obj, tracker, tracks)
            if isempty(tracks)
                return;
            end
            filterType = obj.pFilterType;
            if strcmpi(filterType,'helperTrackingMSCPF')
                particles = getTrackFilterProperties(tracker,tracks(1).TrackID,'Particles');
                numParticles = size(particles{1},2);
                positions = zeros(3,numel(tracks)*numParticles);
                for i = 1:numel(tracks)
                    particles = getTrackFilterProperties(tracker,tracks(i).TrackID,'Particles');
                    if (obj.HasNonLinearState)
                        positions(:,(1:numParticles) + (i-1)*numParticles) = obj.NonLinearTrackPositionFcn(particles{1},obj.NonLinearStateInput)';
                    else
                        positions(:,(1:numParticles) + (i-1)*numParticles) = obj.TrackPositionSelector*particles{1};
                    end
                end
                pos = positions/obj.pScale;
                obj.pFilterPlotter.XData = pos(1,:);
                obj.pFilterPlotter.YData = pos(2,:);
                obj.pFilterPlotter.ZData = pos(3,:);
            elseif strcmpi(filterType,'trackingIMM') || strcmpi(filterType,'trackingGSF')
                trackingFilters = getTrackFilterProperties(tracker,tracks(1).TrackID,'TrackingFilters');
                trackingFilters = trackingFilters{1};
                pos = zeros(numel(tracks)*numel(trackingFilters),3);
                cov = zeros(3,3,numel(tracks)*numel(trackingFilters));
                for i = 1:numel(tracks)
                    trackingFilters = getTrackFilterProperties(tracker,tracks(i).TrackID,'TrackingFilters');
                    trackingFilters = trackingFilters{1};
                    for j = 1:numel(trackingFilters)
                        thisID = j + (i-1)*numel(trackingFilters);
                        if obj.HasNonLinearState
                            [pos(thisID,:),cov(:,:,thisID)] = obj.NonLinearTrackPositionFcn(trackingFilters{j},obj.NonLinearStateInput);
                        else
                            pos(thisID,:) = obj.TrackPositionSelector*trackingFilters{j}.State;
                            cov(:,:,thisID) = obj.TrackPositionSelector*trackingFilters{j}.StateCovariance*obj.TrackPositionSelector';
                        end
                    end
                end
                pos = pos/obj.pScale;
                cov = cov/obj.pScale^2;
                obj.pFilterPlotter.plotTrack(pos,cov);
            end
        end

        function [state, cov] = transformDet(obj,det)

            measParam = det.MeasurementParameters(1);
            
            has3DCov = nargout>1 && measParam.HasRange;
            cov = [];
            
            % Compute initial state from measurement
            if strcmpi(measParam.Frame,'Spherical')
                angs = zeros(2,1);
                numAngs = 1+measParam.HasElevation;
                angs(1:numAngs) = det.Measurement(1:numAngs);
                angCov = [300 300];
                angCov(1) = det.Measurement(1);
                if measParam.HasElevation
                    angCov(2) = det.Measurement(2);
                end
                
                if ~measParam.HasRange
                    rg = [0 100*obj.pScale];
                else
                    idx = numAngs+1;
                    rg = det.Measurement(idx);
                    rgCov = det.MeasurementNoise(idx,idx);
                    if measParam.HasVelocity
                        idx = idx+1;
                        rr = det.Measurement(idx);
                        rrCov = det.MeasurementNoise(idx,idx);
                    end
                    
                    if has3DCov
                        azElRgCov = diag([angCov rgCov]);
                    end
                end
                angs = deg2rad(angs);
                [x,y,z] = sph2cart(angs(1),angs(2),rg);
                pos = [x(:) y(:) z(:)]';
                if measParam.HasVelocity
                    vRg = pos(:,end);
                    uRg = vRg/norm(vRg);
                    vel = repmat(rr*uRg,1,size(pos,2));
                    state = [pos;vel];
                    if has3DCov
                        covPosVel = blkdiag(azElRgCov, rrCov);
                        [posCov,velCov] = matlabshared.tracking.internal.fusion.sph2cartcov(covPosVel, angs(1), angs(2), rg);
                        cov = blkdiag(posCov,velCov);
                    end
                else
                    state = pos;
                    if has3DCov
                        cov = matlabshared.tracking.internal.fusion.sph2cartcov(azElRgCov, angs(1), angs(2), rg);
                    end
                end
            else
                state = det.Measurement(:);
                if has3DCov
                    cov = det.MeasurementNoise;
                end
            end
            
            % Rotate into top-level frame
            for m = 1:numel(det.MeasurementParameters)
                measParam = det.MeasurementParameters(m);
                
                T = measParam.OriginPosition(:);
                orient = measParam.Orientation;
                if ~measParam.IsParentToChild
                    orient = orient';
                end
                
                if measParam.HasVelocity
                    T = [T;measParam.OriginPosition(:)]; %#ok<AGROW>
                    orient = blkdiag(orient,orient);
                end
                
                state = bsxfun(@plus,orient'*state,T);
                if has3DCov
                    cov = orient'*cov*orient;
                end
            end
        end
        
        function plotRangeMetrics(obj,~)
            
           currentError = currentTruthMetrics(obj.ErrorMetrics);
           currentCellError = table2cell(currentError);
           numTruths = size(currentCellError,1);
           simTime = obj.pScenario.SimulationTime;
          
           for i = 1:numTruths
               [thisTruth,rError,rSigmaPlusError,rSigmaMinusError,rrError,rrSigmaPlusError,rrSigmaMinusEror] = deal(currentCellError{i,:});
               thisPlotter = find(obj.pRangePlatformIDs == thisTruth);
               obj.pRangePlotter(thisPlotter).YData = [obj.pRangePlotter(thisPlotter).YData rError/obj.pScale];
               obj.pRangePlotter(thisPlotter).XData = [obj.pRangePlotter(thisPlotter).XData simTime];
               
               errorSigmas = abs([rSigmaPlusError rSigmaMinusError] - rError);
               
               if obj.PlotSigmaBounds
                    obj.pRangePlotter(thisPlotter).YPositiveDelta = [obj.pRangePlotter(thisPlotter).YPositiveDelta errorSigmas(1)/obj.pScale];
                	obj.pRangePlotter(thisPlotter).YNegativeDelta = [obj.pRangePlotter(thisPlotter).YNegativeDelta errorSigmas(2)/obj.pScale];
                    obj.pRangeRatePlotter(thisPlotter).YPositiveDelta = [obj.pRangeRatePlotter(thisPlotter).YPositiveDelta abs(rrSigmaPlusError-rrError)];
                    obj.pRangeRatePlotter(thisPlotter).YNegativeDelta = [obj.pRangeRatePlotter(thisPlotter).YNegativeDelta abs(rrSigmaMinusEror-rrError)];
               else
                   obj.pRangePlotter(thisPlotter).YPositiveDelta = zeros(0,1);
                   obj.pRangePlotter(thisPlotter).YNegativeDelta = zeros(0,1);
                   obj.pRangeRatePlotter(thisPlotter).YNegativeDelta = zeros(0,1);
                   obj.pRangeRatePlotter(thisPlotter).YPositiveDelta = zeros(0,1);
               end
               
               obj.pRangeRatePlotter(thisPlotter).YData = [obj.pRangeRatePlotter(thisPlotter).YData rrError];
               obj.pRangeRatePlotter(thisPlotter).XData = [obj.pRangeRatePlotter(thisPlotter).XData simTime];
           end
        end
    end
end

function txSignals = getTXSignals(signals)
% Returns only the direct path signals, which are the transmitted signals
% (not the reflections)
isDP = isDirectPath(signals);
txSignals = signals(isDP);
end

function [beamPlotterTx, beamPlotterRx] = initBeamPlotter(hax,thisClr)

faceAlpha = 0.5;

wasHeld = ishold(hax);
if ~wasHeld
    hold(hax,'on');
end

beamPlotterTx = cell(1,4);
beamPlotterRx = cell(1,4);
for m = 1:numel(beamPlotterTx)
    h = fill3(hax,NaN,NaN,NaN,'b');
    h.FaceColor = thisClr;
    h.FaceAlpha = faceAlpha;
    h.LineStyle = '-';
    h.EdgeColor = thisClr;
    beamPlotterTx{m} = h;

    h = fill3(hax,NaN,NaN,NaN,'b');
    h.FaceColor = thisClr;
    h.FaceAlpha = faceAlpha;
    h.LineStyle = '-';
    h.EdgeColor = thisClr;
    beamPlotterRx{m} = h;
end

if ~wasHeld
    hold(hax,'off');
end
end

function isAngleOnly = findAngleOnly(dets)
numDets = numel(dets);
isAngleOnly = false(numDets,1);
for iDet = 1:numDets
    thisDet = dets{iDet};
    measParam = thisDet.MeasurementParameters(1);
    isAngleOnly(iDet) = ~measParam.HasRange;
end
end

function plotInitBeams(obj)

scene = obj.pScenario;
time = scene.SimulationTime;
cScenario = clone(scene);
plats = cScenario.Platforms;
iPlats = findPlatforms(scene);

% Collect emitted signals
txSigs = {};
for m = 1:numel(iPlats)
    iPlat = iPlats(m);
    thisPlat = plats{iPlat};
    
    sigs = emit(thisPlat, time);
    for k = 1:numel(sigs)
        txSigs = {txSigs{:} sigs(k)}; %#ok<CCAT>
    end
end

% Collect receive configurations
configs = [];
for m = 1:numel(iPlats)
    iPlat = iPlats(m);
    thisPlat = plats{iPlat};
    
    [~,~,config] = detect(thisPlat,txSigs,time);
    configs = [configs;config]; %#ok<AGROW>
end

plotTXBeams(obj, txSigs);
plotRXBeams(obj, configs);
end

function beam = getBeam(thisSig)
maxRange = 10e4*sqrt(2);

fov = thisSig.FieldOfView;
azSpan = fov(1);
elSpan = fov(2);

if isfield(thisSig,'ElectronicAngle')
    eScan = thisSig.ElectronicAngle;
else
    eScan = zeros(2,1);
end
az0 = eScan(1);
el0 = eScan(2);

% Create beam in sensor frame
angs = linspace(-1,1);

% Horizontal faces
[x,y,z] = sph2cart(deg2rad(azSpan/2*angs+az0),deg2rad(elSpan/2+el0)*ones(1,numel(angs)),maxRange*ones(1,numel(angs)));
horz1 = [x(:) y(:) z(:)];
horz1 = [zeros(1,3);horz1;zeros(1,3)];
% horz1 = horz1([1:end 1],:);

[x,y,z] = sph2cart(deg2rad(azSpan/2*angs+az0),deg2rad(-elSpan/2+el0)*ones(1,numel(angs)),maxRange*ones(1,numel(angs)));
horz2 = [x(:) y(:) z(:)];
horz2 = [zeros(1,3);horz2;zeros(1,3)];
% horz2 = horz2([1:end 1],:);

% Vertical faces
[x,y,z] = sph2cart(deg2rad(azSpan/2+az0)*ones(1,numel(angs)),deg2rad(elSpan/2*angs+el0),maxRange*ones(1,numel(angs)));
vert1 = [x(:) y(:) z(:)];
vert1 = [zeros(1,3);vert1;zeros(1,3)];
% vert1 = vert1([1:end 1],:);

[x,y,z] = sph2cart(deg2rad(-azSpan/2+az0)*ones(1,numel(angs)),deg2rad(elSpan/2*angs+el0),maxRange*ones(1,numel(angs)));
vert2 = [x(:) y(:) z(:)];
vert2 = [zeros(1,3);vert2;zeros(1,3)];
% vert2 = vert2([1:end 1],:);

% Beam face list
beam = {horz1 vert1 horz2 vert2};

% Rotate beam to scenario frame
if isfield(thisSig,'MeasurementParameters')
    measParams = thisSig.MeasurementParameters;
else
    measParams = fusion.internal.interfaces.DataStructures.measurementParametersStruct();
    measParams.OriginPosition(:) = thisSig.OriginPosition;
    measParams.Orientation(:) = toRotmat(thisSig.Orientation);
    measParams.HasAzimuth = true;
    measParams.HasElevation = true;
end

numParams = numel(measParams);
for m = 1:numParams
    thisParam = measParams(m);
    orient = thisParam.Orientation;
    if ~thisParam.IsParentToChild
        orient = orient';
    end
    T = thisParam.OriginPosition(:);
    beam = cellfun(@(x)bsxfun(@plus,orient'*x',T)',beam,'UniformOutput',false);
end
end


function [orientOut, wasQuat] = toRotmat(orientIn)
wasQuat = isa(orientIn, 'quaternion');
if wasQuat
    orientOut = rotmat(orientIn, 'frame');
else
    orientOut = orientIn;
end
end

function isDP = isDirectPath(signals)
if isempty(signals)
    isDP = false(1,0);
else
    if iscell(signals)
        isDP = cellfun(@(s)s.PropagationRange,signals)==0;
    else
        isDP = [signals.PropagationRange]==0;
    end
end
end

function [iPlats, iTgts] = findPlatforms(scene)
% Returns the indices of the platforms in scene.Platforms that have sensors
% and or emitters as iPlats. Also returns the indices of the platforms
% which do not have either sensors or emitters as iTgts.
numPlats = numel(scene.Platforms);
isPlat = false(numPlats,1);
for m = 1:numPlats
    thisPlat = scene.Platforms{m};
    isPlat(m) = ~isempty(thisPlat.Sensors) || ~isempty(thisPlat.Emitters);
end
iPlats = find(isPlat);
iTgts = find(~isPlat);
end

function iPlats = findEmittingPlatform(signals)
% Returns the index of each platform from which each signal was
% emitted.
% signals is an N-element cell array of emission objects
% iPlats is an N-by-1 vector of platform indicies into scene.Platform

numSigs = numel(signals);
iPlats = zeros(numSigs,1);

for m = 1:numSigs
    thisSig = localGetVal(signals,m);
    iPlats(m) = thisSig.PlatformID;
end
end

function iPlats = findDetectingPlatform(scene, dets)
% Returns the index of each platform from which each detection was
% generated.
% dets is an N-element cell array of objectDetection objects
% iPlats is an N-by-1 vector of platform indicies into scene.Platform

% Extract sensor indices from the detection of configuration inputs
if isstruct(dets)
    numConfigs = numel(dets);
    sensorIDs = zeros(1,numConfigs);
    for m = 1:numConfigs
        thisConfig = dets(m);
        sensorIDs(m) = thisConfig.SensorIndex;
    end
else
    sensorIDs = cellfun(@(d)d.SensorIndex,dets);
end

numDets = numel(sensorIDs);
iPlats = zeros(numDets,1);
uSensorIDs = unique(sensorIDs);
iSensorPlats = findPlatforms(scene);
for m = 1:numel(uSensorIDs)
    thisID = uSensorIDs(m);
    
    usePlatIdx = -1;
    for k = 1:numel(iSensorPlats)
        idxPlat = iSensorPlats(k);
        thisPlat = scene.Platforms{idxPlat};
        foundSensor = false;
        for j = 1:numel(thisPlat.Sensors)
            thisSensor = thisPlat.Sensors{j};
            if thisSensor.SensorIndex == thisID
                foundSensor = true;
                break
            end
        end
        if foundSensor
            usePlatIdx = idxPlat;
            break
        end
    end
    
    if usePlatIdx>0
        iFnd = sensorIDs == thisID;
        iPlats(iFnd) = usePlatIdx;
    end
end
end

function [str,type] = platformName(plat)
platformID = plat.PlatformID;
str = sprintf('Platform %i',platformID);
type = 'DisplayName';
end

function [str,type] = detectionsName(plat)
detectionID = plat.PlatformID;
str = sprintf('Detections %i',detectionID);
type = 'DisplayName';
end

function val = localGetVal(thing, idx)
% Indexing that supports either cell or non-cell arrays
if iscell(thing)
    val = thing{idx};
else
    val = thing(idx);
end
end

function flag = isPublishing()
% Returns true when MATLAB is publishing
flag = ~isempty(snapnow('get'));
end

function writeAnimatedGIF(fname, frames, numFrames, rate, loopCount)
dt = 1/rate;
if isstruct(frames)
    % pFrames is set to getframe(h) frames
    for m = 1:numFrames
        im = frame2im(frames(m));
        [A,map] = rgb2ind(im,256);
        if m == 1
            imwrite(A,map,fname,'gif','LoopCount',loopCount,'DelayTime',dt);
        else
            imwrite(A,map,fname,'gif','WriteMode','append','DelayTime',dt);
        end
    end
else
    % pFrames is set to RGB images such as are returned
    % by print('-RGBImage','-opengl','-r0')
    for m = 1:numFrames
        im = frames(:,:,:,m);
        [A,map] = rgb2ind(im,256);
        if m == 1
            imwrite(A,map,fname,'gif','LoopCount',loopCount,'DelayTime',dt);
        else
            imwrite(A,map,fname,'gif','WriteMode','append','DelayTime',dt);
        end
    end
end
end

function localWriteVideo(fname, frames, numFrames, rate, profile)
vid = VideoWriter(fname,profile);
vid.FrameRate = rate;
open(vid);
if isstruct(frames)
    % pFrames is set to getframe(h) frames
    for m = 1:numFrames
        writeVideo(vid,frames(m));
    end
else
    % pFrames is set to RGB images such as are returned
    % by print('-RGBImage','-opengl','-r0')
    writeVideo(vid,frames(:,:,:,1:numFrames));
end
close(vid);
end