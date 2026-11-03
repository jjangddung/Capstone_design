classdef UltraLiveSession < handle
    % Real-time serial acquisition for the existing ultraView window.
    properties (SetAccess=private)
        State
        Config
        Input=[]
        Active=false
        SavedRuns={}
    end
    properties (Access=private)
        Figure
        Timer=[]
        Clock
        Saved=false
        RunMeta={}
        Exported=[]
        SessionTag=''
    end
    methods
        function obj=UltraLiveSession(cfg,f,transport)
            obj.SessionTag=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
            obj.Config=cfg;obj.Figure=f;obj.State=ultraInitialize(cfg);
            obj.State.p=[NaN NaN];obj.State.coverage(:)=0;
            if nargin>2,obj.Input=UltraExperimentInput(cfg,transport);
            else,obj.Input=UltraExperimentInput(cfg);end
            obj.Clock=tic;obj.Active=true;
            obj.Timer=timer('ExecutionMode','fixedSpacing','Period',.05,'BusyMode','drop',...
                'TimerFcn',@(~,~)obj.tick());
            start(obj.Timer);
        end
        function measure(obj,gt)
            assert(obj.Active,'Disconnected. Connect Arduino first.');
            obj.Input.startRun(gt,toc(obj.Clock));
            if isgraphics(obj.Figure)
                h=obj.Figure.UserData;set([h.x h.y h.sx h.sy],'Enable','off');
                if isfield(h,'saved'),h.saved.String=sprintf('Measuring at GT (%.3f, %.3f) m. GT is locked for this run.',gt);end
            end
        end
        function stop(obj)
            obj.Active=false;
            if ~isempty(obj.Timer)&&isvalid(obj.Timer)
                stop(obj.Timer);delete(obj.Timer);obj.Timer=[];
            end
            if ~isempty(obj.Input),delete(obj.Input);obj.Input=[];end
            obj.saveResults();
        end
        function delete(obj),obj.stop();end
    end
    methods (Access=private)
        function handleRunEvents(obj,events)
            for j=1:numel(events)
                e=events{j};
                if strcmp(e.kind,'start')
                    % A new run also closes any older run lacking an END marker.
                    for k=1:numel(obj.RunMeta)
                        if obj.RunMeta{k}.id~=e.id,obj.exportRun(obj.RunMeta{k}.id);end
                    end
                end
                meta=struct('id',e.id,'expected',e.expected,'gt',e.gt,...
                    'ended',strcmp(e.kind,'end'),'tag',obj.SessionTag);
                ids=cellfun(@(m)m.id,obj.RunMeta);ix=find(ids==e.id,1);
                if isempty(ix),obj.RunMeta{end+1}=meta;else,obj.RunMeta{ix}=meta;end
                if meta.ended,obj.exportRun(e.id);end
            end
        end
        function exportPendingRuns(obj)
            if ~isempty(obj.Input),obj.handleRunEvents(obj.Input.takeRunEvents());end
            for j=1:numel(obj.RunMeta),obj.exportRun(obj.RunMeta{j}.id);end
        end
        function exportRun(obj,id)
            if ismember(id,obj.Exported),return;end
            ix=find(cellfun(@(m)m.id,obj.RunMeta)==id,1);if isempty(ix),return;end
            meta=obj.RunMeta{ix};records=obj.State.externalRecords;
            keep=cellfun(@(r)isfield(r,'exportToken')&&r.exportToken==id,records);
            history=obj.State.history(keep);records=records(keep);
            if ~isempty(history)
                gt=vertcat(history.gt);
                if all(isfinite(gt),'all')&&all(abs(gt-gt(1,:))<1e-8,'all'),meta.gt=gt(1,:);
                else,meta.gt=[NaN NaN];end
            end
            saved=ultraSaveRunCsv(history,records,obj.Config,meta);
            obj.SavedRuns{end+1}=saved;obj.Exported(end+1)=id;
            fprintf('Saved RAW + positions: %s\nSaved statistics: %s\n',saved.samples,saved.stats);
            if isgraphics(obj.Figure)
                h=obj.Figure.UserData;
                if isfield(h,'saved')
                    [~,name,ext]=fileparts(saved.samples);s=saved.summary;
                    h.saved.String=sprintf(['Saved: %s%s\nReceived %d / %d | LS successes %d\n' ...
                        'LS mean X/Y: %.3f / %.3f cm | std X/Y: %.3f / %.3f cm\n' ...
                        'LS error mean/std: %.3f / %.3f cm'],name,ext,saved.count,meta.expected,...
                        s.Success_N(1),s.Mean_X_cm(1),s.Mean_Y_cm(1),s.Std_X_cm(1),s.Std_Y_cm(1),...
                        s.Mean_Error_cm(1),s.Std_Error_cm(1));
                    h.saved.TooltipString=sprintf('%s\n%s',saved.samples,saved.stats);
                end
            end
        end
        function tick(obj)
            if ~obj.Active,return;end
            if ~isgraphics(obj.Figure),obj.stop();return;end
            try
                t=toc(obj.Clock);scans=obj.Input.poll(t);
                [obj.State,~]=ultraExternalTick(obj.State,t,scans,obj.Config);
                obj.handleRunEvents(obj.Input.takeRunEvents());
                status=obj.Input.Status;ready=obj.Input.Ready;
                ultraView('update',obj.State,t,[0 0 0 0 2 1],obj.Config);
                if ~obj.Active||~isgraphics(obj.Figure),return;end
                h=obj.Figure.UserData;h.connection.String=status;
                if ready
                    h.run.Enable='on';set([h.x h.y h.sx h.sy],'Enable','on');
                else,h.run.Enable='off';end
            catch err
                obj.stop();
                if isgraphics(obj.Figure)
                    h=obj.Figure.UserData;h.connection.String=['Disconnected: ' err.message];h.run.Enable='off';
                end
                warning('Ultra:LiveInput','%s',err.message);
            end
        end
        function saveResults(obj)
            if obj.Saved,return;end
            obj.exportPendingRuns();
            history=obj.State.history;cfg=obj.Config;externalRecords=obj.State.externalRecords; %#ok<NASGU>
            folder=fullfile(cfg.root,'results');if ~isfolder(folder),mkdir(folder);end
            stem=fullfile(folder,['live_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);
            save([stem '.mat'],'history','cfg','externalRecords');
            ultraSummary(history,[stem '_summary.csv']);
            ids=unique([history.experiment]);
            for id=ids,ultraSummary(history([history.experiment]==id),sprintf('%s_run%d_summary.csv',stem,id));end
            assignin('base','ultraHistory',history);obj.Saved=true;
        end
    end
end
