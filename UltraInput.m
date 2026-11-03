classdef UltraInput < handle
    % Nonblocking host serial/replay adapter. No hardware commands sent.
    properties (Access=private)
        Config
        Port = []
        Buffer = ''
        Lines = strings(0,1)
        Index = 1
        Pending = []
        Session = NaN
        LastScan = -1
        LastRawTime = NaN
        Wrap = 0
        Origin = NaN
        Current = []
        CreatedAt = 0
        ResetNext = true
        LogFID = -1
    end
    properties (SetAccess=private)
        RejectedLines = 0
    end
    methods
        function obj=UltraInput(cfg)
            obj.Config=cfg;
            if strcmp(cfg.inputMode,'serial')
                obj.Port=serialport(cfg.serialPort,cfg.baudRate);
                configureTerminator(obj.Port,'LF');
                folder=fullfile(cfg.root,'results');if ~isfolder(folder),mkdir(folder);end
                tag=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
                obj.LogFID=fopen(fullfile(folder,['serial_raw_' tag '.csv']),'w');
            elseif strcmp(cfg.inputMode,'replay')
                assert(isfile(cfg.replayFile),'Replay CSV file does not exist.');
                obj.Lines=readlines(cfg.replayFile);
                sessions=[];
                for j=1:numel(obj.Lines)
                    m=parseUltraLine(obj.Lines(j),cfg.sensorCount);
                    if ~isempty(m),sessions(end+1)=m.session;end %#ok<AGROW>
                end
                assert(numel(unique(sessions))<=1,'Replay requires one session per CSV. Split sessions first.');
            else
                error('Input mode must be serial or replay.');
            end
        end
        function scans=poll(obj,t)
            scans={};
            if ~isempty(obj.Port)&&obj.Port.NumBytesAvailable>0
                bytes=read(obj.Port,obj.Port.NumBytesAvailable,'uint8');
                obj.Buffer=[obj.Buffer char(bytes(:)')];
                % Keep only complete lines. A partial record never blocks the model.
                last=find(obj.Buffer==newline,1,'last');
                if ~isempty(last)
                    obj.Lines=[obj.Lines;splitlines(string(obj.Buffer(1:last)))];
                    obj.Buffer=obj.Buffer(last+1:end);
                end
                if numel(obj.Buffer)>10000,obj.Buffer='';obj.RejectedLines=obj.RejectedLines+1;end
            end
            while true
                if isempty(obj.Pending)
                    if obj.Index>numel(obj.Lines),break;end
                    line=obj.Lines(obj.Index);obj.Index=obj.Index+1;
                    if strlength(strtrim(line))==0,continue;end
                    if obj.LogFID>=0,fprintf(obj.LogFID,'%s\n',line);end
                    m=parseUltraLine(line,obj.Config.sensorCount);
                    if isempty(m),obj.RejectedLines=obj.RejectedLines+1;continue;end
                    if m.session~=obj.Session
                        % Session boundaries reset estimator and discard partial scans.
                        obj.Session=m.session;obj.LastScan=-1;obj.Current=[];
                        obj.LastRawTime=NaN;obj.Wrap=0;obj.Origin=NaN;obj.ResetNext=true;
                    end
                    if isfinite(obj.LastRawTime)&&m.trigger<obj.LastRawTime
                        if obj.LastRawTime-m.trigger>2^31,obj.Wrap=obj.Wrap+2^32;
                        else,obj.RejectedLines=obj.RejectedLines+1;continue;end
                    end
                    obj.LastRawTime=m.trigger;
                    if ~isfinite(obj.Origin),obj.Origin=(m.trigger+obj.Wrap)/1e6;end
                    m.time=(m.trigger+obj.Wrap)/1e6-obj.Origin;
                    obj.Pending=m;
                end
                m=obj.Pending;
                if strcmp(obj.Config.inputMode,'replay')&&m.time>t,break;end
                obj.Pending=[];
                if m.scan<=obj.LastScan,obj.RejectedLines=obj.RejectedLines+1;continue;end
                if ~isempty(obj.Current)&&m.scan~=obj.Current.scan
                    if m.scan<obj.Current.scan,obj.RejectedLines=obj.RejectedLines+1;continue;end
                    scans{end+1}=obj.closeScan(); %#ok<AGROW>
                end
                if isempty(obj.Current)
                    obj.Current=struct('scan',m.scan,'session',m.session,'raw',nan(obj.Config.sensorCount,1),...
                        'times',nan(obj.Config.sensorCount,1),'hw',false(obj.Config.sensorCount,1),'seen',false(obj.Config.sensorCount,1),'reset',obj.ResetNext);
                    obj.ResetNext=false;obj.CreatedAt=t;
                end
                i=m.sensor;
                if obj.Current.seen(i),obj.RejectedLines=obj.RejectedLines+1;continue;end
                obj.Current.raw(i)=m.range;obj.Current.times(i)=m.time;
                obj.Current.hw(i)=m.ok;obj.Current.seen(i)=true;
                if all(obj.Current.seen),scans{end+1}=obj.closeScan();end %#ok<AGROW>
            end
            if ~isempty(obj.Current)&&t-obj.CreatedAt>=obj.Config.scanTimeout
                scans{end+1}=obj.closeScan();
            end
            % Discard consumed text to avoid unbounded serial buffer growth.
            if ~isempty(obj.Port)&&obj.Index>1000
                obj.Lines=obj.Lines(obj.Index:end);obj.Index=1;
            end
        end
        function delete(obj)
            if obj.LogFID>=0,fclose(obj.LogFID);obj.LogFID=-1;end
            if ~isempty(obj.Port),delete(obj.Port);obj.Port=[];end
        end
    end
    methods (Access=private)
        function sc=closeScan(obj)
            sc=obj.Current;obj.LastScan=sc.scan;obj.Current=[];
        end
    end
end
