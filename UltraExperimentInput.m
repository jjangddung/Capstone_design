classdef UltraExperimentInput < handle
    % Adapter for the supplied command-driven, four-sensor Arduino sketch.
    properties (SetAccess=private)
        Ready=false
        Status='Connecting; waiting for Arduino READY'
        RejectedLines=0
        Config
    end
    properties (Access=private)
        Port=[]
        Buffer=''
        Lines=strings(0,1)
        Index=1
        Pending=[]
        LastRun=NaN
        LastPoint=""
        LastCycle=0
        LastRaw=NaN
        Wrap=0
        Origin=NaN
        Offset=0
        LastTime=NaN
        LastStart=NaN
        ResetNext=true
        LogFID=-1
        Phase='handshake'
        Deadline=inf
        LastHello=-inf
        Target=[NaN NaN]
        FrameOrigin=[0 0]
        RunToken=0
        StartAnnounced=false
        Expected=30
        CurrentGT=[NaN NaN]
        CurrentRun=NaN
        RunEvents={}
    end
    methods
        function obj=UltraExperimentInput(cfg,transport)
            obj.Config=cfg;
            if strcmp(cfg.inputMode,'serial')
                if nargin>1,obj.Port=transport;
                else
                    obj.Port=serialport(cfg.serialPort,cfg.baudRate,'Timeout',1);
                    configureTerminator(obj.Port,'LF');
                end
                folder=fullfile(cfg.root,'results');if ~isfolder(folder),mkdir(folder);end
                name=['serial_experiment_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.csv'];
                obj.LogFID=fopen(fullfile(folder,name),'w');
                [obj.FrameOrigin,~]=ultraSensorFrame(cfg);
                if obj.LogFID>=0,fprintf(obj.LogFID,'# COORDINATE_ORIGIN_M,%.9f,%.9f\n',obj.FrameOrigin);end
                obj.Deadline=15;
            elseif strcmp(cfg.inputMode,'replay')
                obj.Lines=readlines(cfg.replayFile);obj.Phase='replay';obj.Status='Replay: Arduino experiment CSV';
            else,error('Expected serial or replay input.');end
        end
        function startRun(obj,gt,t)
            assert(~isempty(obj.Port)&&obj.Ready,'Ultra:NotReady','Arduino is not ready. Wait for READY.');
            assert(numel(gt)==2&&all(isfinite(gt)),'Enter the measured, stationary target position.');
            b=obj.Config.roi;
            assert(gt(1)>=b(1)&&gt(1)<=b(2)&&gt(2)>=b(3)&&gt(2)<=b(4),'GT is outside the configured ROI.');
            [origin,span]=ultraSensorFrame(obj.Config);
            obj.FrameOrigin=origin;obj.Target=gt(:)'-origin;obj.Ready=false;obj.Phase='geometry';obj.Deadline=t+5;
            obj.Status='Setting geometry; waiting for acknowledgment';
            obj.send(sprintf('G %.6f %.6f',100*span(1),100*span(2)));
        end
        function scans=poll(obj,t)
            scans={};
            if ~isempty(obj.Port)&&obj.Port.NumBytesAvailable>0
                bytes=read(obj.Port,obj.Port.NumBytesAvailable,'uint8');
                obj.Buffer=[obj.Buffer char(bytes(:)')];
                last=find(obj.Buffer==newline,1,'last');
                if ~isempty(last)
                    obj.Lines=[obj.Lines;splitlines(string(obj.Buffer(1:last)))];
                    obj.Buffer=obj.Buffer(last+1:end);
                end
                if numel(obj.Buffer)>16000,obj.Buffer='';obj.RejectedLines=obj.RejectedLines+1;end
            end
            while true
                if isempty(obj.Pending)
                    if obj.Index>numel(obj.Lines),break;end
                    line=strtrim(obj.Lines(obj.Index));obj.Index=obj.Index+1;
                    if strlength(line)==0,continue;end
                    if obj.LogFID>=0,fprintf(obj.LogFID,'%s\n',line);end
                    if startsWith(line,'#'),obj.handleMessage(line,t);continue;end
                    if startsWith(line,'point_id,'),continue;end
                    r=parseUltraExperimentLine(line);
                    if isempty(r),obj.RejectedLines=obj.RejectedLines+1;continue;end
                    r.reportedGT=r.gt;r.reportedArduinoPosition=r.arduinoPosition;
                    r.gt=r.gt+obj.FrameOrigin;
                    r.arduinoPosition=r.arduinoPosition+obj.FrameOrigin;
                    fresh=obj.ResetNext||r.run~=obj.LastRun||r.point~=obj.LastPoint;
                    if ~fresh&&r.scan<=obj.LastCycle,obj.RejectedLines=obj.RejectedLines+1;continue;end
                    if isfinite(obj.LastRun)&&r.run<obj.LastRun,obj.resetClock();fresh=true;end
                    % Unwrap all four sequential timestamps, including rollover within a row.
                    raw=obj.LastRaw;wrap=obj.Wrap;absolute=zeros(4,1);valid=true;
                    for i=1:4
                        if isfinite(raw)&&r.triggerUs(i)<raw
                            if raw-r.triggerUs(i)>2^31,wrap=wrap+2^32;else,valid=false;break;end
                        end
                        raw=r.triggerUs(i);absolute(i)=(raw+wrap)/1e6;
                    end
                    if ~valid,obj.RejectedLines=obj.RejectedLines+1;continue;end
                    if ~isfinite(obj.Origin),obj.Origin=absolute(1);end
                    times=absolute-obj.Origin+obj.Offset;
                    period=NaN;
                    if ~fresh&&isfinite(obj.LastStart),period=times(1)-obj.LastStart;end
                    obj.LastStart=times(1);obj.LastTime=times(end);obj.LastRaw=raw;obj.Wrap=wrap;
                    obj.LastRun=r.run;obj.LastPoint=r.point;obj.LastCycle=r.scan;obj.ResetNext=false;
                    if fresh
                        if ~obj.StartAnnounced
                            obj.RunToken=obj.RunToken+1;obj.Expected=30;
                            obj.CurrentRun=r.run;obj.CurrentGT=r.gt;obj.queueEvent('start');
                        end
                        obj.StartAnnounced=false;
                    end
                    obj.CurrentRun=r.run;obj.CurrentGT=r.gt;r.exportToken=obj.RunToken;
                    r.times=times;r.reset=fresh;r.period=period;
                    obj.Pending=r;
                end
                r=obj.Pending;
                if strcmp(obj.Config.inputMode,'replay')&&r.times(end)>t,break;end
                obj.Pending=[];scans{end+1}=r; %#ok<AGROW>
                if ~isempty(obj.Port)
                    obj.Status=sprintf('Receiving run %d: %d / 30 | rejected lines %d',r.run,r.scan,obj.RejectedLines);
                    obj.Deadline=t+10;
                end
            end
            if ~isempty(obj.Port)
                if t>obj.Deadline
                    obj.Ready=false;obj.Phase='error';obj.Deadline=inf;
                    obj.Status='Arduino response timeout. Disconnect and check port/sketch.';
                elseif strcmp(obj.Phase,'handshake')&&t-obj.LastHello>=1
                    obj.send('HELP');obj.LastHello=t;
                end
                if obj.Index>1000,obj.Lines=obj.Lines(obj.Index:end);obj.Index=1;end
            end
        end
        function events=takeRunEvents(obj)
            events=obj.RunEvents;obj.RunEvents={};
        end
        function delete(obj)
            if obj.LogFID>=0,fclose(obj.LogFID);obj.LogFID=-1;end
            if ~isempty(obj.Port),delete(obj.Port);obj.Port=[];end
        end
    end
    methods (Access=private)
        function queueEvent(obj,kind)
            obj.RunEvents{end+1}=struct('kind',kind,'id',obj.RunToken,'expected',obj.Expected,...
                'gt',obj.CurrentGT,'run',obj.CurrentRun);
        end
        function send(obj,line)
            writeline(obj.Port,line);
            if obj.LogFID>=0,fprintf(obj.LogFID,'# TX %s\n',line);end
        end
        function resetClock(obj)
            obj.LastRaw=NaN;obj.Wrap=0;obj.Origin=NaN;
            if isfinite(obj.LastTime),obj.Offset=obj.LastTime+.001;end
            obj.LastRun=NaN;obj.LastCycle=0;obj.ResetNext=true;obj.LastStart=NaN;
        end
        function handleMessage(obj,line,t)
            if startsWith(line,'# COORDINATE_ORIGIN_M,')
                a=split(line,',');
                if numel(a)==3
                    v=str2double(a(2:3))';
                    if all(isfinite(v)),obj.FrameOrigin=v;end
                end
            elseif startsWith(line,'# Four Sensor')
                obj.resetClock();obj.Ready=false;obj.Phase='handshake';obj.Deadline=t+15;
            elseif startsWith(line,'# Geometry')&&strcmp(obj.Phase,'geometry')
                a=regexp(char(line),'Lx=([\d.]+)\s+Ly=([\d.]+)','tokens','once');
                [~,span]=ultraSensorFrame(obj.Config);expected=100*span;
                if isempty(a)||any(abs(str2double(a)-expected)>.02)
                    obj.Status='Geometry acknowledgment mismatch. Reconnect.';obj.Phase='error';return;
                end
                obj.send(sprintf('P1 %.6f %.6f',100*obj.Target(1),100*obj.Target(2)));
                obj.Phase='measuring';obj.Deadline=t+10;obj.Status='Measuring 30 stationary samples';
            elseif startsWith(line,'# START')
                a=regexp(char(line),'run=(\d+),point=[^,]+,cycles=(\d+)','tokens','once');
                if isempty(a),return;end
                obj.RunToken=obj.RunToken+1;obj.StartAnnounced=true;
                obj.CurrentRun=str2double(a{1});obj.Expected=str2double(a{2});
                obj.CurrentGT=[NaN NaN];
                if ~isempty(obj.Port)&&all(isfinite(obj.Target)),obj.CurrentGT=obj.Target+obj.FrameOrigin;end
                obj.queueEvent('start');
                obj.ResetNext=true;obj.LastStart=NaN;
                if ~isempty(obj.Port),obj.Ready=false;obj.Phase='measuring';obj.Deadline=t+10;end
            elseif line=="# READY"
                if ~isempty(obj.Port)&&~ismember(obj.Phase,{'geometry','measuring'})
                    obj.Ready=true;obj.Phase='ready';obj.Deadline=inf;obj.Status='Ready: enter measured GT [m], then Measure 30';
                end
            elseif startsWith(line,'# END')
                a=regexp(char(line),'run=(\d+)','tokens','once');
                if ~isempty(a)&&str2double(a{1})==obj.CurrentRun,obj.queueEvent('end');end
                if ~isempty(obj.Port),obj.Phase='finishing';obj.Deadline=t+5;end
            elseif startsWith(line,'# ERROR')
                obj.Status=char(line);obj.Ready=false;obj.Phase='error';obj.Deadline=inf;
            end
        end
    end
end
