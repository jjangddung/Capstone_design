function ultraView(action,st,t,command,cfg)
tag='UltraLocalizationViewer'; f=findall(groot,'Type','figure','Tag',tag);
if strcmp(action,'open')
    if ~isempty(f),ultraCloseView(f);end
    f=figure('Name','Ultrasonic localization | GT comparison','Tag',tag,...
        'NumberTitle','off','CloseRequestFcn',@(src,~)ultraCloseView(src),'Position',[50 60 1240 760],'Color',[.96 .97 .99]);
    ax=axes(f,'Units','pixels','Position',[60 180 540 520]);
    axis(ax,[0 1 0 1]);axis(ax,'equal');grid(ax,'on');hold(ax,'on');
    xlabel(ax,'x [m]');ylabel(ax,'y [m]');title(ax,'1 m x 1 m | all sensors face center');
    plot(ax,[0 1 1 0 0],[0 0 1 1 0],'k-','LineWidth',2);
    rectangle(ax,'Position',[cfg.roi(1),cfg.roi(3),diff(cfg.roi(1:2)),diff(cfg.roi(3:4))],...
        'EdgeColor',[.5 .5 .5],'LineStyle',':');
    scatter(ax,cfg.sensorXY(:,1),cfg.sensorXY(:,2),45,'filled','MarkerFaceColor',[.2 .3 .6]);
    quiver(ax,cfg.sensorXY(:,1),cfg.sensorXY(:,2),cfg.direction(:,1)*.12,cfg.direction(:,2)*.12,0,'Color',[.3 .4 .6]);
    for i=1:cfg.sensorCount
        text(ax,min(cfg.sensorXY(i,1)+.01,.94),min(cfg.sensorXY(i,2)+.02,.96),sprintf('S%d',i));
        u=cfg.direction(i,:);v=[-u(2) u(1)];s=cfg.sensorXY(i,:);
        axialPlot=[0; sqrt(2)];
        for signSide=[-1 1]
            pts=s+axialPlot*u+signSide*axialPlot*tand(cfg.beamFullAngleDeg/2)*v;
            plot(ax,pts(:,1),pts(:,2),':','Color',[.65 .65 .65]);
        end
    end
    h.gt=plot(ax,NaN,NaN,'ko-','LineWidth',1.5,'DisplayName','GT at estimate time');
    colors=[.1 .4 .9;.95 .5 .05;.1 .65 .3]; marks={'s','^','d'};
    for k=1:3,h.est(k)=plot(ax,NaN,NaN,['-' marks{k}],'Color',colors(k,:),'LineWidth',1.2);end
    h.body=rectangle(ax,'Position',[.5-cfg.bodySize/2 cfg.bodySize],'EdgeColor',[.6 .6 .6]);
    h.target=rectangle(ax,'Position',[.5-cfg.radius .5-cfg.radius 2*cfg.radius 2*cfg.radius],...
        'Curvature',[1 1],'EdgeColor','k');
    h.now=plot(ax,NaN,NaN,'k+','MarkerSize',10);
    h.goal=plot(ax,NaN,NaN,'ko','MarkerSize',8,'HandleVisibility','off');
    lg=legend(ax,[h.gt h.est h.now],{'GT(t_ref)','LS','LMedS','Motion LMedS','GT now'},'Location','southoutside','Orientation','horizontal');
    lg.AutoUpdate='off';lg.FontSize=8;
    errorAx=axes(f,'Units','pixels','Position',[65 55 530 75]);hold(errorAx,'on');grid(errorAx,'on');
    xlabel(errorAx,'Reference time [s]');ylabel(errorAx,'Error [cm]');
    for k=1:3,h.err(k)=plot(errorAx,NaN,NaN,'Color',colors(k,:),'LineWidth',1.2);end
    label(f,'X / GT [m]',650,705);h.x=uicontrol(f,'Style','edit','String','.50','Position',[750 700 90 28]);
    label(f,'Y / GT [m]',870,705);h.y=uicontrol(f,'Style','edit','String','.50','Position',[970 700 90 28]);
    h.sx=uicontrol(f,'Style','slider','Min',cfg.roi(1),'Max',cfg.roi(2),'Value',.5,'Position',[650 665 190 20],...
        'Callback',@(src,~)set(h.x,'String',sprintf('%.3f',src.Value)));
    h.sy=uicontrol(f,'Style','slider','Min',cfg.roi(3),'Max',cfg.roi(4),'Value',.5,'Position',[870 665 190 20],...
        'Callback',@(src,~)set(h.y,'String',sprintf('%.3f',src.Value)));
    h.connect=uicontrol(f,'Style','pushbutton','String','Connect Arduino (USB)',...
        'Position',[650 735 210 24],'Callback',@(~,~)ultraConnectArduino(cfg));
    h.connection=uicontrol(f,'Style','text','String','Simulation: no Arduino connected',...
        'Position',[865 735 350 24],'HorizontalAlignment','left','BackgroundColor',[.96 .97 .99]);
    h.mode=uicontrol(f,'Style','popupmenu','String',{'Static placement (reset)','Move slowly to goal'},'Position',[650 620 250 30]);
    h.beam=uicontrol(f,'Style','popupmenu','String',{'IDEAL: no angular limit',sprintf('FIXED %.0f deg total (+/-%.0f): ASSUMPTION',cfg.beamFullAngleDeg,cfg.beamFullAngleDeg/2),'LEGACY measured profile: 20-40 cm'},'Value',cfg.defaultCoverageMode,'Position',[650 575 450 30]);
    h.noise=uicontrol(f,'Style','popupmenu','String',{'Ideal distances','Noise + 1 cm quantization','Outlier S2 + wall S4'},'Position',[650 530 450 30]);
    h.apply=uicontrol(f,'Style','pushbutton','String','Apply X/Y + settings','Position',[650 480 220 35],'Callback',@(~,~)apply(f,cfg));
    h.run=uicontrol(f,'Style','pushbutton','String','Run / Restart','Position',[880 480 135 35],'Callback',@(~,~)runModel(f,cfg));
    h.stop=uicontrol(f,'Style','pushbutton','String','Stop','Position',[1025 480 95 35],'Callback',@(~,~)stopView(f));
    h.status=uicontrol(f,'Style','text','String','Example body/radius/ROI. Edit ultraConfig.m for measured dimensions.',...
        'HorizontalAlignment','left','Position',[650 365 550 105],'BackgroundColor',[.96 .97 .99]);
    h.table=uitable(f,'Units','pixels','Position',[650 160 550 195],...
        'ColumnName',{'Method','X [m]','Y [m]','Error [cm]','Status'},...
        'ColumnWidth',{100 85 85 85 180},'Data',cell(4,5));
    h.sensor=uicontrol(f,'Style','text','Position',[650 40 550 110],'HorizontalAlignment','left',...
        'BackgroundColor',[.96 .97 .99],'String','Sensor measurements will appear here.');
    label(f,sprintf('Dotted lines: fixed +/-%.0f deg reference; see selected coverage mode.',cfg.beamFullAngleDeg/2),40,5);
    if ~strcmp(cfg.inputMode,'simulation')
        set([h.x h.y h.sx h.sy h.mode h.beam h.noise],'Enable','off');
    end
    if strcmp(cfg.inputMode,'serial')
        set([h.x h.y h.sx h.sy],'Enable','on');
        set([h.mode h.beam h.noise],'Visible','off');
        h.saved=uicontrol(f,'Style','text','Position',[650 530 550 100],...
            'HorizontalAlignment','left','BackgroundColor',[.96 .97 .99],...
            'String','Enter wall-based GT in X/Y above. Each completed run saves samples + statistics CSV.');
        h.apply.String='GT is for error comparison only';h.apply.Enable='off';
        h.run.String='Measure 30';h.run.Enable='off';h.stop.String='Disconnect';
        h.connection.String=['Connecting ' cfg.serialPort ' ...'];
        h.status.String='Place the target, enter measured X/Y [m], then Measure 30 when READY. No simulated ranges.';
        set([h.body h.target],'Visible','off');
    end
    h.lastDraw=-inf;h.counter=1;f.UserData=h;
    return
end
if isempty(f),return;end
h=f.UserData;if t-h.lastDraw<.1 && t>=h.lastDraw,return;end
h.lastDraw=t;
if ~isempty(st.history)
    ix=find([st.history.experiment]==st.experiment); ix=ix(max(1,end-199):end);
    gt=vertcat(st.history(ix).gt);set(h.gt,'XData',gt(:,1),'YData',gt(:,2));
    for k=1:3
        p=nan(numel(ix),2);
        for j=1:numel(ix),p(j,:)=st.history(ix(j)).result.p(k,:);end
        set(h.est(k),'XData',p(:,1),'YData',p(:,2));
        set(h.err(k),'XData',[st.history(ix).time],'YData',100*vecnorm(p-gt,2,2));
    end
end
if all(isfinite(st.p))
    set([h.body h.target],'Visible','on');
    set(h.body,'Position',[st.p-cfg.bodySize/2 cfg.bodySize]);
    set(h.target,'Position',[st.p-cfg.radius 2*cfg.radius 2*cfg.radius]);
else
    set([h.body h.target],'Visible','off');
end
set(h.now,'XData',st.p(1),'YData',st.p(2));
if strcmp(cfg.inputMode,'simulation')&&command(3)==1
    set(h.goal,'XData',command(1),'YData',command(2));
else
    set(h.goal,'XData',NaN,'YData',NaN);
end
names={'GT(t_ref)','LS','LMedS','Motion LMedS'};data=cell(4,5);
data(1,:)={names{1},st.gtRef(1),st.gtRef(2),NaN,'Reference'};
for k=1:3,data(k+1,:)={names{k+1},st.last.p(k,1),st.last.p(k,2),100*st.error(k),char(st.last.status(k))};end
h.table.Data=data;
if strcmp(cfg.inputMode,'simulation')
    if command(5)==1
        mode='IDEAL: angular limit disabled (dotted lines are reference only)';
    elseif command(5)==2
        mode=sprintf('FIXED %.0f deg total (+/-%.0f): MODEL ASSUMPTION',cfg.beamFullAngleDeg,cfg.beamFullAngleDeg/2);
    else
        mode='LEGACY measured 20-40 cm; dotted fixed-angle lines are reference only';
    end
else
    mode=['INPUT: ' upper(cfg.inputMode) ' | GT from recorded experiment'];
end
h.status.String=sprintf('%s\nt=%.2f s | t_ref=%.2f | experiment=%d\nGT now=(%.3f, %.3f) | new scan count=%d\nValid sensors=%d / %d | r=%.4f m (check measured value)',...
    mode,t,st.last.tRef,st.experiment,st.p(1),st.p(2),numel(st.history),sum(st.last.valid),cfg.sensorCount,cfg.radius);
h.sensor.String=sprintf('Corrected distances [m]: %s\nValid mask: %s\nCoverage codes: %s\n1=assumed/inside  2=outside  3=UNCALIBRATED\nBackground: %s',...
    mat2str(st.last.range',3),mat2str(st.last.valid'),mat2str(st.coverage'),cfg.backgroundSource);
if ~strcmp(cfg.inputMode,'simulation')
    period=NaN;
    if ~isempty(st.externalRecords)&&isfield(st.externalRecords{end},'period'),period=st.externalRecords{end}.period;end
    h.sensor.String=sprintf('Received distances [m]: %s\nValid mask: %s\nMeasured scan period: %.1f ms | sensor order: Left / Right / Bottom / Top\nBackground: %s',...
        mat2str(st.raw',4),mat2str(st.last.valid'),1000*period,cfg.backgroundSource);
end
f.UserData=h;drawnow limitrate;
end
function label(f,txt,x,y)
uicontrol(f,'Style','text','String',txt,'Position',[x y 550 22],'HorizontalAlignment','left','BackgroundColor',[.96 .97 .99]);
end
function apply(f,cfg)
if ~strcmp(cfg.inputMode,'simulation'),return;end
h=f.UserData;x=str2double(h.x.String);y=str2double(h.y.String);b=cfg.roi;
if ~isfinite(x)||~isfinite(y)||x<b(1)||x>b(2)||y<b(3)||y>b(4)
    h.status.String='Rejected: coordinates outside ROI. Previous position retained.';return;
end
h.counter=h.counter+1;f.UserData=h;
cmd=[x y h.mode.Value-1 h.counter h.beam.Value h.noise.Value];
set_param('ultrasonic_localization/UserCommand','Value',mat2str(cmd,12));
end
function runModel(f,cfg)
if strcmp(cfg.inputMode,'serial')
    h=f.UserData;
    try
        assert(isappdata(f,'UltraLiveSession'),'Connect Arduino first.');
        s=getappdata(f,'UltraLiveSession');s.measure([str2double(h.x.String),str2double(h.y.String)]);
        h.run.Enable='off';
    catch err,h.connection.String=err.message;end
    return
end
model='ultrasonic_localization';
if ~strcmp(get_param(model,'SimulationStatus'),'stopped'),return;end
apply(f,cfg);set_param(model,'SimulationCommand','start');
end
function stopView(f)
if isappdata(f,'UltraLiveSession')
    s=getappdata(f,'UltraLiveSession');s.stop();
    h=f.UserData;h.connection.String='Disconnected; results saved. Arduino may finish its current run.';h.run.Enable='off';
else,stopModel();end
end
function stopModel()
if bdIsLoaded('ultrasonic_localization')&&~strcmp(get_param('ultrasonic_localization','SimulationStatus'),'stopped')
    set_param('ultrasonic_localization','SimulationCommand','stop');
end
end
