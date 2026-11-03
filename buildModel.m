function model=buildModel(cfg)
% Rebuild only this project's generated model. No third-party models changed.
if nargin<1,cfg=ultraConfig();end
b=cfg.roi;w=cfg.bodySize; m=cfg.clearance;
assert(all(isfinite([b w cfg.radius]))&&cfg.radius>=0&&all(w>0),'Invalid geometry.');
assert(b(1)<b(2)&&b(3)<b(4)&&b(1)<=.5&&b(2)>=.5&&b(3)<=.5&&b(4)>=.5,'ROI must include initial center.');
assert(b(1)>=w(1)/2+m&&b(2)<=1-w(1)/2-m&&b(3)>=w(2)/2+m&&b(4)<=1-w(2)/2-m,...
    'ROI would allow body/wall collision. Update bodySize, clearance, and ROI.');
assert(cfg.slot>=cfg.Ts&&abs(cfg.slot/cfg.Ts-round(cfg.slot/cfg.Ts))<1e-8,'slot must be a positive integer multiple of Ts.');
model='ultrasonic_localization';
if bdIsLoaded(model),close_system(model,0);end
assignin('base','ultraCfg',cfg);
load_system('simulink');new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete',...
    'FixedStep',num2str(cfg.Ts),'StopTime',num2str(cfg.stopTime),...
    'ReturnWorkspaceOutputs','on');
add_block('simulink/Sources/Digital Clock',[model '/Time'],'SampleTime',num2str(cfg.Ts),'Position',[40 70 100 100]);
add_block('simulink/Sources/Constant',[model '/UserCommand'],...
    'Value',sprintf('[0.5 0.5 0 1 %d 1]',cfg.defaultCoverageMode),'SampleTime',num2str(cfg.Ts),'Position',[30 180 220 225]);
add_block('simulink/User-Defined Functions/MATLAB System',[model '/UltrasonicPipeline'],...
    'System','UltraSystem','Position',[300 85 540 230]);
set_param([model '/UltrasonicPipeline'],'SimulateUsing','Interpreted execution');
set_param([model '/UltrasonicPipeline'],'Config','ultraCfg');
add_block('simulink/Sinks/To Workspace',[model '/SignalLog'],...
    'VariableName','ultraSignals','SaveFormat','Timeseries','Position',[660 95 790 135]);
add_block('simulink/Sinks/Display',[model '/GT_Estimates_Status'],...
    'Position',[650 185 810 335]);
add_line(model,'Time/1','UltrasonicPipeline/1','autorouting','on');
add_line(model,'UserCommand/1','UltrasonicPipeline/2','autorouting','on');
add_line(model,'UltrasonicPipeline/1','SignalLog/1','autorouting','on');
add_line(model,'UltrasonicPipeline/1','GT_Estimates_Status/1','autorouting','on');
note=Simulink.Annotation(model,['Run launchUltra for position controls and GT/estimate plot.' newline ...
    'Coverage: default fixed 38 deg total (assumption); ideal / legacy optional.' newline ...
    'Host interpreted MATLAB System; raw sensors -> LS / LMedS / Motion LMedS.']);
note.Position=[30 365 790 430];
set_param(model,'InitFcn','assert(exist(''ultraCfg'',''var'')==1,''Run launchUltra first.'');');
save_system(model,fullfile(cfg.root,[model '.slx']));
end
