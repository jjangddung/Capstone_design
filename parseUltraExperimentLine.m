function row=parseUltraExperimentLine(line)
% User's 24-column Arduino experiment CSV. Distances/GT converted cm -> m.
row=[];p=split(strtrim(string(line)),',');
if numel(p)~=24||~any(p(1)==["P1","P2","P3","P4","P5","EMPTY","ROBOT","TARGET"]),return;end
v=str2double(p([2:9 14:24]));
run=v(1);cycle=v(2);gt=v(3:4)';d=v(5:8);ts=v(9:12);echo=v(13:16);
status=p(10:13);
if any(~isfinite([run cycle]))||any([run cycle]<1)||any(mod([run cycle],1)~=0),return;end
if ~(all(isfinite(gt)&gt>=0)||all(isnan(gt))),return;end
if any(~ismember(status,["OK","TIMEOUT"])),return;end
if any(~isfinite(ts)|ts<0|ts>=2^32|mod(ts,1)~=0),return;end
if any(~isfinite(echo)|echo<0|echo>=2^32|mod(echo,1)~=0),return;end
ok=status=="OK";
if any(ok&(~isfinite(d)|d<0)),return;end
receivedRaw=d/100;
d(~ok)=NaN;
row=struct('point',p(1),'run',run,'scan',cycle,'raw',d/100,'hw',ok,...
    'receivedRaw',receivedRaw,'sensorStatus',status,'gt',gt/100,'triggerUs',ts,'echoUs',echo,...
    'arduinoPosition',v(17:18)'/100,'arduinoError',v(19)/100);
end
