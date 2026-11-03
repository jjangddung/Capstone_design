function tests=testArduinoExperiment
tests=functiontests(localfunctions);
end
function line=row(run,cycle,p,ts,status)
c=ultraConfig();d=100*ultraRange(p,c);
line=sprintf('P1,%d,%d,%.6f,%.6f,',run,cycle,100*(p(1)-.025),100*(p(2)-.025));
for i=1:4
    if status(i)=="OK",line=[line sprintf('%.9f,',d(i))];else,line=[line 'NaN,'];end %#ok<AGROW>
end
line=[line char(join(status,',')) ','];
for i=1:4,line=[line sprintf('%.0f,',ts(i))];end %#ok<AGROW>
line=[line '2900,2900,2900,2900,50,50,0'];
end
function testParse(tc)
l=row(1,1,[.48 .52],[0 63000 126000 189000],repmat("OK",1,4));r=parseUltraExperimentLine(l);
verifyEqual(tc,r.gt,[.455 .495],'AbsTol',1e-10);verifySize(tc,r.raw,[4 1]);
verifyEqual(tc,r.raw,ultraRange([.48 .52],ultraConfig()),'AbsTol',1e-10);
verifyEmpty(tc,parseUltraExperimentLine('# READY'));
verifyEmpty(tc,parseUltraExperimentLine('point_id,run_id'));
verifyEmpty(tc,parseUltraExperimentLine(l+",EXTRA"));
verifyEmpty(tc,parseUltraExperimentLine(strrep(l,'OK','BOGUS')));
end
function testHandshakeAndSerial(tc)
c=ultraConfig();c.inputMode='serial';port=UltraTestTransport();a=UltraExperimentInput(c,port);
cl=onCleanup(@()delete(a)); %#ok<NASGU>
a.poll(0);verifyEqual(tc,port.Sent,"HELP");
verifyError(tc,@()startBeforeReady(a),'Ultra:NotReady');
port.inject('# Four Sensor Localization Experiment');port.inject('# READY');a.poll(.1);verifyTrue(tc,a.Ready);
a.startRun([.48 .52],.2);verifyFalse(tc,a.Ready);verifyEqual(tc,port.Sent(end),"G 95.000000 95.000000");
port.inject('# Geometry Lx=95.00 Ly=95.00');a.poll(.3);
verifyEqual(tc,port.Sent(end),"P1 45.500000 49.500000");
port.inject('# START,run=1,point=P1,cycles=30');
port.inject(row(1,1,[.48 .52],[0 63000 126000 189000],repmat("OK",1,4)));
sc=a.poll(.5);verifyEqual(tc,numel(sc),1);
s=ultraInitialize(c);[s,~]=ultraExternalTick(s,.5,sc,c);
verifyTrue(tc,all(s.last.success));verifyEqual(tc,s.gtRef,[.48 .52],'AbsTol',1e-10);
verifyLessThan(tc,max(s.error),1e-8);
port.inject('# END,run=1,point=P1');port.inject('# READY');a.poll(.6);verifyTrue(tc,a.Ready);
end
function startBeforeReady(a),a.startRun([.5 .5],0);end
function testReplayRunsAndTimeout(tc)
file=[tempname '.csv'];fid=fopen(file,'w');
fprintf(fid,'# COORDINATE_ORIGIN_M,0.025,0.025\n');
fprintf(fid,'%s\n',row(1,1,[.4 .4],[0 63000 126000 189000],["OK","TIMEOUT","OK","OK"]));
fprintf(fid,'%s\n',row(1,2,[.4 .4],[252000 315000 378000 441000],repmat("OK",1,4)));
fprintf(fid,'# START,run=2,point=P1,cycles=30\n');
fprintf(fid,'%s\n',row(2,1,[.6 .6],[1000000 1063000 1126000 1189000],repmat("OK",1,4)));
fclose(fid);cl=onCleanup(@()delete(file)); %#ok<NASGU>
c=ultraConfig();c.inputMode='replay';c.replayFile=file;a=UltraExperimentInput(c);cl2=onCleanup(@()delete(a)); %#ok<NASGU>
verifyEmpty(tc,a.poll(.1));sc=a.poll(.2);verifyEqual(tc,numel(sc),1);verifyFalse(tc,sc{1}.hw(2));
s=ultraInitialize(c);[s,~]=ultraExternalTick(s,.2,sc,c);verifyTrue(tc,all(s.last.success));
sc=a.poll(.5);verifyEqual(tc,sc{1}.period,.252,'AbsTol',1e-10);
[s,~]=ultraExternalTick(s,.5,sc,c);
sc=a.poll(1.2);[s,~]=ultraExternalTick(s,1.2,sc,c);
verifyEqual(tc,s.experiment,2);verifyTrue(tc,all(s.last.success));verifyLessThan(tc,max(s.error),1e-8);
end
function testRolloverDuplicateAndReset(tc)
c=ultraConfig();c.inputMode='serial';p=UltraTestTransport();a=UltraExperimentInput(c,p);cl=onCleanup(@()delete(a)); %#ok<NASGU>
ts=mod(2^32-60000+(0:3)*63000,2^32);
l=row(1,1,[.5 .5],ts,repmat("OK",1,4));p.inject(l);sc=a.poll(0);
verifyEqual(tc,diff(sc{1}.times),repmat(.063,3,1),'AbsTol',1e-9);
p.inject(l);verifyEmpty(tc,a.poll(.1));verifyEqual(tc,a.RejectedLines,1);
p.inject('# Four Sensor Localization Experiment');p.inject('# READY');
p.inject(row(1,1,[.5 .5],[0 63000 126000 189000],repmat("OK",1,4)));
sc2=a.poll(.5);verifyTrue(tc,sc2{1}.reset);verifyGreaterThan(tc,sc2{1}.times(1),sc{1}.times(end));
end
function testUnavailableAndMalformed(tc)
c=ultraConfig();c.inputMode='serial';p=UltraTestTransport();a=UltraExperimentInput(c,p);cl=onCleanup(@()delete(a)); %#ok<NASGU>
p.inject('corrupted line');verifyEmpty(tc,a.poll(0));verifyEqual(tc,a.RejectedLines,1);
a.poll(16);verifyFalse(tc,a.Ready);verifySubstring(tc,a.Status,'timeout');
end
function testLiveTimerAndView(tc)
c=ultraConfig();c.inputMode='serial';c.background(:)=NaN;
ultraView('open',[],0,[],c);f=findall(groot,'Tag','UltraLocalizationViewer');
port=UltraTestTransport();s=UltraLiveSession(c,f,port);setappdata(f,'UltraLiveSession',s);
cl=onCleanup(@()ultraCloseView(f)); %#ok<NASGU>
port.inject('# READY');pause(.3);drawnow;verifyTrue(tc,s.Input.Ready);
s.measure([.48 .52]);port.inject('# Geometry Lx=95.00 Ly=95.00');pause(.2);
port.inject('# START,run=1,point=P1,cycles=30');
port.inject(row(1,1,[.48 .52],[0 63000 126000 189000],repmat("OK",1,4)));
pause(.4);drawnow;
verifyTrue(tc,all(s.State.last.success));verifyLessThan(tc,max(s.State.error),1e-8);
h=f.UserData;verifyEqual(tc,h.table.Data{2,5},'VALID');
s.stop();verifyFalse(tc,s.Active);
end
