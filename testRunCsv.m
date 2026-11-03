function tests=testRunCsv
tests=functiontests(localfunctions);
end
function [h,r,c]=fixture()
c=ultraConfig();c.root=tempname;mkdir(c.root);s=ultraInitialize(c);
for i=1:3
    sc=struct('scan',i,'run',1,'raw',ultraRange([.5 .5],c),'times',repmat(i,4,1),...
        'hw',true(4,1),'reset',i==1,'gt',[.5 .5],'period',1,'triggerUs',repmat(i*1000000,4,1),...
        'echoUs',repmat(2700,4,1),'receivedRaw',ultraRange([.5 .5],c),...
        'sensorStatus',repmat("OK",4,1),'exportToken',1);
    [s,~]=ultraExternalTick(s,i,{sc},c);
end
h=s.history;r=s.externalRecords;
for i=1:3,h(i).result.p=repmat([.5+i*.01 .5],3,1);h(i).result.success(:)=true;end
end
function testStatsAndRaw(tc)
[h,r,c]=fixture();
r{2}.receivedRaw(1)=9;h(2).raw(1)=9;h(2).result.valid(1)=false;
h(3).result.success(2)=false; % Must not export a failed finite estimate as valid.
m=struct('id',1,'expected',3,'gt',[.5 .5],'ended',true,'tag','TEST');
o=ultraSaveRunCsv(h,r,c,m);T=readtable(o.samples);S=readtable(o.stats);
verifyEqual(tc,height(T),3);verifyTrue(tc,o.complete);
verifyEqual(tc,T.S1_RAW_cm(2),900);verifyFalse(tc,logical(T.S1_Used(2)));
verifyEqual(tc,S.S1_RAW_Valid_N(1),3);verifyEqual(tc,S.Mean_X_cm(1),52,'AbsTol',1e-10);
verifyEqual(tc,S.Std_X_cm(1),1,'AbsTol',1e-10);verifyEqual(tc,S.Mean_Error_cm(1),2,'AbsTol',1e-10);
verifyEqual(tc,S.RMSE_cm(1),sqrt(14/3),'AbsTol',1e-10);
verifyTrue(tc,isnan(T.LMedS_X_cm(3)));verifyEqual(tc,S.Success_N(2),2);
verifyEqual(tc,S.Failure_Rate(2),1/3,'AbsTol',1e-10);
verifySubstring(tc,o.samples,'GT_X50.000cm_Y50.000cm_Run001_N3_');
o2=ultraSaveRunCsv(h,r,c,m);verifyNotEqual(tc,o.samples,o2.samples);verifyTrue(tc,isfile(o.samples));
end
function testPartialAndEmpty(tc)
[h,r,c]=fixture();m=struct('id',2,'expected',30,'gt',[.5 .5],'ended',false,'tag','TEST');
o=ultraSaveRunCsv(h(1),r(1),c,m);verifySubstring(tc,o.samples,'N1_INCOMPLETE');
verifyTrue(tc,isnan(o.summary.Std_X_cm(1)));
o=ultraSaveRunCsv(h([]),{},c,m);verifyEqual(tc,o.count,0);verifyEqual(tc,o.summary.Success_N,zeros(3,1));
verifyTrue(tc,all(isnan(o.summary.Failure_Rate)));
end
function testAutoSaveThirtyAndGTLock(tc)
c=ultraConfig();c.root=tempname;mkdir(c.root);c.inputMode='serial';c.background(:)=NaN;
ultraView('open',[],0,[],c);f=findall(groot,'Tag','UltraLocalizationViewer');
p=UltraTestTransport();s=UltraLiveSession(c,f,p);setappdata(f,'UltraLiveSession',s);
cl=onCleanup(@()ultraCloseView(f)); %#ok<NASGU>
p.inject('# READY');waitFor(@()s.Input.Ready);
s.measure([.5 .5]);h=f.UserData;verifyEqual(tc,h.x.Enable,'off');
p.inject('# Geometry Lx=95.00 Ly=95.00');waitFor(@()numel(p.Sent)>=2);
p.inject('# START,run=1,point=P1,cycles=30');
for i=1:30
    ts=(i-1)*252000+(0:3)*63000;
    d=ultraRange([.5 .5],c)*100;
    line=sprintf(['P1,1,%d,47.5,47.5,%.6f,%.6f,%.6f,%.6f,OK,OK,OK,OK,' ...
        '%d,%d,%d,%d,2700,2700,2700,2700,47.5,47.5,0'],i,d,ts);
    p.inject(line);
end
p.inject('# END,run=1,point=P1');p.inject('# READY');
waitFor(@()numel(s.SavedRuns)==1);
o=s.SavedRuns{1};verifyEqual(tc,o.count,30);verifyTrue(tc,o.complete);
T=readtable(o.samples);verifyEqual(tc,T.GT_X_cm,repmat(50,30,1));
verifyEqual(tc,T.S1_RAW_cm,repmat(100*(.475-c.radius),30,1),'AbsTol',1e-8);
verifyEqual(tc,T.S1_Echo_us,repmat(2700,30,1));
verifyEqual(tc,h.x.Enable,'on');verifyTrue(tc,any(contains(string(cellstr(h.saved.String)),'GT_X50.000')));
folder=fullfile(fileparts(mfilename('fullpath')),'results');if ~isfolder(folder),mkdir(folder);end
drawnow;fr=getframe(f);imwrite(fr.cdata,fullfile(folder,'run_export_preview.png'));
% Change GT for a second run, then disconnect before the 30 samples finish.
s.measure([.55 .45]);p.inject('# Geometry Lx=95.00 Ly=95.00');pause(.2);
p.inject('# START,run=2,point=P1,cycles=30');
for i=1:2
    ts=8000000+(i-1)*252000+(0:3)*63000;d=ultraRange([.55 .45],c)*100;
    p.inject(sprintf(['P1,2,%d,52.5,42.5,%.6f,%.6f,%.6f,%.6f,OK,OK,OK,OK,' ...
        '%d,%d,%d,%d,2700,2700,2700,2700,52.5,42.5,0'],i,d,ts));
end
waitFor(@()numel(s.State.history)==32);s.stop();
verifyEqual(tc,numel(s.SavedRuns),2); % No duplicate of completed run 1.
o2=s.SavedRuns{2};verifyFalse(tc,o2.complete);verifyEqual(tc,o2.count,2);
verifySubstring(tc,o2.samples,'GT_X55.000cm_Y45.000cm_Run002_N2_INCOMPLETE');
T2=readtable(o2.samples);verifyEqual(tc,T2.GT_X_cm,[55;55],'AbsTol',1e-8);
end
function waitFor(fn)
t=tic;
while ~fn()&&toc(t)<6,pause(.05);drawnow;end
assert(fn(),'Timed out waiting for timer test.');
end
