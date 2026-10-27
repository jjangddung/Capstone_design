function tests=testUltra
tests=functiontests(localfunctions);
end
function testKnownPositions(tc)
c=ultraConfig();
for p=[.5 .5;.4 .42;.6 .58;.35 .65]'
    d=ultraRange(p,c);[o,~]=ultraEstimate(d,zeros(6,1),true(6,1),[],c);
    verifyTrue(tc,all(o.success));verifyLessThan(tc,max(vecnorm(o.p-p',2,2)),1e-7);
end
end
function testRadiusAndCalibration(tc)
c=ultraConfig();c.radius=.02;c.alpha=(1:.01:1.05)';c.beta=(0:.001:.005)';p=[.47 .53];
d=c.alpha.*ultraRange(p,c)+c.beta;[o,~]=ultraEstimate(d,zeros(6,1),true(6,1),[],c);
verifyLessThan(tc,max(vecnorm(o.p-p,2,2)),1e-7);
end
function testOutlier(tc)
c=ultraConfig();p=[.49 .52];d=ultraRange(p,c);d(2)=d(2)+.1;
[o,~]=ultraEstimate(d,zeros(6,1),true(6,1),[],c);
verifyTrue(tc,all(o.success));verifyLessThan(tc,norm(o.p(2,:)-p),1e-6);
verifyGreaterThan(tc,norm(o.p(1,:)-p),norm(o.p(2,:)-p)+1e-3);
end
function testNoDataAndWall(tc)
c=ultraConfig();[o,~]=ultraEstimate(nan(6,1),zeros(6,1),false(6,1),[],c);
verifyFalse(tc,any(o.success));verifyTrue(tc,all(isnan(o.p),'all'));
[o,~]=ultraEstimate(c.background,zeros(6,1),true(6,1),[],c);verifyFalse(tc,any(o.valid));
end
function testMeasuredUnknown(tc)
c=ultraConfig();s=ultraInitialize(c);
for t=0:c.Ts:.4,[s,y]=ultraTick(s,t,[.5 .5 0 1 2 1],c);end %#ok<ASGLU>
verifyEqual(tc,s.coverage,3*ones(6,1));verifyFalse(tc,any(s.last.success));
end
function testFreshAndTeleport(tc)
c=ultraConfig();s=ultraInitialize(c);count=0;
for t=0:c.Ts:.7
    [s,y]=ultraTick(s,t,[.48 .52 0 1 1 1],c);count=count+y(29);
end
verifyEqual(tc,count,2);verifyEqual(tc,numel(s.history),2);
[s,y]=ultraTick(s,.71,[.6 .4 0 2 1 1],c);
verifyEqual(tc,s.p,[.6 .4]);verifyFalse(tc,any(s.last.success));verifyEqual(tc,y(29),0);
end
function testVectorSpeed(tc)
c=ultraConfig();s=ultraInitialize(c);old=s.p;
[s,~]=ultraTick(s,0,[.6 .6 1 1 1 1],c);
verifyLessThanOrEqual(tc,norm(s.p-old),c.vSim*c.Ts+1e-12);
end
function testParser(tc)
m=parseUltraLine('M,1,7,120,1,12000000,12004000,640,OK');
verifyEqual(tc,m.range,.64);verifyEqual(tc,m.sensor,1);
verifyEmpty(tc,parseUltraLine('M,1,7,120,9,0,0,640,OK'));
verifyEmpty(tc,parseUltraLine('M,1,7,120,1,0,0,nan,OK'));
m=parseUltraLine('M,1,7,120,1,0,0,nan,TIMEOUT');verifyFalse(tc,m.ok);
end
function testMotionRejectAndRecovery(tc)
c=ultraConfig();[~,tr]=ultraEstimate(ultraRange([.4 .4],c),zeros(6,1),true(6,1),[],c);
[o,tr]=ultraEstimate(ultraRange([.6 .6],c),.1*ones(6,1),true(6,1),tr,c);
verifyFalse(tc,o.success(3));
for t=[.2 .3 .4 .5 .6]
    [o,tr]=ultraEstimate(ultraRange([.6 .6],c),t*ones(6,1),true(6,1),tr,c);
end
verifyTrue(tc,o.success(3));
end
function testTimeSpan(tc)
c=ultraConfig();[o,~]=ultraEstimate(ultraRange([.5 .5],c),(0:5)',true(6,1),[],c);
verifyEqual(tc,o.status,repmat("TIME_SPAN_TOO_LARGE",3,1));
end
function testColumnCommand(tc)
c=ultraConfig();s=ultraInitialize(c);
[s,y]=ultraTick(s,0,[.5;.5;0;1;1;1],c);
verifySize(tc,y,[32 1]);verifyEqual(tc,s.p,[.5 .5]);
end
function testReplayEquivalence(tc)
c=ultraConfig();p=[.46 .54];file=[tempname '.csv'];fid=fopen(file,'w');
d=ultraRange(p,c);
for i=1:6,fprintf(fid,'M,1,1,1,%d,%d,%d,%.9f,OK\n',i,(i-1)*60000,(i-1)*60000+4000,1000*d(i));end
fclose(fid);cleaner=onCleanup(@()delete(file)); %#ok<NASGU>
c.inputMode='replay';c.replayFile=file;adapter=UltraInput(c);cl=onCleanup(@()delete(adapter)); %#ok<NASGU>
sc=adapter.poll(1);verifyEqual(tc,numel(sc),1);
[a,~]=ultraEstimate(sc{1}.raw,sc{1}.times,sc{1}.hw,[],c);
[b,~]=ultraEstimate(d,(0:5)'*.06,true(6,1),[],c);
verifyEqual(tc,a.p,b.p,'AbsTol',1e-9);
s=ultraInitialize(c);[s,~]=ultraExternalTick(s,1,sc,c);
verifyTrue(tc,all(isnan(s.gtRef)));verifyTrue(tc,all(isnan(s.error)));
end
function testRolloverAndMissing(tc)
c=ultraConfig();file=[tempname '.csv'];fid=fopen(file,'w');
fprintf(fid,'M,1,1,1,1,4294900000,4294904000,500,OK\n');
fprintf(fid,'M,1,1,1,2,12704,16704,500,OK\n');fclose(fid);
cleaner=onCleanup(@()delete(file)); %#ok<NASGU>
c.inputMode='replay';c.replayFile=file;a=UltraInput(c);cl=onCleanup(@()delete(a)); %#ok<NASGU>
verifyEmpty(tc,a.poll(0));verifyEmpty(tc,a.poll(.1));sc=a.poll(1.1);
verifyEqual(tc,numel(sc),1);verifyEqual(tc,sum(sc{1}.seen),2);
verifyGreaterThan(tc,sc{1}.times(2),sc{1}.times(1));
end
