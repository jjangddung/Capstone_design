function tests=testFourWalls
tests=functiontests(localfunctions);
end
function testInsetCenterRange(tc)
c=ultraConfig();[origin,span]=ultraSensorFrame(c);
verifyEqual(tc,origin,[.025 .025],'AbsTol',1e-12);
verifyEqual(tc,span,[.95 .95],'AbsTol',1e-12);
verifyEqual(tc,ultraRange([.5 .5],c),repmat(.475-c.radius,4,1),'AbsTol',1e-12);
end
function testLayoutAndCoverage(tc)
c=ultraConfig();
verifyEqual(tc,c.sensorXY,[.025 .5;.975 .5;.5 .025;.5 .975]);
verifyEqual(tc,c.direction,[1 0;-1 0;0 1;0 -1]);
for x=linspace(.4,.6,5)
    for y=linspace(.4,.6,5)
        s=ultraInitialize(c);
        for t=0:c.Ts:.2,[s,~]=ultraTick(s,t,[x y 0 1 2 1],c);end
        verifyEqual(tc,s.coverage,ones(4,1));
        verifyTrue(tc,all(s.last.success));
        verifyLessThan(tc,max(vecnorm(s.last.p-[x y],2,2)),1e-7);
    end
end
end
function testMissingSensors(tc)
c=ultraConfig();p=[.48 .52];d=ultraRange(p,c);
for missing=1:4
    hw=true(4,1);hw(missing)=false;
    [o,~]=ultraEstimate(d,zeros(4,1),hw,[],c);
    verifyTrue(tc,all(o.success));
    verifyLessThan(tc,max(vecnorm(o.p-p,2,2)),1e-7);
end
[o,~]=ultraEstimate(d,zeros(4,1),[true;true;false;false],[],c);
verifyFalse(tc,any(o.success));
end
function testFourRecordScan(tc)
c=ultraConfig();file=[tempname '.csv'];fid=fopen(file,'w');
fprintf(fid,'M,1,1,1,5,0,4000,500,OK\n');
for i=1:4
    fprintf(fid,'M,1,1,1,%d,%d,%d,495,OK\n',i,(i-1)*60000,(i-1)*60000+4000);
end
fclose(fid);clean=onCleanup(@()delete(file)); %#ok<NASGU>
c.inputMode='replay';c.replayFile=file;a=UltraInput(c);cl=onCleanup(@()delete(a)); %#ok<NASGU>
verifyEmpty(tc,a.poll(.12));sc=a.poll(.18);
verifyEqual(tc,numel(sc),1);verifySize(tc,sc{1}.raw,[4 1]);
verifyTrue(tc,all(sc{1}.seen));verifyEqual(tc,a.RejectedLines,1);
verifyEmpty(tc,parseUltraLine('M,1,1,1,6,0,4000,500,OK'));
end
