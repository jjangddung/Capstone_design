function tests=testBeam38
tests=functiontests(localfunctions);
end
function testEdges(tc)
c=ultraConfig();verifyEqual(tc,c.beamFullAngleDeg,38);verifyEqual(tc,c.defaultCoverageMode,2);
verifyTrue(tc,ultraFixedCoverage(.4,.4*tand(19),c));
verifyTrue(tc,ultraFixedCoverage(.4,-.4*tand(19),c));
verifyFalse(tc,ultraFixedCoverage(.4,.14,c));
verifyFalse(tc,ultraFixedCoverage(.4,-.14,c));
verifyFalse(tc,ultraFixedCoverage(-.4,0,c));
verifyFalse(tc,ultraFixedCoverage(0,0,c));
verifyFalse(tc,ultraFixedCoverage(NaN,0,c));
end
function testBeyondOldRange(tc)
c=ultraConfig();verifyTrue(tc,ultraFixedCoverage(.7,.7*tand(18),c));
verifyFalse(tc,ultraFixedCoverage(.7,.7*tand(20),c));
s=ultraInitialize(c);
for t=0:c.Ts:.4,[s,~]=ultraTick(s,t,[.5 .5 0 1 2 1],c);end
verifyEqual(tc,s.coverage,ones(c.sensorCount,1));verifyTrue(tc,all(s.last.success));
end
function testRotatedSensor(tc)
c=ultraConfig();i=1;u=c.direction(i,:);v=[-u(2) u(1)];
for angle=[18 20 -18 -20]
    p=c.sensorXY(i,:)+.4*u+.4*tand(angle)*v;
    delta=p-c.sensorXY(i,:);
    verifyEqual(tc,ultraFixedCoverage(dot(delta,u),dot(delta,v),c),abs(angle)<19);
end
end
