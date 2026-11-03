function tests=testRangeExtra
tests=functiontests(localfunctions);
end
function testBoundaries(tc)
c=ultraConfig();c.background(:)=NaN;
verifyEqual(tc,c.radius,.009);verifyEqual(tc,c.residualLimit,.025);
lo=.375-c.radius-c.residualLimit-c.roiRangeExtra;
hi=hypot(.575,.1)-c.radius+c.residualLimit+c.roiRangeExtra;
verifyEqual(tc,lo,.291,'AbsTol',1e-12);
for x=[lo+1e-7 hi-1e-7]
    [o,~]=ultraEstimate(repmat(x,4,1),zeros(4,1),true(4,1),[],c);
    verifyTrue(tc,all(o.valid));
end
for x=[lo-1e-7 hi+1e-7]
    [o,~]=ultraEstimate(repmat(x,4,1),zeros(4,1),true(4,1),[],c);
    verifyFalse(tc,any(o.valid));
end
end
function testObservedDistances(tc)
c=ultraConfig();c.background(:)=NaN;d=[34.008;50.232;61.346;52.205]/100;
[o,~]=ultraEstimate(d,zeros(4,1),true(4,1),[],c);verifyTrue(tc,all(o.valid));
c.roiRangeExtra=0;
[o,~]=ultraEstimate(d,zeros(4,1),true(4,1),[],c);verifyEqual(tc,o.valid,[false;true;false;true]);
end
