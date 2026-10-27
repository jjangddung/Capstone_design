function m=parseUltraLine(line)
% CSV protocol v1. Malformed messages return [] and never become distances.
m=[];parts=split(strtrim(string(line)),',');
if numel(parts)~=9||parts(1)~="M"||parts(2)~="1",return;end
v=str2double(parts(3:8));
if any(~isfinite(v(1:5)))||any(v(1:5)<0)||any(mod(v(1:5),1)~=0),return;end
if v(3)<1||v(3)>6||v(4)>=2^32||v(5)>=2^32,return;end
if ~any(parts(9)==["OK","TIMEOUT","OUT_OF_RANGE","DEVICE_ERROR"]),return;end
if parts(9)=="OK" && (~isfinite(v(6))||v(6)<0),return;end
m=struct('session',v(1),'scan',v(2),'sensor',v(3),'trigger',v(4),...
    'done',v(5),'range',v(6)/1000,'ok',parts(9)=="OK",'status',parts(9));
end
