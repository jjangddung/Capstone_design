function [st,y]=ultraExternalTick(st,t,scans,cfg)
fresh=false;
for j=1:numel(scans)
    sc=scans{j};
    if sc.reset,st.track=[];st.experiment=st.experiment+1;end
    [st.last,st.track]=ultraEstimate(sc.raw,sc.times,sc.hw,st.track,cfg);
    st.raw=sc.raw;st.times=sc.times;st.hw=sc.hw;st.scanId=sc.scan;
    st.p=[NaN NaN];st.gtRef=[NaN NaN];
    if isfield(sc,'gt')
        if all(isfinite(sc.gt)),st.p=sc.gt;st.gtRef=sc.gt;end
    elseif cfg.staticGTConfirmed&&all(isfinite(cfg.staticGT))
        st.p=cfg.staticGT;st.gtRef=cfg.staticGT;
    end
    st.error=vecnorm(st.last.p-st.gtRef,2,2);
    h=struct('time',st.last.tRef,'gt',st.gtRef,'result',st.last,...
        'raw',sc.raw,'sensorTimes',sc.times,'hw',sc.hw,'experiment',st.experiment);
    st.history(end+1)=h;fresh=true;
    st.externalRecords{end+1}=sc;
end
y=[st.p(:);st.gtRef(:);reshape(st.last.p',6,1);st.error;st.last.range;...
    double(st.last.valid);double(st.last.success);double(fresh);st.last.tRef;t;st.experiment];
end
