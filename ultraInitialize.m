function st=ultraInitialize(cfg)
st.p=[.5 .5]; st.command=-1; st.experiment=0; st.nextSensor=1; st.nextTime=0;
st.raw=nan(cfg.sensorCount,1); st.times=nan(cfg.sensorCount,1); st.hw=false(cfg.sensorCount,1); st.coverage=zeros(cfg.sensorCount,1);
st.externalRecords={};
st.gtTime=[]; st.gt=[]; st.track=[]; st.scanId=0;
st.commands=zeros(0,7);
st.rng=RandStream('mt19937ar','Seed',cfg.seed);
st.history=struct('time',{},'gt',{},'result',{},'raw',{},'sensorTimes',{},'hw',{},'experiment',{});
st.last=struct('p',nan(3,2),'success',false(3,1),'status',repmat("WAITING",3,1),...
    'range',nan(cfg.sensorCount,1),'valid',false(cfg.sensorCount,1),'tRef',NaN,'residual',nan(cfg.sensorCount,3),'inlier',false(cfg.sensorCount,3),'computeMs',zeros(3,1));
st.gtRef=[NaN NaN]; st.error=nan(3,1);
end
