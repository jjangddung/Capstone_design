function saved=ultraSaveRunCsv(history,records,cfg,meta)
% One stationary experiment: unfiltered sensor data plus all estimator results.
% meta: id, expected, gt (wall metres), ended, tag (session timestamp).
n=numel(history);assert(numel(records)==n,'History/RAW row alignment mismatch.');
folder=fullfile(cfg.root,'results');if ~isfolder(folder),mkdir(folder);end
gt=nan(n,2);raw=nan(n,4);echo=nan(n,4);trigger=nan(n,4);hw=false(n,4);
used=false(n,4);status=strings(n,4);sample=zeros(n,1);boardRun=nan(n,1);period=nan(n,1);
corrected=nan(n,4);pos=nan(n,2,3);ok=false(n,3);state=strings(n,3);arduino=nan(n,2);
for j=1:n
    h=history(j);r=records{j};gt(j,:)=h.gt;raw(j,:)=h.raw(:)'*100;
    if isfield(r,'receivedRaw'),raw(j,:)=r.receivedRaw(:)'*100;end
    hw(j,:)=h.hw(:)';used(j,:)=h.result.valid(:)';corrected(j,:)=h.result.range(:)'*100;
    sample(j)=r.scan;
    if isfield(r,'run'),boardRun(j)=r.run;end
    if isfield(r,'echoUs'),echo(j,:)=r.echoUs(:)';end
    if isfield(r,'triggerUs'),trigger(j,:)=r.triggerUs(:)';end
    if isfield(r,'period'),period(j)=r.period;end
    if isfield(r,'arduinoPosition'),arduino(j,:)=100*r.arduinoPosition;end
    for i=1:4
        if isfield(r,'sensorStatus'),status(j,i)=r.sensorStatus(i);
        elseif hw(j,i),status(j,i)="OK";else,status(j,i)="TIMEOUT";end
    end
    for k=1:3
        ok(j,k)=h.result.success(k)&&all(isfinite(h.result.p(k,:)));
        if ok(j,k),pos(j,:,k)=100*h.result.p(k,:);end
        state(j,k)=h.result.status(k);
    end
end
complete=meta.ended&&n==meta.expected&&isequal(sort(sample),(1:meta.expected)');
T=table(repmat(meta.id,n,1),boardRun,sample,reshape([history.time],[],1),...
    gt(:,1)*100,gt(:,2)*100,period*1000,sum(used,2),...
    'VariableNames',{'Run_ID','Arduino_Run_ID','Sample_ID','Time_s','GT_X_cm','GT_Y_cm','ScanPeriod_ms','UsedSensor_N'});
for i=1:4
    prefix=sprintf('S%d_',i);
    T.([prefix 'RAW_cm'])=raw(:,i);T.([prefix 'Echo_us'])=echo(:,i);
    T.([prefix 'Time_us'])=trigger(:,i);T.([prefix 'Status'])=status(:,i);
    T.([prefix 'Used'])=used(:,i);T.([prefix 'Corrected_cm'])=corrected(:,i);
end
names=["LS";"LMedS";"MotionLMedS"];
S=table(repmat(meta.id,3,1),names,repmat(100*meta.gt(1),3,1),repmat(100*meta.gt(2),3,1),...
    repmat(meta.expected,3,1),repmat(n,3,1),repmat(complete,3,1),...
    'VariableNames',{'Run_ID','Algorithm','GT_X_cm','GT_Y_cm','Expected_N','Received_N','Complete'});
fields={'Success_N','GT_Samples','Failure_Rate','Mean_X_cm','Mean_Y_cm','Std_X_cm','Std_Y_cm',...
    'Bias_X_cm','Bias_Y_cm','Mean_Error_cm','Std_Error_cm','RMSE_cm','Max_Error_cm'};
for f=fields,S.(f{1})=nan(3,1);end
for k=1:3
    p=pos(:,:,k);delta=p-100*gt;err=vecnorm(delta,2,2);prefix=char(names(k));
    T.([prefix '_X_cm'])=p(:,1);T.([prefix '_Y_cm'])=p(:,2);
    T.([prefix '_ErrorX_cm'])=delta(:,1);T.([prefix '_ErrorY_cm'])=delta(:,2);
    T.([prefix '_Error_cm'])=err;T.([prefix '_Success'])=ok(:,k);T.([prefix '_Status'])=state(:,k);
    valid=ok(:,k);ev=valid&all(isfinite(gt),2);S.Success_N(k)=sum(valid);S.GT_Samples(k)=sum(ev);
    if n>0,S.Failure_Rate(k)=1-sum(valid)/n;end
    S.Mean_X_cm(k)=avg(p(valid,1));S.Mean_Y_cm(k)=avg(p(valid,2));
    S.Std_X_cm(k)=sd(p(valid,1));S.Std_Y_cm(k)=sd(p(valid,2));
    S.Bias_X_cm(k)=avg(delta(ev,1));S.Bias_Y_cm(k)=avg(delta(ev,2));
    S.Mean_Error_cm(k)=avg(err(ev));S.Std_Error_cm(k)=sd(err(ev));
    if any(ev),S.RMSE_cm(k)=sqrt(mean(err(ev).^2));S.Max_Error_cm(k)=max(err(ev));end
end
T.Arduino_X_cm=arduino(:,1);T.Arduino_Y_cm=arduino(:,2);
for i=1:4
    v=raw(hw(:,i)&isfinite(raw(:,i)),i);prefix=sprintf('S%d_RAW_',i);
    S.([prefix 'Valid_N'])=repmat(numel(v),3,1);
    S.([prefix 'Mean_cm'])=repmat(avg(v),3,1);
    S.([prefix 'Std_cm'])=repmat(sd(v),3,1);
end
if all(isfinite(meta.gt)),label=sprintf('GT_X%.3fcm_Y%.3fcm',100*meta.gt(1),100*meta.gt(2));
else,label='GT_UNKNOWN';end
suffix='';if ~complete,suffix='_INCOMPLETE';end
stem=sprintf('%s_Run%03d_N%d%s_%s',label,meta.id,n,suffix,meta.tag);
base=fullfile(folder,stem);v=1;
while isfile([base '_samples.csv'])||isfile([base '_stats.csv'])
    v=v+1;base=fullfile(folder,sprintf('%s_v%d',stem,v));
end
samples=[base '_samples.csv'];stats=[base '_stats.csv'];
writetable(T,samples);writetable(S,stats);
saved=struct('samples',samples,'stats',stats,'summary',S,'count',n,'complete',complete);
end
function y=avg(x)
if isempty(x),y=NaN;else,y=mean(x);end
end
function y=sd(x)
if numel(x)<2,y=NaN;else,y=std(x,0);end
end
