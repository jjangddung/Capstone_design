function summary=ultraSummary(history,file)
names=["LS";"LMedS";"MotionLMedS"];N=numel(history);
successCount=zeros(3,1);gtCount=zeros(3,1);rmseCm=nan(3,1);maxCm=nan(3,1);meanMs=nan(3,1);
for k=1:3
    errors=[];ms=[];
    for j=1:N
        h=history(j);ms(end+1)=h.result.computeMs(k); %#ok<AGROW>
        if h.result.success(k)
            successCount(k)=successCount(k)+1;
            if all(isfinite(h.gt)),errors(end+1)=norm(h.result.p(k,:)-h.gt)*100;end %#ok<AGROW>
        end
    end
    gtCount(k)=numel(errors);
    if ~isempty(errors),rmseCm(k)=sqrt(mean(errors.^2));maxCm(k)=max(errors);end
    if ~isempty(ms),meanMs(k)=mean(ms);end
end
failureRate=nan(3,1);if N>0,failureRate=1-successCount/N;end
summary=table(names,repmat(N,3,1),successCount,gtCount,rmseCm,maxCm,failureRate,meanMs,...
    'VariableNames',{'Algorithm','Scans','Successes','GT_Samples','RMSE_cm','MaxError_cm','FailureRate','MeanCompute_ms'});
if nargin>1,writetable(summary,file);end
end
