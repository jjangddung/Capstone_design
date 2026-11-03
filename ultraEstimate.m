function [out,track] = ultraEstimate(raw,times,hwOK,track,cfg)
% Same estimator for simulated, replayed, and Arduino measurements. No GT input.
if isempty(track)
    track=struct('p',[NaN NaN],'t',NaN,'fail',0,'pending',[NaN NaN]);
end
out=struct('p',nan(3,2),'success',false(3,1),'status',repmat("INSUFFICIENT",3,1),...
    'range',(raw(:)-cfg.beta)./cfg.alpha,'valid',false(cfg.sensorCount,1),...
    'tRef',NaN,'residual',nan(cfg.sensorCount,3),'inlier',false(cfg.sensorCount,3),'computeMs',zeros(3,1));
d=out.range;
valid=hwOK(:)&isfinite(d)&isfinite(times(:))&d>=cfg.rangeLimits(1)&d<=cfg.rangeLimits(2);
rangeMargin=cfg.residualLimit;
if isfield(cfg,'roiRangeExtra'),rangeMargin=rangeMargin+cfg.roiRangeExtra;end
for i=1:cfg.sensorCount
    if isfinite(cfg.background(i)) && abs(d(i)-cfg.background(i))<=cfg.backgroundTolerance(i)
        valid(i)=false;
    end
    s=cfg.sensorXY(i,:); b=cfg.roi;
    nearest=[min(max(s(1),b(1)),b(2)),min(max(s(2),b(3)),b(4))];
    corners=[b(1) b(3);b(1) b(4);b(2) b(3);b(2) b(4)];
    lo=norm(nearest-s)-cfg.radius-rangeMargin;
    hi=max(vecnorm(corners-s,2,2))-cfg.radius+rangeMargin;
    valid(i)=valid(i)&&d(i)>=lo&&d(i)<=hi;
end
out.valid=valid; ids=find(valid);
if isempty(ids), track.fail=track.fail+1; return; end
out.tRef=mean(times(ids));
if numel(ids)<3, track.fail=track.fail+1; return; end
if cfg.vMax*(max(times(ids))-min(times(ids)))>cfg.maxScanMotion
    out.status(:)="TIME_SPAN_TOO_LARGE"; track.fail=track.fail+1; return;
end
timer=tic; p=fitPosition(ids,d,cfg);
if goodGeometry(p,ids,cfg)
    out.p(1,:)=p; out.success(1)=true; out.status(1)="VALID";
    out.residual(ids,1)=d(ids)-ultraRange(p,cfg,ids); out.inlier(ids,1)=true;
else
    out.status(1)="POOR_GEOMETRY";
end
out.computeMs(1)=1000*toc(timer);
timer=tic; combos=nchoosek(ids,3); candidates=nan(size(combos,1),2); scores=inf(size(combos,1),1);
for k=1:size(combos,1)
    q=fitPosition(combos(k,:),d,cfg);
    if goodGeometry(q,combos(k,:),cfg)
        candidates(k,:)=q; scores(k)=median((d(ids)-ultraRange(q,cfg,ids)).^2);
    end
end
[p,inliers,ok]=chooseCandidate(candidates,scores,ids,d,cfg,[]);
out.computeMs(2)=1000*toc(timer);
if ok
    out.p(2,:)=p; out.success(2)=true; out.status(2)="VALID";
    out.inlier(inliers,2)=true; out.residual(ids,2)=d(ids)-ultraRange(p,cfg,ids);
else
    out.status(2)="NO_CONSENSUS";
end
timer=tic;
hasPrevious=all(isfinite(track.p));
reinit=hasPrevious&&(track.fail>=3||out.tRef-track.t>cfg.reinitializeAfter);
gate=[];
if hasPrevious && ~reinit
    gate=[track.p cfg.vMax*max(0,out.tRef-track.t)+cfg.motionMargin];
end
[p,inliers,ok]=chooseCandidate(candidates,scores,ids,d,cfg,gate);
if reinit && ok
    if ~all(isfinite(track.pending)) || norm(p-track.pending)>cfg.motionMargin+cfg.vMax*.5
        track.pending=p; ok=false; out.status(3)="REINITIALIZING";
    end
end
if ok
    out.p(3,:)=p; out.success(3)=true; out.status(3)="VALID";
    out.inlier(inliers,3)=true; out.residual(ids,3)=d(ids)-ultraRange(p,cfg,ids);
    track.p=p; track.t=out.tRef; track.fail=0; track.pending=[NaN NaN];
else
    track.fail=track.fail+1;
    if out.status(3)~="REINITIALIZING", out.status(3)="MOTION_REJECTED"; end
end
out.computeMs(3)=1000*toc(timer);
end

function p=fitPosition(ids,d,cfg)
s=cfg.sensorXY(ids,:); D=d(ids)+cfg.radius;
A=2*(s(2:end,:)-s(1,:));
b=sum(s(2:end,:).^2,2)-sum(s(1,:).^2)-D(2:end).^2+D(1)^2;
if rank(A)<2, p=[NaN NaN]; return; end
p=(A\b)'; p=clip(p,cfg.roi);
for iter=1:30
    v=p-s; n=vecnorm(v,2,2); if any(n<1e-8), p=[NaN NaN]; return; end
    J=v./n; residual=n-cfg.radius-d(ids);
    delta=(J'*J+1e-9*eye(2))\(J'*residual);
    old=sum(residual.^2); accepted=false;
    for scale=[1 .5 .25 .125 .0625]
        q=clip(p-scale*delta',cfg.roi);
        if sum((ultraRange(q,cfg,ids)-d(ids)).^2)<=old+1e-14
            accepted=true; break;
        end
    end
    if ~accepted || norm(q-p)<1e-10, break; end
    p=q;
end
end
function p=clip(p,b)
p=[min(max(p(1),b(1)),b(2)),min(max(p(2),b(3)),b(4))];
end
function ok=goodGeometry(p,ids,cfg)
ok=false; if any(~isfinite(p)), return; end
v=p-cfg.sensorXY(ids,:); n=vecnorm(v,2,2); if any(n<1e-8),return;end
s=svd(v./n); ok=numel(s)==2&&s(2)/s(1)>cfg.geometryRatio;
end
function [p,inliers,ok]=chooseCandidate(candidates,scores,ids,d,cfg,gate)
p=[NaN NaN]; inliers=[]; ok=false;
if ~isempty(gate), scores(vecnorm(candidates-gate(1:2),2,2)>gate(3))=inf; end
[best,k]=min(scores); if ~isfinite(best),return;end
q=candidates(k,:); inliers=ids(abs(d(ids)-ultraRange(q,cfg,ids))<=cfg.residualLimit);
if numel(inliers)<3,return;end
q=fitPosition(inliers,d,cfg);
if ~goodGeometry(q,inliers,cfg),return;end
if ~isempty(gate)&&norm(q-gate(1:2))>gate(3),return;end
if any(abs(d(inliers)-ultraRange(q,cfg,inliers))>cfg.residualLimit),return;end
p=q; ok=true;
end
