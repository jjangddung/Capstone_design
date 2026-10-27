function [st,y]=ultraTick(st,t,command,cfg)
% command = [goalX goalY movementMode applyCounter coverageMode noiseMode]
% coverageMode: 1 assumed detection, 2 measured-only; movementMode: 0 static,1 move.
command=command(:)';goal=command(1:2); b=cfg.roi;
inside=goal(1)>=b(1)&&goal(1)<=b(2)&&goal(2)>=b(3)&&goal(2)<=b(4);
if command(4)~=st.command
    st.commands(end+1,:)=[t command];
    st.command=command(4);
    if command(3)==0&&inside
        st.p=goal; st.experiment=st.experiment+1; st.track=[];
        st.nextSensor=1; st.nextTime=t; st.raw(:)=NaN; st.times(:)=NaN; st.hw(:)=false;
        st.gtTime=[]; st.gt=[]; st.last.p(:)=NaN; st.last.success(:)=false;
        st.last.status(:)="RESET"; st.error(:)=NaN; st.gtRef(:)=NaN;
    end
end
if command(3)==1&&inside
    delta=goal-st.p; n=norm(delta);
    if n>0, st.p=st.p+delta*min(1,cfg.vSim*cfg.Ts/n); end
end
st.gtTime(end+1,1)=t; st.gt(end+1,:)=st.p;
fresh=false;
if t+cfg.Ts/100>=st.nextTime
    i=st.nextSensor; v=st.p-cfg.sensorXY(i,:); u=cfg.direction(i,:);
    axial=dot(v,u); lateral=dot(v,[-u(2) u(1)]);
    known=axial>=cfg.beamAxial(1)&&axial<=cfg.beamAxial(end);
    detected=axial>0; st.coverage(i)=1;
    if command(5)==2
        if ~known
            detected=false; st.coverage(i)=3; % uncalibrated, not physical nondetection
        else
            detected=abs(lateral)<=interp1(cfg.beamAxial,cfg.beamHalfWidth,axial);
            if ~detected,st.coverage(i)=2;end
        end
    end
    d=ultraRange(st.p,cfg,i); hw=true;
    if st.coverage(i)==3
        d=NaN; hw=false;
    elseif ~detected
        d=cfg.background(i);
    end
    if command(6)==2&&hw
        d=d+cfg.noiseStd*randn(st.rng);
        if cfg.quantum>0,d=round(d/cfg.quantum)*cfg.quantum;end
    elseif command(6)==3&&hw
        d=d+cfg.noiseStd*randn(st.rng);
        if i==2,d=d+.10;end
        if i==4,d=cfg.background(i);end
    end
    st.raw(i)=cfg.alpha(i)*d+cfg.beta(i); st.times(i)=t; st.hw(i)=hw;
    st.nextSensor=i+1; st.nextTime=st.nextTime+cfg.slot;
    if i==6
        st.scanId=st.scanId+1;
        [st.last,st.track]=ultraEstimate(st.raw,st.times,st.hw,st.track,cfg);
        st.gtRef=[NaN NaN];
        if isfinite(st.last.tRef)&&numel(st.gtTime)>1
            st.gtRef=interp1(st.gtTime,st.gt,st.last.tRef,'linear');
        end
        st.error=vecnorm(st.last.p-st.gtRef,2,2);
        h=struct('time',st.last.tRef,'gt',st.gtRef,'result',st.last,...
            'raw',st.raw,'sensorTimes',st.times,'hw',st.hw,'experiment',st.experiment);
        st.history(end+1)=h; fresh=true; st.nextSensor=1;
        % Retain enough GT history for next scan interpolation.
        keep=st.gtTime>=t-2; st.gtTime=st.gtTime(keep); st.gt=st.gt(keep,:);
    end
end
y=[st.p(:);st.gtRef(:);reshape(st.last.p',6,1);st.error;st.last.range;...
    double(st.last.valid);double(st.last.success);double(fresh);st.last.tRef;t;st.experiment];
end
