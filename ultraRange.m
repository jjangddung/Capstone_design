function d = ultraRange(p,cfg,ids)
if nargin<3, ids=1:6; end
d=vecnorm(cfg.sensorXY(ids,:)-p(:)',2,2)-cfg.radius;
end
