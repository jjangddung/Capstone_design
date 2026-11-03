function [origin,span]=ultraSensorFrame(cfg)
% Arduino's opposing-sensor coordinate origin expressed in wall coordinates.
s=cfg.sensorXY;
origin=[s(1,1),s(3,2)];
span=[s(2,1)-s(1,1),s(4,2)-s(3,2)];
assert(all(isfinite([origin span]))&&all(span>0),'Invalid opposing sensor geometry.');
end
