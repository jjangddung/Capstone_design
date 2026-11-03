function detected=ultraFixedCoverage(axial,lateral,cfg)
% Assumed horizontal cone; center of calibration target, radius not added again.
halfAngle=cfg.beamFullAngleDeg/2;
assert(isfinite(halfAngle)&&halfAngle>0&&halfAngle<90,'Full angle must be between 0 and 180 degrees.');
detected=isfinite(axial)&isfinite(lateral)&axial>0&...
    abs(lateral)<=axial*tand(halfAngle)+1e-12;
end
