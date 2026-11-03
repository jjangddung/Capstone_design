function cfg = ultraConfig()
% All distances in metres and all times in seconds. Edit measured values here.
cfg.sensorInset = .025; % Measurement reference faces: 2.5 cm inside each wall
a=cfg.sensorInset;
cfg.sensorXY = [a .5;1-a .5;.5 a;.5 1-a]; % S1 left, S2 right, S3 bottom, S4 top
cfg.sensorCount = size(cfg.sensorXY,1);
cfg.direction = (.5-cfg.sensorXY)./vecnorm(.5-cfg.sensorXY,2,2);
cfg.radius = .009; % Measured cylinder radius: 9 mm
cfg.bodySize = [.4 .4]; % EXAMPLE only, not measured robot dimensions
cfg.clearance = .05;
cfg.roi = [.4 .6 .4 .6]; % conservative example central region
cfg.Ts = .01;
cfg.slot = .06; % EXAMPLE trigger-start spacing, not validated hardware timing
cfg.vSim = .08;
cfg.vMax = .15;
cfg.motionMargin = .025;
cfg.maxScanMotion = .06;
cfg.reinitializeAfter = 2;
cfg.residualLimit = .025;
cfg.roiRangeExtra = .05; % Extra 5 cm on BOTH sides of ROI distance bounds
cfg.geometryRatio = .02;
cfg.rangeLimits = [.02 5];
cfg.alpha = ones(cfg.sensorCount,1);
cfg.beta = zeros(cfg.sensorCount,1);
cfg.noiseStd = .003;
cfg.quantum = .01; % tentative SEN0388 resolution, confirm actual SKU
cfg.seed = 42;
cfg.beamFullAngleDeg = 38; % Assumed full horizontal angle, NOT measured at all ranges
cfg.defaultCoverageMode = 2; % 1=ideal, 2=fixed angle, 3=legacy measured profile
cfg.beamAxial = [.2 .3 .4]; % Retained only for optional legacy mode
cfg.beamHalfWidth = cfg.beamAxial.*tand([39 29 22]);
% Geometric background approximation. Replace with empty-room measurements.
cfg.background = zeros(cfg.sensorCount,1);
for i=1:cfg.sensorCount
    s=cfg.sensorXY(i,:); u=cfg.direction(i,:);
    distances=[];
    for k=1:2
        if u(k)>0, distances(end+1)=(1-s(k))/u(k); end %#ok<AGROW>
        if u(k)<0, distances(end+1)=-s(k)/u(k); end %#ok<AGROW>
    end
    cfg.background(i)=min(distances);
end
cfg.backgroundTolerance = .03*ones(cfg.sensorCount,1);
cfg.backgroundSource = 'ASSUMED geometry; not calibrated';
cfg.stopTime = 60;
cfg.showUI = true;
cfg.inputFormat = 'legacy9'; % Arduino connect button selects experiment24
cfg.inputMode = 'simulation'; % simulation | replay | serial
cfg.replayFile = '';
cfg.serialPort = 'COM3';
cfg.baudRate = 115200;
cfg.scanTimeout = 1.0;
cfg.staticGT = [NaN NaN]; % ONLY independently measured, stationary target
cfg.staticGTConfirmed = false;
cfg.liveWallCalibrated = false;
cfg.root = fileparts(mfilename('fullpath'));
end
