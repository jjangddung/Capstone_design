function cfg = ultraConfig()
% All distances in metres and all times in seconds. Edit measured values here.
cfg.sensorXY = [.05 .01;.95 .01;.05 .99;.95 .99;.01 .5;.99 .5];
cfg.direction = (.5-cfg.sensorXY)./vecnorm(.5-cfg.sensorXY,2,2);
cfg.radius = .005; % EXAMPLE only: replace with measured target radius
cfg.bodySize = [.4 .4]; % EXAMPLE only, not measured robot dimensions
cfg.clearance = .05;
cfg.roi = [.35 .65 .35 .65]; % conservative example central region
cfg.Ts = .01;
cfg.slot = .06; % EXAMPLE trigger-start spacing, not validated hardware timing
cfg.vSim = .08;
cfg.vMax = .15;
cfg.motionMargin = .025;
cfg.maxScanMotion = .06;
cfg.reinitializeAfter = 2;
cfg.residualLimit = .025;
cfg.geometryRatio = .02;
cfg.rangeLimits = [.02 5];
cfg.alpha = ones(6,1);
cfg.beta = zeros(6,1);
cfg.noiseStd = .003;
cfg.quantum = .01; % tentative SEN0388 resolution, confirm actual SKU
cfg.seed = 42;
cfg.beamAxial = [.2 .3 .4];
cfg.beamHalfWidth = cfg.beamAxial.*tand([39 29 22]);
% Geometric background approximation. Replace with empty-room measurements.
cfg.background = zeros(6,1);
for i=1:6
    s=cfg.sensorXY(i,:); u=cfg.direction(i,:);
    distances=[];
    for k=1:2
        if u(k)>0, distances(end+1)=(1-s(k))/u(k); end %#ok<AGROW>
        if u(k)<0, distances(end+1)=-s(k)/u(k); end %#ok<AGROW>
    end
    cfg.background(i)=min(distances);
end
cfg.backgroundTolerance = .03*ones(6,1);
cfg.backgroundSource = 'ASSUMED geometry; not calibrated';
cfg.stopTime = 60;
cfg.showUI = true;
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
