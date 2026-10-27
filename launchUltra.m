function launchUltra(cfg)
% Run this function from MATLAB to open the model and interactive controls.
root=fileparts(mfilename('fullpath'));addpath(root);
if nargin<1,cfg=ultraConfig();end
if ~strcmp(cfg.inputMode,'simulation')&&~cfg.liveWallCalibrated
    % Do not reject real measurements using an unmeasured synthetic background.
    cfg.background(:)=NaN;cfg.backgroundSource='DISABLED - calibrate empty-room data';
end
model=buildModel(cfg);
ultraView('open',[],0,[],cfg);
open_system(model);
try,set_param(model,'EnablePacing','on','PacingRate','1');catch,end
fprintf('Set X/Y, click Apply, then Run. Defaults are illustrative.\n');
end
