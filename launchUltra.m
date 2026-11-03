function launchUltra(cfg)
% Run this function from MATLAB to open the model and interactive controls.
root=fileparts(mfilename('fullpath'));addpath(root);
if nargin<1,cfg=ultraConfig();end
if ~strcmp(cfg.inputMode,'simulation')&&~cfg.liveWallCalibrated
    % Do not reject real measurements using an unmeasured synthetic background.
    cfg.background(:)=NaN;cfg.backgroundSource='DISABLED - calibrate empty-room data';
end
if strcmp(cfg.inputMode,'serial')
    cfg.inputFormat='experiment24';
    ultraView('open',[],0,[],cfg);
    f=findall(groot,'Type','figure','Tag','UltraLocalizationViewer');
    try
        session=UltraLiveSession(cfg,f);setappdata(f,'UltraLiveSession',session);
    catch err
        h=f.UserData;h.connection.String=['Connection failed: ' err.message];
        h.run.Enable='off';
        warning('Ultra:Connect','%s',err.message);
    end
    return
end
model=buildModel(cfg);
ultraView('open',[],0,[],cfg);
open_system(model);
try,set_param(model,'EnablePacing','on','PacingRate','1');catch,end
fprintf('Set X/Y, click Apply, then Run. Defaults are illustrative.\n');
end
