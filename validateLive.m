root=fileparts(mfilename('fullpath'));addpath(root);
r=[runtests(fullfile(root,'testUltra.m')),runtests(fullfile(root,'testBeam38.m')),...
    runtests(fullfile(root,'testFourWalls.m')),runtests(fullfile(root,'testArduinoExperiment.m'))];
disp(r);assertSuccess(r);
% Original Simulink simulation remains usable.
c=ultraConfig();c.showUI=false;model=buildModel(c);sim(model,'StopTime','0.5');
h=evalin('base','ultraHistory');assert(all(h(end).result.success));
fprintf('SIMULATION_PASSED\n');
% Replay the new CSV through Simulink and the same external estimator.
folder=fullfile(root,'data');if ~isfolder(folder),mkdir(folder);end
file=fullfile(folder,'experiment24_example.csv');fid=fopen(file,'w');
fprintf(fid,'# SYNTHETIC TEST DATA, not hardware measurements\n');
fprintf(fid,'P1,1,1,50,50,47,47,47,47,OK,OK,OK,OK,0,63000,126000,189000,2900,2900,2900,2900,50,50,0\n');
fclose(fid);
c.inputMode='replay';c.inputFormat='experiment24';c.replayFile=file;
c.showUI=true;buildModel(c);ultraView('open',[],0,[],c);sim(model,'StopTime','0.5');
h=evalin('base','ultraHistory');assert(all(h(end).result.success));assert(max(h(end).result.p-[.5 .5],[],'all')<1e-8);
f=findall(groot,'Tag','UltraLocalizationViewer');drawnow;fr=getframe(f);
imwrite(fr.cdata,fullfile(root,'results','replay_preview.png'));ultraCloseView(f);close_system(model,0);
% Render disconnected serial layout; never open a physical port in this test.
c.inputMode='serial';ultraView('open',[],0,[],c);f=findall(groot,'Tag','UltraLocalizationViewer');
drawnow;fr=getframe(f);imwrite(fr.cdata,fullfile(root,'results','serial_preview.png'));ultraCloseView(f);
fprintf('EXPERIMENT_REPLAY_AND_UI_PASSED\n');
fprintf('AVAILABLE_SERIAL_PORTS:\n');disp(serialportlist('available'));
