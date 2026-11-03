function ultraConnectArduino(cfg)
% Choose a port without assuming that the first USB device is the Arduino.
try
    ports=serialportlist('available');
    if isempty(ports)
        errordlg('No available serial port. Connect Arduino by USB and close Serial Monitor / Plotter.','Arduino connection');return;
    end
    [idx,ok]=listdlg('PromptString','Select the Arduino COM port (115200 baud)',...
        'ListString',cellstr(ports),'SelectionMode','single','ListSize',[340 150]);
    if ~ok,return;end
    cfg.inputMode='serial';cfg.inputFormat='experiment24';cfg.serialPort=char(ports(idx));cfg.baudRate=115200;
    cfg.staticGTConfirmed=false;cfg.staticGT=[NaN NaN];
    if bdIsLoaded('ultrasonic_localization')&&~strcmp(get_param('ultrasonic_localization','SimulationStatus'),'stopped')
        set_param('ultrasonic_localization','SimulationCommand','stop');
    end
    launchUltra(cfg);
catch err,errordlg(err.message,'Arduino connection');end
end
