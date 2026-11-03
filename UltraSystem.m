classdef UltraSystem < matlab.System
    % Host-side interpreted Simulink adapter. No Arduino code generation.
    properties (Nontunable)
        Config = ultraConfig()
    end
    properties (Access=private)
        State
        Input = []
    end
    methods (Access=protected)
        function setupImpl(obj)
            obj.State=ultraInitialize(obj.Config);
            if ~strcmp(obj.Config.inputMode,'simulation')
                if isfield(obj.Config,'inputFormat')&&strcmp(obj.Config.inputFormat,'experiment24')
                    obj.Input=UltraExperimentInput(obj.Config);
                else,obj.Input=UltraInput(obj.Config);end
                obj.State.p=[NaN NaN];obj.State.coverage(:)=0;
            end
        end
        function y=stepImpl(obj,t,command)
            if strcmp(obj.Config.inputMode,'simulation')
                [obj.State,y]=ultraTick(obj.State,t,command,obj.Config);
            else
                [obj.State,y]=ultraExternalTick(obj.State,t,obj.Input.poll(t),obj.Config);
            end
            if obj.Config.showUI
                ultraView('update',obj.State,t,command,obj.Config);
            end
        end
        function resetImpl(obj)
            obj.State=ultraInitialize(obj.Config);
            if ~strcmp(obj.Config.inputMode,'simulation')
                obj.State.p=[NaN NaN];obj.State.coverage(:)=0;
            end
        end
        function releaseImpl(obj)
            if ~isempty(obj.Input),delete(obj.Input);obj.Input=[];end
            externalRecords=obj.State.externalRecords; %#ok<NASGU>
            history=obj.State.history; cfg=obj.Config; commands=obj.State.commands; %#ok<NASGU>
            folder=fullfile(cfg.root,'results'); if ~isfolder(folder),mkdir(folder);end
            tag=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
            save(fullfile(folder,[cfg.inputMode '_' tag '.mat']),'history','cfg','commands','externalRecords');
            ultraSummary(history,fullfile(folder,[cfg.inputMode '_' tag '_summary.csv']));
            assignin('base','ultraHistory',history);
        end
        function sz=getOutputSizeImpl(obj),sz=[20+2*obj.Config.sensorCount 1];end
        function dt=getOutputDataTypeImpl(~),dt='double';end
        function b=isOutputComplexImpl(~),b=false;end
        function b=isOutputFixedSizeImpl(~),b=true;end
        function b=isInputSizeMutableImpl(~,~),b=false;end
    end
end
