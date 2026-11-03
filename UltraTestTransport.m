classdef UltraTestTransport < handle
    % In-memory transport for tests; never opens a serial device.
    properties
        Buffer=''
        Sent=strings(0,1)
    end
    properties (Dependent)
        NumBytesAvailable
    end
    methods
        function n=get.NumBytesAvailable(obj),n=numel(obj.Buffer);end
        function b=read(obj,n,~),b=uint8(obj.Buffer(1:n));obj.Buffer=obj.Buffer(n+1:end);end
        function writeline(obj,s),obj.Sent(end+1)=string(s);end
        function inject(obj,s),obj.Buffer=[obj.Buffer char(s) newline];end
    end
end
