function file=makeDemoReplay()
% Synthetic stationary data for testing the Arduino-compatible CSV adapter.
c=ultraConfig();folder=fullfile(c.root,'data');if ~isfolder(folder),mkdir(folder);end
file=fullfile(folder,'synthetic_arduino.csv');fid=fopen(file,'w');
assert(fid>=0);cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
p=[.48 .52];d=ultraRange(p,c);
for scan=1:20
    for i=1:6
        t=round(((scan-1)*6+i-1)*c.slot*1e6);
        fprintf(fid,'M,1,1,%d,%d,%d,%d,%.9f,OK\n',scan,i,t,t+4000,d(i)*1000);
    end
end
end
