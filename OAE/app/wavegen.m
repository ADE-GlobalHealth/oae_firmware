clear; clc;   % Clear The Previous Points
Ns     = 128; % Set The Number of Sample Points
RES    = 12;  % Set The DAC Resolution
OFFSET = 0;   % Set An Offset Value For The DAC Output

T = 0:((2*pi/(Ns-1))):(2*pi);
Y = sin(T);
Y = Y + 1;    
Y = Y*((2^RES-1)-2*OFFSET)/(2+OFFSET);
Y = round(Y);                  
plot(T, Y);
grid

fprintf('%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n', Y);


fileID = fopen('.txt','w');
fprintf(fileID,'%6s %12s\n','x','exp(x)');
fprintf(fileID,'%6.2f %12.8f\n',A);
fclose(fileID);