/**Let the compiler optimize the below precompiled array
 * This turns out to be the easiest way to do this because
 * the compiler will place code data in the flash memory from
 * the top page down.
 * We can let the compiler deal with placing the data instead of
 * doing it ourselves through trying to place the data near the end
 * pages at runtime.
 * 
 */
// #include "lut.h"


//Matlab code
// NS = 128; %min # of samples for one wave
// NTot = NS*6 %Length of buffer - and of resultant array
// t = linspace(0,2*pi,NTot);
// f1 = sin(6*t);
// f2 = sin(5*t);
// figure();
// plot(t,f1);
// hold on;
// plot(t,f2);
// hold off;

// f1 = round(2048*(f1+1));
// f2 = round(2048*(f2+1));

// arr = bitor(bitshift(f1,16,'uint32'),bitshift(f2,-2,"uint32"),"uint32");