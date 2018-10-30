%% DataFFT
% Author: Rich Symmonds
% Started: 19/09/2018
% Updated: 21/10/2018

%% Data capture and variable setup
% Open data file
fileid = fopen('278.csv');

% Read data from .csv file
readData = textscan(fileid,'%f %f %f %f %f %f %f','Delimiter',',');

% Extract sort data
time = readData{1,1}(:,1);
wind = readData{1,2}(:,1);
xAcc = readData{1,3}(:,1);
yAcc = readData{1,4}(:,1);
roll = readData{1,6}(:,1);
ptch = readData{1,7}(:,1);

N = length(xAcc);
w=window(@blackmanharris,N);
xAcc_w = (xAcc).*w;
yAcc_w = (yAcc).*w;
time = time - time(1);
figure;

%% Plot wind speed
subplot(2,1,1);
avgWind = mean(wind);
fSt = (avgWind -(19.7*0.00001821/0.1))*0.198/0.1;
wind = smoothdata(wind,'sgolay');

plot(time,wind)
xlabel('Time [s]')
ylabel('Wind Speed [m/s]');
title({'Wind speed as a function of time'});
annotation('textbox',[.9 .685 .1 .2],'String',"Mean",'EdgeColor','none')
annotation('textbox',[.9 .65 .1 .2],'String',avgWind,'EdgeColor','none')
annotation('textbox',[.9 .585 .1 .2],'String',"f_S",'EdgeColor','none')
annotation('textbox',[.9 .54 .1 .2],'String',fSt,'EdgeColor','none')

%% Plot FFT of x-axis Acceleration
subplot(2,1,2);
Fs = 100;            % Sampling frequency                    

X = fft(xAcc_w);

P2 = abs(X/N);
P1 = P2(1:N/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(N/2))/N;
plot(f,P1);

%% Plot FFT of y-axis Acceleration
hold on;

Y = fft(yAcc_w);

Q2 = abs(Y/N);
Q1 = Q2(1:N/2+1);
Q1(2:end-1) = 2*Q1(2:end-1);

plot(f,Q1,'r'); 

title('Single-Sided Amplitude Spectrum of AccX and AccY')
xlabel('f (Hz)')
ylabel('Magnitude')