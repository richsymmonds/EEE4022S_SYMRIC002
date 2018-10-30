%% Iteration_Models
% Author: Rich Symmonds
% Started: 19/09/2018
% Modified: 21/10/2018

%% Plot Reynolds numbers
rho = 1.1974;
d = 0.1;
mu = 0.000018121;
syms speed;
Re = (rho.*speed.*d)./mu;

f1 = figure(1);
cla; hold on; grid on;

fplot(Re,[0 20]);

xlabel('Wind Speed [m/s]');
ylabel('Reynolds Number');
title('Reynolds number as a function of wind speed');

%% Plot Strouhal frequencies
St = 0.198.*(1-(19.7./Re));
Fs = speed.*St./d;

f2 = figure(2);
cla; hold on; grid on;

fplot(Fs,[0 20]);

xlabel('Wind Speed [m/s]');
ylabel('Strouhal Frequency [Hz]');
title('Strouhal frequency as a function of wind speed');

%% Plot wind profiles
z1 = 4.4;
uz1 = 1:20;
syms z2;
uz2 = uz1.*((log(z2./0.5))./(log(4.4./0.5)));
f3 = figure(3);
cla; hold on; grid on;

for i=1:20
    fplot(uz2,[0.5 4.4]);
end

xlabel('Height [m]');
ylabel('Wind Speed [m/s]');
title('Wind speed profiles as a function of height');
axis tight

%% Plot non-uniform drag force profiles 
syms z;
u1 = [1,5,10,20];
Cd_const = 1;

f4 = figure(4);
cla; hold on; grid on;

for j=1:4
    u2 = u1(1,j).*((log(z./0.5))./(log(4.4./0.5)));
    Fd1 = 0.5.*rho.*d.*(u2.^2).*Cd_const;
    fplot(Fd1,[0.5 4.4]);
end

xlabel('Height [m]');
ylabel('Drag Profile [N/m]');
title('Drag force profile as a function of height');
axis tight
legend('1 m/s','5 m/s','10 m/s','20 m/s');

%% Plot uniform drag force profiles (run as section after initial run to store variables - won't plot on afull run for some reason)
u1 = [1,5,10,20];

Cd_var = [1,1.15,0.9,0.5];

f5 = figure(5);
cla; hold on; grid on;

for k=1:4
    Fd2 = 0.5.*rho.*u1(1,k).*Cd_var(1,k);
    fplot(Fd2,[0.5 4.4]);
end

ylim([-1 9]);
xlabel('Wind Speed [m/s]');
ylabel('Drag Profile [N/m]');
title('Drag force profile as a function of wind speed');
legend('1 m/s','5 m/s','10 m/s','20 m/s');

%% Plot lift force profiles 
syms t;
u1 = [1,5,10,20];
wSt = [1.974,9.894,19.79,39.594];
Cl = 1;

cla; hold on;

f6 = figure(6);
Fl1 = 0.5.*rho.*d.*(u1(1)^2).*Cl.*sin(wSt(1).*t);
fplot(Fl1);
ylim([-0.1 0.1]);
xlabel('Time [s]');
ylabel('Lift Profile [N/m]');
title('Lift force profile for a 1 m/s wind as a function of time');

f7 = figure(7);
Fl2 = 0.5.*rho.*d.*(u1(2).^2).*Cl.*sin(wSt(2).*t);
fplot(Fl2);
xlim([-2 2]);
xlabel('Time [s]');
ylabel('Lift Profile [N/m]');
title('Lift force profile for a 5 m/s wind as a function of time');

f8 = figure(8);
Fl3 = 0.5.*rho.*d.*(u1(3).^2).*Cl.*sin(wSt(3).*t);
fplot(Fl3);
xlim([-1 1]);
xlabel('Time [s]');
ylabel('Lift Profile [N/m]');
title('Lift force profile for a 10 m/s wind as a function of time');

f9 = figure(9);
Fl4 = 0.5.*rho.*d.*(u1(4).^2).*Cl.*sin(wSt(4).*t);
fplot(Fl4);
xlim([-1 1]);
xlabel('Time [s]');
ylabel('Lift Profile [N/m]');
title('Lift force profile for a 20 m/s wind as a function of time');
