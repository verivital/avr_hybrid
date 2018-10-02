% Hysteresis controller for a buck converter

%clc  % clears command window
%clear all; % warning, all results and workspace cleared

addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\matlab\hardware\supportpackages\sharedarduino\target')
addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\matlab\hardware\supportpackages\sharedarduino\target\server')
addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\supportpackages\arduinobase\src')
addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\supportpackages\arduinobase\include')
addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\svd\src')
addpath('C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\svd\include')

% simulink custom paths
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\matlab\hardware\supportpackages\sharedarduino\target\server
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\matlab\hardware\supportpackages\sharedarduino\target
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\ioserver\ioserver\src
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\ioserver\ioserver\inc
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\supportpackages\arduinobase\src
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\supportpackages\arduinobase\include
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\svd\src
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\svd\include
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\ioserver\template\peripherals\inc
% C:\ProgramData\MATLAB\SupportPackages\R2018b\toolbox\target\shared\ioserver\template\peripherals\src

opt_fixed_point = 0;
opt_single = 1;

Vs_noise = 0;%0.1

i0 = 0.0;
v0 = 0.646;

if opt_fixed_point
    i0 = fi(i0,1,16,32);
    v0 = fi(v0,1,16,32);
end

if opt_single
    i0 = single(i0);
    v0 = single(v0);
end

%Define Buck paramerts
T = 1/50000;
Vs = 24;
Vref = 12;
Vtol = Vref/120;% Tolerance level for hysteresis band 
C = 2.2e-3;
L = 2.65e-3;
R = 10;% load resistance
rs = 200e-3;% switching loss
rL = 520e-3;%  inductor loss
Tmax = T*1000;% 

if opt_fixed_point
    T = fi(T);
    Vs = fi(Vs,1,64,32);
    Vref = fi(Vref,1,64,32);
    Vtol = fi(Vtol);
    C = fi(C);
    L = fi(L);
    R = fi(R);
    rs = fi(rs);
    rL = fi(rL);
    %Tmax = fi(Tmax);
end

if opt_single
    T = single(T);
    Vs = single(Vs);
    Vref = single(Vref);
    Vtol = single(Vtol);
    C = single(C);
    L = single(L);
    R = single(R);
    rs = single(rs);
    rL = single(rL);
    %Tmax = fi(Tmax);
end
    
% Define transition matrices

if opt_fixed_point
    fzero = fi(0);
else
    fzero = 0;
    
    if opt_single
        fzero = single(fzero);
    end
end

Ac_nom = [-1*(rs+rL)/L, -(1/L); (1/C), -(1/(R*C))];% switch closed
Bc_nom = [(1/L); fzero];

Ao_nom = [-rL/L, -(1/L); (1/C), -(1/(R*C))];% switch open
Bo_nom = [fzero; fzero];
        
Ad_nom = [fzero, fzero; fzero, -(1/(R*C))];%For DCM
Bd_nom = [fzero; fzero];%For DCM

if opt_fixed_point
    dutyT = numerictype('Signed',true,'WordLength',64,'FractionLength',32);
    D = dutyT.divide(Vref,Vs);
else
    D = Vref / Vs;
    
    if opt_single
        D = single(D);
    end
end

%D = Vref / Vs;% duty cycle; multiply with cr incase of open loop


sys(1).Ac = Ac_nom;
sys(1).Bc = Bc_nom;
sys(1).Ao = Ao_nom;
sys(1).Bo = Bo_nom;
sys(1).Ad = Ad_nom;%For DCM
sys(1).Bd = Bd_nom;%For DCM
    
% set parameters used in stateflow simulation
a00c = sys(1).Ac(1,1);
a01c = sys(1).Ac(1,2);
a10c = sys(1).Ac(2,1);
a11c = sys(1).Ac(2,2);
    
b0c = sys(1).Bc(1);
b1c = sys(1).Bc(2);
    
a00o = sys(1).Ao(1,1);
a01o = sys(1).Ao(1,2);
a10o = sys(1).Ao(2,1);
a11o = sys(1).Ao(2,2);
    
b0o = sys(1).Bo(1);
b1o = sys(1).Bo(2);
    
%DCM
    
a00d = sys(1).Ad(1,1);
a01d = sys(1).Ad(1,2);
a10d = sys(1).Ad(2,1);
a11d = sys(1).Ad(2,2);
    
b0d = sys(1).Bd(1);
b1d = sys(1).Bd(2);
    

