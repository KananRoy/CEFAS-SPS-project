% Pnumatic Ram & Hot Roll Press control 
% Kanan Roy
% Run this before running the Simulink model


clear all

%% Linear actuator parameters 

La = 6.4e-3; % Windings inductance in Henry
Arm_resistance = 2.2;    % Motor resistance in Ohm
D = 8e-5; % Damping coefficient in Nm/rads-1
J = 5.3e-5; % Motor inertia in kgm2
Ke = 0.121; % Voltage constant in V/rads-1
Kt = 0.121; % Torque constant in Nm/A
Km = 1e7; % Motor Stiffness in N/m
Ks = 1.8e5; % Screw Stiffness in N/m
l = 2.4e-3; % in m/rev
M_load = 20; % Load mass in Kg 500
M_scr = 2; % Screw mass in Kg 
C_scr = 1.2e3; % Screw damping in N/ms-1

h = l/(2*pi);


%% Linear Actuator state space model

Ala = [(-Arm_resistance/La) (-Ke/La)   0                  0            0                  0           0;
     (Kt/J)                 (-D/J)     (-(h^2*Km)/J)      0            ((h*Km)/J)         0           0;
     0                      1          0                  0            0                  0           0;
     0                      0          (h*Km)/M_scr     (-C_scr/M_scr) -(Ks+Km)/M_scr   C_scr/M_scr   Ks/M_scr;
     0                      0          0                  1            0                  0           0;
     0                      0          0                  C_scr/M_load     Ks/M_load         -(C_scr/M_load) -(Ks/M_load);
     0                      0          0                  0            0                  1           0]
Bla = [1/La; 0; 0; 0; 0; 0; 0];
Cla = [0 0 0 0 0 0 1];
Dla = zeros(size(Cla,1),size(Bla,2));

sys = ss(Ala,Bla,Cla,0);
G = tf(sys)
num = [4.088e10 6.132e12];
den = [1 1005 5.398e6 2.117e9 3.803e11 2.958e13 1.966e15 67.4];
[z,p,k] = tf2zp(num,den)

%% Controbality and Observability

[v_Ala, d_Ala] = eig(Ala);
Controlability_check = inv(v_Ala)*Bla

Observability_check =  Cla*v_Ala

%% Minimum Realization
sysr = minreal(sys);
sys_tf = minreal(G);

load('New_plant.mat')

%% LQR
lqr_Q = zeros(7);
lqr_Q(1,1) = 100;

lqr_R = 10;
lqr_Lr = lqr(Plant.A,Plant.B,lqr_Q,lqr_R); 
C_new = [1 0 0 0 0 0 0];

%%  Augment system with disturbances and noise
Vd = .01*eye(7);  % disturbance covariance
Vn = 0.001;       % noise covariance

% BF = [Bla Vd 0*Bla];  % augment inputs to include disturbance and noise

% sysC = ss(Ala,BF,Cla,[0 0 0 0 0 0 0 0 Vn]);  % build big state space system... with single output

% sysFullOutput = ss(Ala,BF,eye(7),zeros(7,size(BF,2)));  % system with full state output, disturbance, no noise

%%  Build Kalman filter
[Kf,P,E] = lqe(Plant.A,Vd,C_new,Vd,Vn);  % design Kalman filter
% Kf = (lqr(Ala',Cla',Vd,Vn))';   % alternatively, possible to design using "LQR" code

sysKF = ss(Plant.A-Kf*Plant.C,[Plant.B Kf],eye(7),0*[Plant.B Kf]);  % Kalman f

A_kf = Plant.A  - Plant.B*lqr_Lr - Kf*C_new;


%% Static model parameters

he = 0.05; %thickness at entry in m
%hf = 0.5; %thickness at exit in mm
R = 0.1778; %Roller outer radius in m
% phi = 50*2*pi/360; % angle in radians
mu = 0.3; % constant
% L = 0.010; % workplace length in m
% have = (he + hf)/2; % average height
wo = 0.127;% initial width in m
wf = 0.180;% we need to compute this with the densinfication factor
% the initial height, and the initial width and final height3000; % final width in m
w = (wo + wf)/2; % average width
rpm = 1; 
omega = 2*pi*rpm;
k = 500e6; % Strengthning coefficient
n = 0.25; % Work hardning exponent





%% Equations
% phi = (acosd(1-(he-hf)/(2*R))*2*pi)/360;
% L = R*phi
% eps = ln(he/hf);
% H = 2*sqrt(R/hf)* atand(sqrt(R/hf)*phi);
% have = hf + 2*R*(1-cos(phi));
% Yf = k * (eps/(n+1));
