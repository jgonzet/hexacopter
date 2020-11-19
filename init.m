clear all;clc;

%load('../2018 - ICRIS/MatlabAle/EstimatedParameters.mat');
load('EstimatedParameters.mat');

%% Initial conditions

% DEPRECATED:
% rollinitialtilterror=0*pi/180;
% pitchinitialtilterror=0*pi/180;
% yawinitialtilterror=0*pi/180;
% END

% CONDICIONES QUE LO HACEN PELOTA c/ FALLA

rollinitialtilt=0*pi/180;
pitchinitialtilt=-0*pi/180;
yawinitialtilt= 0*pi/180;
rollinitialrate=0*pi/180;
pitchinitialrate=0*pi/180;
yawinitialrate=0*pi/180;

P1=kml2struct('P1.kml');
P2=P1;


h0=40; % 40 meters initial height. That's when the RC switches to "automatic".
ipos_g=[P1.Lat P1.Lon h0];

ivel_b=[0 0 0]';
ieuler = [rollinitialtilt pitchinitialtilt yawinitialtilt]';
irates_b = [rollinitialrate pitchinitialrate yawinitialrate]';

%gyrobias=.05*pi/180*[1 1 1];
%accelbias=.3*[1 1 1];

gyrobias = 0*pi/180*[1 1 1];
% 0.07 grados/seg
accelbias = 0*[1 1 1];

% PERTUBACION CONSTANTE PULSO:
% 0.1 Nm
% 4 N

[mag_field_vector, hor_intensity, declination, inclination, total_intensity, ...
    mag_field_sec_variation, sec_variation_horizontal, sec_variation_declination, ...
    sec_variation_inclination, sec_variation_total] = ...
    igrfmagm(h0,P1.Lat, P1.Lon, decyear('31-December-2017','dd-mmm-yyyy'));
%    igrf11magm(h0,P1.Lat, P1.Lon, decyear('31-December-2014','dd-mmm-yyyy'));

%% OTHER SIMULATION CONDITIONS
SuperAllocation=0; % USE JIG new allocation algorithm
SatSimul=1; %simulate with PWMC saturation
PWMSat_lowerlimit=0; % LIMITS in percent
PWMSat_upperlimit=100;
ActLowPass=1; %simulate with Actuator Low Pass Filter.
DisableMagModel=0; % Disable Magnetic Model Simulation
DisableYawControl=0; % Disable Yaw Control
rotorFail=0; % 0, es que no hay falla. 1 a 6 es falla.
UseMqFail=1; % NO TOCAR, ver mas abajo linea "MqFail= Mq;"
UseOmegaTrue=1; % USE true omega (simulated) to compute tilt estimation
                % and attitude control .
UsePhiThetaTrue=1; % USE true Phi and Theta (simulated) to compute tilt 
                   % estimation and attitude control .
UseZLMN=0; % NO ACTUATOR ALLOCATION. Beautiful life!

GPSerrorSimul=0; 

%% Disturbances:
DisturbanceNmbr= 2;
% 1 Constant
% 2 BandLimitedWhiteNoise
% 3 Pulse
% 4 Triangle
%SimWithConstDisturbances=1; % Or "0" for BandLimitedWhiteNoise
ConstDistForce=zeros(3,1);
ConstDistTorque=[0;0;0];
%ConstDistTorque=-1e-2*[sqrt(3)/2 0.5 1]/sqrt(2) % De Juan

dist_force_dir=[0 ;1 ;0]; % All ZERO for no dist.
dist_torque_dir=[0 ;1 ;0]; % All ZERO for no dist.
MDistGain=1;%.295; Moments ramp disturbance
FDistGain=0;%.295; Moments ramp disturbance

% PULSE DISTURBANCE Pulse Amplitude 

PulseAmpRoll=0*pi/180;
PulseAmpPitch=0*pi/180;
PulseAmpYaw=0;

% In seconds. PulseT*PulseW/100 seconds follow attitude second part of
% pulse cycle command zero.

PulseTRoll=40; 
PulseTPitch=40; 
PulseTYaw=40;

% Percentage duty cycle.
PulseWRoll=50; 
PulseWPitch=50; 
PulseWYaw=50 ;


%% SIMULATOR INIT

%mm=1.4; % mass 1.4 Kg
mm=3.2;

g=[0;0;9.8] ; % gravity vector in NED Frame
CG=[0;0;0]; % IMU LOCATION
% Nominal Inertia

%JJxx0=0.06;
%JJyy0=0.06;
%JJxx0=0.2;
%JJyy0=0.2;
%JJzz0=0.04;
%JJzz0=0.12;
%JJzz0=0.4;

% IDENTIFICADOS
JJxx0=EstimatedParameters(end)/EstimatedParameters(end-2);
JJyy0=EstimatedParameters(end)/EstimatedParameters(end-2);
JJzz0=EstimatedParameters(end)/EstimatedParameters(end-1);

%JJ0=mdiag(JJxx0,JJyy0,JJzz0);
JJ0=mdiag(JJxx0,JJyy0,JJzz0);
%JJ=JJ0/2;

JJ=JJ0;
%JJ(1,1)=JJ(1,1)*2;

%L=0.55/2; % (DJI 550 torque arm is 550mm/2)
L=0.55/2;
%L=0.77/2;

%Ktw0=0.0667*ones(6,1);
Ktw0=0.1104*ones(6,1);
%Ktw=0.0667*(1+.1*2*(rand(6,1)-.5)).*ones(6,1); % DISTURBED
Ktw=Ktw0.*ones(6,1);
Kmw = 0.0346*ones(6,1)/10;
%Kmw = Ktw0*0.1;
%Cdz0=17.03;
Cdz0=0;
%Cdz=17.03*(1+.1*2*(rand(6,1)-.5)).*ones(6,1); % DISTURBED
Cdz=Cdz0.*ones(6,1);

rollArm=L*sqrt(3)/2;
pitchArm = L;

%% STATE SPACE MATRICES OF THE LOW PASS ACTUATOR DYNAMICS

%Ts=1/80; % 50e-3; % sampling time
Ts=1/200; % 50e-3; % sampling time
%T1=80e-3; % Pole of first order Low Pass Transfer Function of PWM to Force and Moment actions.
T1=1/EstimatedParameters(end);

% Alp=-1/T1*eye(3);
% Blp=1/T1*[1;1;1];
% Clp=[1 1 1];
% Dlp=0;

FIClp=[0;0;-mm*g(3)];
MIClp=[0;0;0];

% The following matriz gives
%[T;Mx;My;Mz] = Mq*[T1;T2;T3;T4;T5;T6];

%% ZLMN matrix and pseudoinverse
%OJO QUE ALEJANDRO ACA TIENE EL BRAZO 1 APUNTANDO HACIA ADELANTE, Y EL EJ Z
%APUNTANDO PARA ARRIBA, POR ESO CAMBIA LA MATRIZ OBTENIDA CON EL GET_A

% theta=73*pi/180;
theta=90*pi/180;

kt=Kmw(1);
kf=Ktw(1);
d=L;

%========================================================
%inclinacion simple
AT = [ [  -kt*cos(theta), (kt*cos(theta))/2 + (3^(1/2)*d*kf*sin(theta))/2, (kt*cos(theta))/2 + (3^(1/2)*d*kf*sin(theta))/2,   -kt*cos(theta),   (kt*cos(theta))/2 - (3^(1/2)*d*kf*sin(theta))/2, (kt*cos(theta))/2 - (3^(1/2)*d*kf*sin(theta))/2];...
       [ d*kf*sin(theta), (d*kf*sin(theta))/2 - (3^(1/2)*kt*cos(theta))/2, (3^(1/2)*kt*cos(theta))/2 - (d*kf*sin(theta))/2, -d*kf*sin(theta), - (3^(1/2)*kt*cos(theta))/2 - (d*kf*sin(theta))/2, (3^(1/2)*kt*cos(theta))/2 + (d*kf*sin(theta))/2];...
       [  -kt*sin(theta),                                   kt*sin(theta),                                  -kt*sin(theta),    kt*sin(theta),                                    -kt*sin(theta),                                   kt*sin(theta)]];

F = [ [ -kf*cos(theta),        -(kf*cos(theta))/2,         (kf*cos(theta))/2,  kf*cos(theta),          (kf*cos(theta))/2,         -(kf*cos(theta))/2];...
    [              0, (3^(1/2)*kf*cos(theta))/2, (3^(1/2)*kf*cos(theta))/2,              0, -(3^(1/2)*kf*cos(theta))/2, -(3^(1/2)*kf*cos(theta))/2];...
    [ -kf*sin(theta),            -kf*sin(theta),            -kf*sin(theta), -kf*sin(theta),             -kf*sin(theta),             -kf*sin(theta)]];

R= [ cosd(30), -sind(30),0; sind(30), cosd(30),0; 0,0,1];
AT=R*AT;
F=R*F;

% AT(1,:)=-AT(1,:);
% AT(2,:)=-AT(2,:);
AT(3,:)=-AT(3,:);

SuperMq=[F;AT];
inclinacion='simple';
if strcmp(inclinacion,'simple')
    Mq=SuperMq(3:6,:);
elseif strcmp(inclinacion,'doble')
    Mq = get_A( 0 , 70*pi/180, -10*pi/180, kf, kt, d);
    Mq = [Mq(4,:);Mq(1:3,:)];
end

%=======================================================
MqFail= Mq;

if rotorFail ~= 0,
    MqFail(:,rotorFail)=zeros(4,1);
end

pinvMq=pinv(Mq);
pinvMqFail=pinv(MqFail);

if rotorFail ~= 0,
    pinvMqFail(rotorFail,:)=zeros(1,4);
end

Z0=-mm*g(3); % hovering thrust (nominal)
ZLMN0=[Z0;zeros(3,1)];

PWM0=mm*g(3)/(6*Ktw0(1))+Cdz0;

T0=-(PWM0-Cdz0).*Ktw0;
T0kg=T0/9.8;

%% SUPERALLOCATION ALGORITHM

% first calculate "w" vector.

index=[];
ww=zeros(6,1);
if rotorFail ~= 0,
    for ii=1:6,
        if ii~=rotorFail,
            index=[index ii];
        end
    end
    w = null(MqFail(:,index));
    for ii=1:6,
        if ii ~=rotorFail;
            if ii<=rotorFail,
                ww(ii)=w(ii);
            else
                ww(ii)=w(ii-1);
            end
        end
    end
end
w=ww;
w=w/norm(w);

%% CONTROLLER GAINS HAND TUNED
% ALTITUD
%Kdalt=8;
%Kpalt=20;
%Kialt=2;
alphaalt=1;
Kialt=0.5;
Kpalt=1;
Kdalt=8;

% PITCH AND ROLL
% ESTAS NO SE
% Kd=0.6;
% Kp=1.2;
% Ki=2.0;
% ESTAS ANDAN!
% Kd=1;
% Kp=2;
% Ki=3.0;
% Kd=2;
% Kp=3;
% Ki=2;
% ESTAS NO!
%Ki=0.03;
%Kp=2;
%Kd=1;
adhocGain=1;
Kd=EstimatedParameters(1)*adhocGain;
Kp=EstimatedParameters(2)*adhocGain;
Ki=EstimatedParameters(3)*adhocGain;
UseFailureGainsRollPitch=1;
KdF=0.5000;
KpF=1.4000;
KiF=1.0000;


%alpha=0.8;
%alpha=1;
beta=0.98;

%%  YAW
% Kdyaw=1;
% Kpyaw=4;
% Kiyaw=2;
% alphayaw=0;
% Kdyaw=3;
% Kpyaw=12;
% Kiyaw=6;
alphayaw=0;

Kdyaw=EstimatedParameters(4);
Kpyaw=EstimatedParameters(5);
Kiyaw=EstimatedParameters(6);
% Kdyaw=0.4;
% Kpyaw=1.4;
% Kiyaw=2;

%0.7350    0.2338    0.0186
% 0.058825734134240   0.000528173863993
% (0.2109 s^2 + 0.2073 s + 0.01083)/s

%% XY

Kdxy =0.0900;
Kpxy =0.0400;
Kixy = 0.0100;

%alphaxy=1;

%% ATTITUDE COMMAND SATURATION

MaxRollComm=30*pi/180;
MaxPitchComm=30*pi/180;
GuidStrat=1; % 1 para pulsos, 2 PIDs, 3 referencias por archivo,4 nulas
LookUpTablesStartDelay=0; % Delay between roll pitch and yaw References
LookUpTablesDelayBetweenChannels=0; % Delay between roll pitch and yaw References
LookUpTablesChannels=[0;1;0];  % [1;0;0] Roll, [0;1;0] Pitch, [0;0;1] Yaw
LookUpTablesInputScalings=1;
RPYEstSysss=ss(RPYEstSys);

Tm = 0.2;
