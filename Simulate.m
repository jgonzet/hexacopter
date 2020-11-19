load('initialization.mat')
GuidStrat=1; % 1: pulsos, 2: Waypoints, 3: vuelos reales,4: ref. nulas

SimTime = 10;
FailTime = 0.5;
Fs = 200;
Ts=1/Fs;
Time_Falla = SimTime-FailTime;
samplesgood=(SimTime-FailTime)*Fs+1;

mm0 = 2.2;   % valor medio de la masa
mmvar = 0.2; % varianza
rng('shuffle')


%% GUIDANCE TO RANDOM POINTS
TNewWP = 30; %estaba en 5
VarNewWP = 10;  %varianza para gaussiana
MaxNewWP= 3; % maximo para uniforme
VarNewWP_h = 3;


% Perturbance parameters, on, time between steps, variance
HighPerturbOn = 1;
TNewDist = 10;
VarNewDist = 0.2; % ESTO BARDEA MUCHO
MaxNewDist = 0.4;
DelayDist = 0.0; % Donde empieza el primer viento


%% Angle Commands Generator
MaxNewRef = 15;
TNewRef = 2+Ts;
Time_Ref_Off = 8; %Tiempo de apagado de las referencias
step_yaw = 7;
yaw_rise_time = 0.8;
yawref_amp = 30;

noise_power= 1 *0.00001;

% Cantidad de vuelos, importante para nombrar los csv:
init_sim = 1;
end_sim = 1000;

vuelos_chotos = 0;
%=======================================
for motorfailing=1:6

    folder = ['fail' num2str(motorfailing)];
    motFailNotKnow = ones(1,6);
    motFailNotKnow(motorfailing) = 0;

    f = waitbar(0,'Please wait...');
    mkdir(folder)

    i = init_sim;
    
    while i<=end_sim
        
        text=['Motor ' num2str(motorfailing) '  (' num2str(round(100*(i-1)/end_sim,2),3) '%)'];
        waitbar((i-init_sim)/(end_sim-init_sim),f,text);
        name=['run' num2str(i)];
        
        randint = round(rand*100000);
        Amp_yaw_ref = randi([-yawref_amp,+yawref_amp]);
        %generamos valores aleatorios para cada run:
        mm = mm0 +(-mmvar+2*mmvar*rand(1));         % masa del vehiculo
        sesgo_balance = 0.01*rand(3,1).*[1,1,0.1]';  % desbalance de masa

        % Fluctuations in The Force Luke
        delta = 0.01;                               % pequeñas diferencias entre performance
        fmin = 1-delta;                             % de los motores (~1%)
        fmax = 1+delta;
        var_ff = (fmin + (fmax-fmin).*rand(6,1))';

        % Semillas para generacion de datos aleatorios
        seedWP = round(rand(3,1)*100000);          % generacion de waypoints
        seed_refs = round(rand(3,1)*100000);        % generacion de waypoints
        seed_torque=round(rand(3,1)*100000);        % generacion de torques de viento
        seed_force_vib = round(rand(3,1)*100000);   % torques de vibracion
        seed_torque_vib = round(rand(3,1)*100000);  % fuerzas de vibracion
        seed_WP = round(rand(3,1)*10000);           
        sim hexacopterECEFFTC8

        angulos = Angles.signals.values;
        aux = sum(sum(abs(angulos(1:samplesgood,:))>30));

        if(aux==0)
            save_run(folder, name, Angles, AttCommands, AngleRates, ff, ZLMN, Ts)
            i=i+1;
        else 
            vuelos_chotos = vuelos_chotos+1;
        end
        
    end

    close(f)
    
end