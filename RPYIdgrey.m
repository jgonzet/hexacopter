% np*nk=k*(kd*s^2 + kp*s + ki)
% np*nk=n=k*kd*s^2 + k*kp*s + k*ki;
% n= b3*s^3 + b2*s^2 + b1*s + b0
% d = s^4 + p*s^3 + k*kd*s^2 + k*kp*s + k*ki
% d = s^4 + a3s^3 + a2*s^2 + a1*s + ao
% A= [0 1 0 0;...
%       0 0 1 0;...
%       0 0 1 0;...
%       -a0 -a1 -a2 -a3];...
% B= [0 ; 0 ; 0 ; 1];
% C= [b0 b1 b2 b3];...
% D=0;
%
%
% T(s)=np*nk/d;

function sys=RPYIdgrey(kd,kp,ki,kdy,kpy,kiy),
% sys=PitchRollIdgrey

odefun = 'RPYsys';

p = 20;
k = 200;
ky = 1000;

%kp = 1;
%ki = 1;
%kd = 1;
parameters = {'kd',kd;'kp',kp;'ki',ki;...
                      'kdy',kdy;'kpy',kpy;'kiy',kiy;...
                      'k',k;'ky',ky;'p',p};
fcn_type = 'c';
sys = idgrey(odefun,parameters,fcn_type);

% Los parametros del PID son fijos.
sys.Structure.Parameters(1).Free = false;
sys.Structure.Parameters(2).Free = false;
sys.Structure.Parameters(3).Free = false;
sys.Structure.Parameters(4).Free = false;
sys.Structure.Parameters(5).Free = false;
sys.Structure.Parameters(6).Free = false;
% Maximo y minimo "k"
sys.Structure.Parameters(7).Minimum = 50;
sys.Structure.Parameters(7).Maximum = 1000;
% Maximo y minimo "ky"
sys.Structure.Parameters(7).Minimum = 50;
sys.Structure.Parameters(7).Maximum = 20000;
% Maximo y minimo "p"
sys.Structure.Parameters(9).Minimum = 1;
sys.Structure.Parameters(9).Maximum = 100;
