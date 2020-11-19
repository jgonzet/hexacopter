function Ato = get_A( flagm , gamma, delta, kf, kt, d) %angles in radians
%BUILD_A Summary of this function goes here
%   Detailed explanation goes here
alfa=pi/6; 
alfa=0;
rot1=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

alfa=-pi/6;
alfa=-1*pi/3;
rot2=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

alfa=-pi/6-1*pi/3;
alfa=-2*pi/3;
rot3=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

alfa=-pi/6-2*pi/3;
alfa=-3*pi/3;
rot4=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

alfa=-pi/6-3*pi/3;
alfa=-4*pi/3;
rot5=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

alfa=-pi/6-4*pi/3;
alfa=-5*pi/3;
rot6=[cos(alfa) -sin(alfa) 0; sin(alfa) cos(alfa) 0 ; 0 0 1];

rot1(abs(rot1)<1e-3)=0;
rot2(abs(rot2)<1e-3)=0;
rot3(abs(rot3)<1e-3)=0;
rot4(abs(rot4)<1e-3)=0;
rot5(abs(rot5)<1e-3)=0;
rot6(abs(rot6)<1e-3)=0;

e_CCW=[-cos(gamma), -sin(gamma)*sin(delta), -sin(gamma)*cos(delta)]';
e_CW=[-cos(gamma), +sin(gamma)*sin(delta), -sin(gamma)*cos(delta)]';

D=[d 0 0]';
H_CCW=kf*cross(D,e_CCW);
H_CW=kf*cross(D,e_CW);

v_CCW=H_CCW-kt*e_CCW;
v_CW=H_CW+kt*e_CW;

At=[rot1*v_CW, rot2*v_CCW, rot3*v_CW, rot4*v_CCW, rot5*v_CW, rot6*v_CCW];

%________________________________________________
unos=-ones(1,6)*kf*sin(gamma)*cos(delta);
if flagm>0,
    Iflg = eye(6); Iflg(flagm,flagm)=0;
    unos(flagm)=0;
    At = At*Iflg;
end
Ato=[At;unos];

end

