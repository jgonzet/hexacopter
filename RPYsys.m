function [A,B,C,D] = RPYsys(kd,kp,ki,kdy,kpy,kiy,k,ky,p,Ts)
%[A,B,C,D] = PRYsys(kd,kp,ki,kdy,kpy,kiy,k,ky,p,Ts)
s=tf('s');
n=k*(kd*s^2 + kp*s + ki);
d = s^4 + p*s^3 + k*kd*s^2 + k*kp*s + k*ki;
nn=tfdata(n,'v');
dd=tfdata(d,'v');
[Arp,Brp,Crp,Drp] = tf2ss(nn,dd);
ny=ky*(kdy*s^2 + kpy*s + kiy);
dy= s^4 + p*s^3 + ky*kdy*s^2 + ky*kpy*s + ky*kiy;
nny=tfdata(ny,'v');
ddy=tfdata(dy,'v');
[Ay,By,Cy,Dy] = tf2ss(nny,ddy);
A=mdiag(Arp,Arp,Ay);
B=mdiag(Brp,Brp,By);
C=mdiag(Crp,Crp,Cy);
D=mdiag(Drp,Drp,Dy);
end 
