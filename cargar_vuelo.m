load('initialization.mat')
%GuidStrat=3; % 1 is pulse, 2 is PID, 3 is refs % 4 is zero ref

%Guardo los angulos en variables
load('testing/Outdoors_mov1_filt.mat')
%load testing/movement.mat
%motFailNotKnow=ones(1,6);   %no fail

f1 = timeseries(data.f1,[0:0.005:0.005*(length(data.index)-1)]);
f2 = timeseries(data.f2,[0:0.005:0.005*(length(data.index)-1)]);
f3 = timeseries(data.f3,[0:0.005:0.005*(length(data.index)-1)]);
f4 = timeseries(data.f4,[0:0.005:0.005*(length(data.index)-1)]);
f5 = timeseries(data.f5,[0:0.005:0.005*(length(data.index)-1)]);
f6 = timeseries(data.f6,[0:0.005:0.005*(length(data.index)-1)]);

rollref    = timeseries(data.rollref,[0:0.005:0.005*(length(data.index)-1)]);
pitchref   = timeseries(data.pitchref,[0:0.005:0.005*(length(data.index)-1)]);
yawref     = timeseries(data.yawref,[0:0.005:0.005*(length(data.index)-1)]);
roll       = timeseries(data.roll,[0:0.005:0.005*(length(data.index)-1)]);
pitch      = timeseries(data.pitch,[0:0.005:0.005*(length(data.index)-1)]);
yaw        = timeseries(data.yaw,[0:0.005:0.005*(length(data.index)-1)]);
SimTime = 0.005*(length(data.index)-1);
