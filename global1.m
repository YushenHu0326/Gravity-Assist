clear global;
close all;

G = 6.6743e-11; % Gravitational constant (m^3/(kg*s^2))
M = 1.989e30; % Mass of sun (kg)
earth_year = 365.25*24*60*60; % duration of year (s)

%Calculate res = distance of earth from sun
%so that period or circular orbit
%is exactly earth_year:
res = 0.5*(sqrt(2*G*M)*earth_year/pi)^(2/3); % (m)

%Set duration of simulation and number of steps.
tmax = earth_year; % total time of simulation (s)
clockmax = 1000; % number of time steps
%Then duration of time step is determined:
dt = tmax/clockmax; % duration of time step (s)

F = moviein(clockmax);

rate = rateControl(100);
reset(rate);

% initial conditions for planet 1:
sr1 = 1; % scale factor for initial speed of planet 1
x1 = res;
y1 = 0;
u1 = 0;
v1 = sr1*sqrt(G*M/res);

% initial conditions for planet 2:
sr2 = 1; % scale factor for initial speed of planet 2 (change as needed)
res2 = 1.5*res; % distance of planet 2 from sun (change as needed)
x2 = res2;
y2 = 0;
u2 = 0;
v2 = sr2*sqrt(G*M/res2);

% arrays to store trajectory:
tsave = zeros(1,clockmax);
xsave1 = zeros(1,clockmax);
ysave1 = zeros(1,clockmax);
xsave2 = zeros(1,clockmax);
ysave2 = zeros(1,clockmax);

a=1.2*max(res, res2);

for clock=1:clockmax
    t = clock*dt;
    r1 = sqrt(x1^2+y1^2);
    u1 = u1 - dt*G*M*x1/r1^3;
    v1 = v1 - dt*G*M*y1/r1^3;
    x1 = x1 + dt*u1;
    y1 = y1 + dt*v1;

    r2 = sqrt(x2^2+y2^2);
    u2 = u2 - dt*G*M*x2/r2^3;
    v2 = v2 - dt*G*M*y2/r2^3;
    x2 = x2 + dt*u2;
    y2 = y2 + dt*v2;
    
    tsave(clock) = t;
    xsave1(clock) = x1;
    ysave1(clock) = y1;
    xsave2(clock) = x2;
    ysave2(clock) = y2;

    plot(0,0,'r*','linewidth',2); % Sun
    hold on;
    plot(x1,y1,'bo','linewidth',2); % Planet 1
    plot(xsave1(1:clock),ysave1(1:clock),'b','linewidth',2); % Trail of Planet 1
    plot(x2,y2,'go','linewidth',2); % Planet 2
    plot(xsave2(1:clock),ysave2(1:clock),'g','linewidth',2); % Trail of Planet 2
    hold off;
    axis equal;
    axis([-a,a,-a,a]);
    axis manual;

    waitfor(rate);
    F(clock) = getframe;
end
movie(F, 1);

v = VideoWriter('two_planets.avi');
v.open();
v.writeVideo(F);
v.close();
