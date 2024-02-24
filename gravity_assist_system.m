function gravity_assist_system()

clear global;
close all;

% constant

SLICE = 300;

% in Nm^2/kg^2
G = 6.6743e-11;

% in days
TIME_STEP_TOTAL = 300;

DELTAT = 60*60*24/SLICE;

DAYS_PER_TIME_STEP = 20*SLICE;

TACC = 390*SLICE;
%TACC = 1;

%A = 0.10187708*SLICE;
A = 0.101775*SLICE;
%A = 0.13889*SLICE;

% helper functions

function proj = vec2vecProj(a,b)
    p = dot(a,b)/(norm(b)*norm(b));
    proj = p*b;
end

function a = sideBoost(a0,v,d)
    s = cross(v/norm(v),[0,0,1])*d;
    a = a0 + s;
end

function gravity = calculateGravity(p1,p2,m,M)
    % delta in position (million km)
    d = p2-p1;
    % calculate gravity in kgm/s^2
    g = G*m*M/(norm(d)*norm(d));
    gravity = g*d/norm(d);
end

function a = updateAcceleration(m,f)
    % calculate acceleration in m/s^2
    a = f/m;
end

function v = updateVelocity(v0,a)
    % calculate velocity in m/s
    v = v0+a*DELTAT;
end

function p = updatePosition(p0,v)
    % calculate position after a day
    p = p0+v*DELTAT;
end

function w = updateWorkSlice(f,d)
    w = norm(vec2vecProj(f,d))*norm(d)*sign(dot(f,d));
end

% properties

% mass in kg
spacecraftM = 1000;
% position in m
spacecraftPos = [-0.5*(sqrt(2*G*1.989e+30)*365.25*24*60*60/pi)^(2/3),0,0];
% velocity in m/s
spacecraftVel = [0,-sqrt((G*1.989e+30)/(0.5*(sqrt(2*G*1.989e+30)*365.25*24*60*60/pi)^(2/3))),0];
% acceleration in m/s^2
spacecraftAcc = [0,0,0];

% same for the planet & star
starM = 1.989e+30;
starPos = [0,0,0];
starVel = [0,0,0];
starAcc = [0,0,0];

planet1Res = 0.5*(sqrt(2*G*starM)*365.25*24*60*60/pi)^(2/3); 
planet1M = 5.9722e+24;
planet1Pos = [-planet1Res,0,0];
planet1Vel = [0,-sqrt(G*starM/planet1Res),0];
planet1Acc = [0,0,0];

for i = 1:53*SLICE
    gM1 = calculateGravity(planet1Pos,starPos,planet1M,starM);
    planet1Acc = updateAcceleration(planet1M,gM1);
    planet1Vel = updateVelocity(planet1Vel,planet1Acc);
    planet1Pos = updatePosition(planet1Pos,planet1Vel);
    spacecraftPos = planet1Pos;
end

planet2Res = 0.5*(sqrt(2*G*starM)*4331*24*60*60/pi)^(2/3); 
planet2M = 1898e+24;
planet2Pos = [-planet2Res,0,0];
planet2Vel = [0,-sqrt(G*starM/planet2Res),0];
planet2Acc = [0,0,0];

for i = 1:1732*SLICE
    gM2 = calculateGravity(planet2Pos,starPos,planet2M,starM);
    planet2Acc = updateAcceleration(planet2M,gM2);
    planet2Vel = updateVelocity(planet2Vel,planet2Acc);
    planet2Pos = updatePosition(planet2Pos,planet2Vel);
end

planet3Res = 0.5*(sqrt(2*G*starM)*10747*24*60*60/pi)^(2/3); 
planet3M = 568e+24;
planet3Pos = [0,-planet3Res,0];
planet3Vel = [sqrt(G*starM/planet3Res),0,0];
planet3Acc = [0,0,0];

for i = 1:6757*SLICE
    gM3 = calculateGravity(planet3Pos,starPos,planet3M,starM);
    planet3Acc = updateAcceleration(planet3M,gM3);
    planet3Vel = updateVelocity(planet3Vel,planet3Acc);
    planet3Pos = updatePosition(planet3Pos,planet3Vel);
end

planet4Res = 0.5*(sqrt(2*G*starM)*30589*24*60*60/pi)^(2/3); 
planet4M = 86.8e+24;
planet4Pos = [planet4Res,0,0];
planet4Vel = [0,sqrt(G*starM/planet4Res),0];
planet4Acc = [0,0,0];

for i = 1:12235*SLICE
    gM4 = calculateGravity(planet4Pos,starPos,planet4M,starM);
    planet4Acc = updateAcceleration(planet4M,gM4);
    planet4Vel = updateVelocity(planet4Vel,planet4Acc);
    planet4Pos = updatePosition(planet4Pos,planet4Vel);
end

% animation

curve1 = animatedline('LineWidth',1,'Color','red');
curve2 = animatedline('LineWidth',1,'Color','cyan');
curve3 = animatedline('LineWidth',1,'Color','yellow');
curve4 = animatedline('LineWidth',1,'Color','green');
curve5 = animatedline('LineWidth',1,'Color','magenta');
set(gca,'XLim',[-5e+12,5e+12],'YLim',[-5e+12,5e+12],'ZLim',[-5e+12,5e+12]);
set(gca,"Color",[0.2,0.2,0.2]);
set(gca,"XColor",[1,1,1]);
set(gca,"YColor",[1,1,1]);

hold on;
grid on;

min_dist = 1e+10;
V = zeros(1,TIME_STEP_TOTAL);
WStar = 0;
wStar = zeros(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP);
W2 = 0;
w2 = zeros(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP);
W3 = 0;
w3 = zeros(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP);
W4 = 0;
w4 = zeros(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP);

for i=1:TIME_STEP_TOTAL
    addpoints(curve1,planet1Pos(1),planet1Pos(2),planet1Pos(3));
    addpoints(curve2,planet2Pos(1),planet2Pos(2),planet2Pos(3));
    addpoints(curve3,planet3Pos(1),planet3Pos(2),planet3Pos(3));
    addpoints(curve4,planet4Pos(1),planet4Pos(2),planet4Pos(3));
    addpoints(curve5,spacecraftPos(1),spacecraftPos(2),spacecraftPos(3));
    
    head1 = scatter3(planet1Pos(1),planet1Pos(2),planet1Pos(3),5,'red');
    head2 = scatter3(planet2Pos(1),planet2Pos(2),planet2Pos(3),10,'cyan');
    head3 = scatter3(planet3Pos(1),planet3Pos(2),planet3Pos(3),50,'yellow');
    head4 = scatter3(planet4Pos(1),planet4Pos(2),planet4Pos(3),100,'green');
    head5 = scatter3(spacecraftPos(1),spacecraftPos(2),spacecraftPos(3),2,'magenta');
    head6 = scatter3(0,0,0,20,[1,1,1]);

    legend([head6,head1,head2,head3,head4,head5],"Sun","Earth","Jupiter","Saturn","Uranus","Spacecraft")

    drawnow
    pause(0.01);
    
    if i ~= TIME_STEP_TOTAL
        delete(head1);
        delete(head2);
        delete(head3);
        delete(head4);
        delete(head5);
    end

    V(1,i) = norm(spacecraftVel);
    
    for j = 1:DAYS_PER_TIME_STEP
        planet1Pos = updatePosition(planet1Pos,planet1Vel);
        planet2Pos = updatePosition(planet2Pos,planet2Vel);
        planet3Pos = updatePosition(planet3Pos,planet3Vel);
        planet4Pos = updatePosition(planet4Pos,planet4Vel);

        gM1 = calculateGravity(planet1Pos,starPos,planet1M,starM);
        gM2 = calculateGravity(planet2Pos,starPos,planet2M,starM);
        gM3 = calculateGravity(planet3Pos,starPos,planet3M,starM);
        gM4 = calculateGravity(planet4Pos,starPos,planet4M,starM);

        planet1Acc = updateAcceleration(planet1M,gM1);
        planet2Acc = updateAcceleration(planet2M,gM2);
        planet3Acc = updateAcceleration(planet3M,gM3);
        planet4Acc = updateAcceleration(planet4M,gM4);

        planet1Vel = updateVelocity(planet1Vel,planet1Acc);
        planet2Vel = updateVelocity(planet2Vel,planet2Acc);
        planet3Vel = updateVelocity(planet3Vel,planet3Acc);
        planet4Vel = updateVelocity(planet4Vel,planet4Acc);
        
        if mod((i-1)*DAYS_PER_TIME_STEP+j,200) == 0
            %V(1,(i-1)*DAYS_PER_TIME_STEP+j/20000+1) = norm(spacecraftVel);
            %disp(norm(spacecraftVel))
            %disp(norm(spacecraftVel));
            %disp(norm(spacecraftPos-planet2Pos))
            %disp((i-1)*DAYS_PER_TIME_STEP+j)
            %disp(norm(calculateGravity(spacecraftPos,planet2Pos,spacecraftM,planet2M)))
        end
        
        if norm(spacecraftPos-planet2Pos) < min_dist
            min_dist = norm(spacecraftPos-planet2Pos);
            disp(min_dist)
        end

        if (i-1)*DAYS_PER_TIME_STEP+j < TACC
            spacecraftPos = planet1Pos;
            spacecraftVel = planet1Vel;
        elseif (i-1)*DAYS_PER_TIME_STEP+j == TACC
            spacecraftAcc = spacecraftVel/norm(spacecraftVel)*A;
            spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
        else
            previousSpacecraftPos = spacecraftPos;
            spacecraftPos = updatePosition(spacecraftPos,spacecraftVel);
            gm = calculateGravity(spacecraftPos,starPos,spacecraftM,starM);
            gm = gm + calculateGravity(spacecraftPos,planet2Pos,spacecraftM,planet2M);
            gm = gm + calculateGravity(spacecraftPos,planet3Pos,spacecraftM,planet3M);
            gm = gm + calculateGravity(spacecraftPos,planet4Pos,spacecraftM,planet4M);
            spacecraftAcc = updateAcceleration(spacecraftM,gm);
            spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
            WStar = updateWorkSlice(calculateGravity(spacecraftPos,starPos,spacecraftM,starM),spacecraftPos-previousSpacecraftPos);
            W2 = updateWorkSlice(calculateGravity(spacecraftPos,planet2Pos,spacecraftM,planet2M),spacecraftPos-previousSpacecraftPos);
            W3 = updateWorkSlice(calculateGravity(spacecraftPos,planet3Pos,spacecraftM,planet3M),spacecraftPos-previousSpacecraftPos);
            W4 = updateWorkSlice(calculateGravity(spacecraftPos,planet4Pos,spacecraftM,planet4M),spacecraftPos-previousSpacecraftPos);
            wStar(1,(i-1)*DAYS_PER_TIME_STEP+j) = WStar;
            w2(1,(i-1)*DAYS_PER_TIME_STEP+j) = W2;
            w3(1,(i-1)*DAYS_PER_TIME_STEP+j) = W3;
            w4(1,(i-1)*DAYS_PER_TIME_STEP+j) = W4;
        end
    end
end

disp("final velocity")
disp(norm(spacecraftVel))

disp("escape velocity")
disp(sqrt(2*G*starM/norm(spacecraftPos-starPos)))

X = linspace(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP/SLICE,TIME_STEP_TOTAL);
figure
plot(X,V)
grid on
title("Velocity of the spacecraft over time")
xlabel("time - days")
ylabel("velocity - m/s^2")

X = linspace(1,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP/SLICE,TIME_STEP_TOTAL*DAYS_PER_TIME_STEP);
figure
plot(X,wStar)
hold on
plot(X,w2)
hold on
plot(X,w3)
hold on
plot(X,w4)
hold off
grid on
title("Works from planets to the spacecraft over time")
xlabel("time - days")
ylabel("work - N*m")
legend("Sun","Jupiter","Saturn","Uranus")

figure
plot(w2(1285*SLICE:1287*SLICE))
title("day 1285 to day 1287")
ylabel("work - N*m")
grid on

disp(.5*spacecraftM*norm(spacecraftVel)*norm(spacecraftVel)-G*starM*spacecraftM/norm(starPos-spacecraftPos))

end
