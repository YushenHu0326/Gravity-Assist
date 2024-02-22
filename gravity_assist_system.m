function gravity_assist_system()

clear global;
close all;

% constant

% in Nm^2/kg^2
G = 6.6743e-11;

% in days
TIME_STEP_TOTAL = 1000;

TIME_STEP = 60*60*24;

DAYS_PER_TIME_STEP = 20;

TACC_1 = 2390;
TACC_2 = 3550;

A_1 = 0.1;
A_1_s = 0.0;
A_2 = 0.04;
A_2_s = 0.08;

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
    v = v0+a*TIME_STEP;
end

function p = updatePosition(p0,v)
    % calculate position after a day
    p = p0+v*TIME_STEP;
end

function w = updateWork(w0,f,d,dir)
    w = w0 + vec2vecProj(f,dir)*norm(vec2vecProj(d,dir));
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

for i = 1:53
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

for i = 1:1732
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

for i = 1:6757
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

for i = 1:12235
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

    drawnow
    pause(0.01);
    
    if i ~= TIME_STEP_TOTAL
        delete(head1);
        delete(head2);
        delete(head3);
        delete(head4);
        delete(head5);
    end
    
    for j = 1:DAYS_PER_TIME_STEP
        %disp((i-1)*DAYS_PER_TIME_STEP+j)
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
        
        %disp(norm(spacecraftPos-planet2Pos))
        %disp((i-1)*DAYS_PER_TIME_STEP+j)
        disp(norm(calculateGravity(spacecraftPos,planet2Pos,spacecraftM,planet2M)))

        if (i-1)*DAYS_PER_TIME_STEP+j < TACC_1
            spacecraftPos = planet1Pos;
            spacecraftVel = planet1Vel;
        elseif (i-1)*DAYS_PER_TIME_STEP+j == TACC_1
            spacecraftAcc = spacecraftVel/norm(spacecraftVel)*A_1;
            spacecraftAcc = sideBoost(spacecraftAcc,spacecraftVel,A_1_s);
            spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
        elseif (i-1)*DAYS_PER_TIME_STEP+j == TACC_2
            spacecraftAcc = spacecraftVel/norm(spacecraftVel)*A_2;
            spacecraftAcc = sideBoost(spacecraftAcc,spacecraftVel,A_2_s);
            spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
        else
            spacecraftPos = updatePosition(spacecraftPos,spacecraftVel);
            gm = calculateGravity(spacecraftPos,starPos,spacecraftM,starM);
            gm = gm + calculateGravity(spacecraftPos,planet1Pos,spacecraftM,planet1M);
            gm = gm + calculateGravity(spacecraftPos,planet2Pos,spacecraftM,planet2M);
            gm = gm + calculateGravity(spacecraftPos,planet3Pos,spacecraftM,planet3M);
            gm = gm + calculateGravity(spacecraftPos,planet4Pos,spacecraftM,planet4M);
            spacecraftAcc = updateAcceleration(spacecraftM,gm);
            spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
        end
    end
end

end
