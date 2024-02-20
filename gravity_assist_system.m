function gravity_assist_system()

% constant

% in Nm^2/kg^2
G = 6.6743e-11;

% in days
TIME_STEP_TOTAL = 1000;

TIME_STEP = 60*60*24;

% length scale in million km
DISTANCE = 1e+9;

% helper functions

function gravity = calculateGravity(p1,p2,m,M)
    % delta in position (million km)
    d = p2-p1;
    % convert million km to m
    d = d*1e+9;
    % calculate gravity in kgm/s^2
    g = G*m*M/(norm(d)*norm(d));
    gravity = g*d/norm(d);
end

function a = updateAcceleration(m,f)
    % calculate acceleration in m/s^2
    a = f/m;
end

function v = updateVelocity(v0,a)
    % calculate velocity in million km/s
    v = v0+a/DISTANCE;
end

function p = updatePosition(p0,v)
    % calculate position after a day
    p = p0+v*TIME_STEP;
end

% properties

% mass in kg
spacecraftM = 1000;
% position in million km
spacecraftPos = [-5,-.5,-1.8];
% velocity in million km/s
spacecraftVel = [0.15e-6,0.05e-6,0.05e-6];
% acceleration in m/s^2
spacecraftAcc = [0,0,0];

% same for the planet & star
starM = 4e+34;
starPos = [0,0,0];
starVel = [0,0,0];
starAcc = [0,0,0];

planet1M = 6.39e+27;
planet1Pos = [0,-50,0];
planet1Vel = [0.2e-4,0,0];
planet1Acc = [0,0,0];

planet2M = 3.39e+27;
planet2Pos = [0,30,0];
planet2Vel = [-0.2e-4,0,0];
planet2Acc = [0,0,0];

% animation

curve1 = animatedline('LineWidth',1);
curve2 = animatedline('LineWidth',1);
set(gca,'XLim',[-100,100],'YLim',[-100,100],'ZLim',[-100,100]);

hold on;
grid on;

for i=1:TIME_STEP_TOTAL
    addpoints(curve1,planet1Pos(1),planet1Pos(2),planet1Pos(3));
    addpoints(curve2,planet2Pos(1),planet2Pos(2),planet2Pos(3));
    head1 = scatter3(planet1Pos(1),planet1Pos(2),planet1Pos(3),50,'red');
    head2 = scatter3(planet2Pos(1),planet2Pos(2),planet2Pos(3),50,'blue');

    drawnow
    pause(0.01);
    
    if i ~= TIME_STEP_TOTAL
        delete(head1);
        delete(head2);
    end

    planet1Pos = updatePosition(planet1Pos,planet1Vel);
    planet2Pos = updatePosition(planet2Pos,planet2Vel);

    gM1 = calculateGravity(planet1Pos,starPos,planet1M,starM);
    gM2 = calculateGravity(planet2Pos,starPos,planet2M,starM);

    planet1Acc = updateAcceleration(planet1M,gM1);
    planet2Acc = updateAcceleration(planet2M,gM2);

    planet1Vel = updateVelocity(planet1Vel,planet1Acc);
    planet2Vel = updateVelocity(planet2Vel,planet2Acc);

    disp(planet1Pos)
end

end