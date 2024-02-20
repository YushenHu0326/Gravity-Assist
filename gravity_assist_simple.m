function gravity_assist_simple()

% constant

% in Nm^2/kg^2
G = 6.6743e-11;

% in days
TIME_STEP_TOTAL = 500;

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

% same for the planet
planetM = 6.39e+28;
planetPos = [0,-5,0];
planetVel = [0,0.2e-6,0];
planetAcc = [0,0,0];

% animation

curve1 = animatedline('LineWidth',1);
curve2 = animatedline('LineWidth',1);
set(gca,'XLim',[-10,10],'YLim',[-10,10],'ZLim',[-10,10]);

hold on;
grid on;

for i=1:TIME_STEP_TOTAL
    addpoints(curve1,spacecraftPos(1),spacecraftPos(2),spacecraftPos(3));
    addpoints(curve2,planetPos(1),planetPos(2),planetPos(3));
    head1 = scatter3(spacecraftPos(1),spacecraftPos(2),spacecraftPos(3),10,'red');
    head2 = scatter3(planetPos(1),planetPos(2),planetPos(3),100,'blue');

    drawnow
    pause(0.01);
    
    if i ~= TIME_STEP_TOTAL
        delete(head1);
        delete(head2);
    end

    spacecraftPos = updatePosition(spacecraftPos,spacecraftVel);
    planetPos = updatePosition(planetPos,planetVel);

    gm = calculateGravity(spacecraftPos,planetPos,spacecraftM,planetM);
    gM = calculateGravity(planetPos,spacecraftPos,planetM,spacecraftM);

    spacecraftAcc = updateAcceleration(spacecraftM,gm);
    planetAcc = updateAcceleration(planetM,gM);

    spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
    planetVel = updateVelocity(planetVel,planetAcc);
    disp(norm(spacecraftVel));
end

end
