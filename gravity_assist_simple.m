function gravity_assist_simple()

% constant

G = 6.6743e-11;

TIME_STEP_TOTAL = 1000;

% helper functions

function gravity = calculateGravity(p1,p2,m,M)
    d = [p2(1)-p1(1),p2(2)-p1(2),p2(3)-p1(3)];
    g = G*m*M/(norm(d)*norm(d));
    gravity = [g*d(1)/norm(d),g*d(2)/norm(d),g*d(2)/norm(d)];
end

function a = updateAcceleration(a0,m,f)
    a = [a0(1)+f(1)/m,a0(2)+f(2)/m,a0(3)+f(3)/m];
    a = [a(1)*1e-18,a(2)*1e-18,a(3)*1e-18];
end

function v = updateVelocity(v0,a)
    v = [v0(1)+a(1),v0(2)+a(2),v0(3)+a(3)];
end

function p = updatePosition(p0,v)
    p = [p0(1)+v(1),p0(2)+v(2),p0(3)+v(3)];
end

% properties
% mass in kg
% distance in million km (10^9 m)

spacecraftM = 1000;
spacecraftPos = [-0.5,-2,0];
spacecraftVel = [0,0.01,0];
spacecraftAcc = [0,0,0];

planetM = 6.39e+23;
planetPos = [0,0,0];
planetVel = [0,0,0];
planetAcc = [0,0,0];

% animation

curve1 = animatedline('LineWidth',1);
curve2 = animatedline('LineWidth',1);
set(gca,'XLim',[-20,20],'YLim',[-20,20],'ZLim',[-20,20]);

hold on;
grid on;

for i=1:TIME_STEP_TOTAL
    addpoints(curve1,spacecraftPos(1),spacecraftPos(2),spacecraftPos(3));
    addpoints(curve2,planetPos(1),planetPos(2),planetPos(3));
    head1 = scatter3(spacecraftPos(1),spacecraftPos(2),spacecraftPos(3),1,'red');
    head2 = scatter3(planetPos(1),planetPos(2),planetPos(3),10,'blue');

    drawnow
    pause(0.01);
    
    if i ~= TIME_STEP_TOTAL
        delete(head1);
        delete(head2);
    end

    spacecraftPos = updatePosition(spacecraftPos,spacecraftVel);
    planetPos = updatePosition(planetPos,planetVel);

    gm = calculateGravity(spacecraftPos,planetPos,spacecraftM,planetM);

    spacecraftAcc = updateAcceleration(spacecraftAcc,spacecraftM,gm);

    spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
    planetVel = updateVelocity(planetVel,planetAcc);
end

end
