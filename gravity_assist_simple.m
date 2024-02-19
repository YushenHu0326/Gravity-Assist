function gravity_assist_simple()

% helper functions
function proj = vec2vecProj(a,b)
    p = dot(a,b)/(norm(b)*norm(b));
    proj = [b(1)*p,b(2)*p,b(3)*p];
end

function v = updateVelocity(v0,a)
    v = [v0(1)+a(1),v0(2)+a(2),v0(3)+a(3)];
end

function p = updatePosition(p0,v)
    p = [p0(1)+v(1),p0(2)+v(2),p0(3)+v(3)];
end

% properties

TIME_STEP_TOTAL = 1000;

spacecraftPos = [-100,-100,0];
spacecraftVel = [0.1,0.1,0.1];
spacecraftAcc = [0,0,0];

planetPos = [0,0,0];
planetVel = [0,0,0];
planetAcc = [0,0,0];

% animation

curve1 = animatedline('LineWidth',1);
curve2 = animatedline('LineWidth',1);
set(gca,'XLim',[-200,200],'YLim',[-200,200],'ZLim',[-200,200]);

hold on;
grid on;

for i=1:TIME_STEP_TOTAL
    addpoints(curve1,spacecraftPos(1),spacecraftPos(2),spacecraftPos(3));
    addpoints(curve2,planetPos(1),planetPos(2),planetPos(3));
    head1 = scatter3(spacecraftPos(1),spacecraftPos(2),spacecraftPos(3),1,'red');
    head2 = scatter3(planetPos(1),planetPos(2),planetPos(3),10,'blue');

    drawnow
    pause(0.01);

    delete(head1);
    delete(head2);

    spacecraftPos = updatePosition(spacecraftPos,spacecraftVel);
    planetPos = updatePosition(planetPos,planetVel);

    spacecraftVel = updateVelocity(spacecraftVel,spacecraftAcc);
    planetVel = updateVelocity(planetVel,planetAcc);
end

end
