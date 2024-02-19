function gravity_assist_static()

% helper functions
function proj = vec2vecProj(a,b)
    p = dot(a,b)/(norm(b)*norm(b));
    proj = [b(1)*p,b(2)*p,b(3)*p];
end

% properties

TIME_STEP_TOTAL = 1000;

spacecraftPos = [-100,-100,0];
spacecraftVel = [0.1,0.1,0.1];

planetPos = [0,0,0];
planetVel = [0,0,0];

% animation

curve = animatedline('LineWidth',1);
set(gca,'XLim',[-200,200],'YLim',[-200,200],'ZLim',[-200,200]);

hold on;
grid on;

for i=1:TIME_STEP_TOTAL
    addpoints(curve,spacecraftPos(1),spacecraftPos(2),spacecraftPos(3));
    head1 = scatter3(spacecraftPos(1),spacecraftPos(2),spacecraftPos(3),1,'red');
    head2 = scatter3(planetPos(1),planetPos(2),planetPos(3),10,'blue');

    drawnow
    pause(0.01);

    delete(head1);
    delete(head2);

    spacecraftPos(1) = spacecraftPos(1) + spacecraftVel(1);
    spacecraftPos(2) = spacecraftPos(2) + spacecraftVel(2);
    spacecraftPos(3) = spacecraftPos(3) + spacecraftVel(3);

    planetPos(1) = planetPos(1) + planetVel(1);
    planetPos(2) = planetPos(2) + planetVel(2);
    planetPos(3) = planetPos(3) + planetVel(3);
end

end
