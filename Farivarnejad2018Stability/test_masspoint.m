close all
clear

mp = pkgMechanics.MassPointPlanar;
kb = pkgHebi.HebiKeyboard;
cycle = 0.1;
figure
while true
    % display information
    plot(mp.position(1),mp.position(2),'ro');hold on
    axis([-5 5 -5 5])
    mp.getStates;
    % keyboard force input
    state = read(kb);
    forceDirection = [state.UP,state.DOWN,state.LEFT,state.RIGHT];
    disp(['forceDirection: ',mat2str(forceDirection)])
    forceTemplate = [0,0,-1,1;1,-1,0,0];
    forceSum = sum(forceDirection.*forceTemplate,2);
    % input visualization
    quiver(mp.position(1),mp.position(2),mp.velocity(1),mp.velocity(2))
    quiver(mp.position(1),mp.position(2),forceSum(1),forceSum(2))
    % update information
    mp.updateStates(forceSum,cycle);
    pause(cycle)
    hold off
    clc
end