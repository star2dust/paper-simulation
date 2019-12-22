close all
clear

rb = pkgMechanics.RigidBodyPlanar([1,1,1,1],[-1,-1,1,1;-1,1,1,-1]);
kb = pkgHebi.HebiKeyboard();
cycle = 0.1;
figure
while true
    % display information
    verticePosition = [rb.verticeList.position];
    verticeVelocity = [rb.verticeList.velocity];
    fill(verticePosition(1,:),verticePosition(2,:),'r');hold on
    axis([-5 5 -5 5])
    rb.getStates;
    % keyboard force input
    state = read(kb);
    contactPoint = state.keys('1234');
    forceDirection = [state.UP,state.DOWN,state.LEFT,state.RIGHT];
    disp(['forceDirection: ',mat2str(forceDirection)])
    forceTemplate = [0,0,-1,1;1,-1,0,0];
    forceList = kron(contactPoint,sum(forceDirection.*forceTemplate,2));
    % input visualization
    quiver(rb.translation.position(1),rb.translation.position(2),rb.translation.velocity(1),rb.translation.velocity(2))
    quiver(verticePosition(1,:),verticePosition(2,:),verticeVelocity(1,:),verticeVelocity(2,:))
    quiver(verticePosition(1,:),verticePosition(2,:),forceList(1,:),forceList(2,:))
    % update information
    rb.updateStates(forceList,cycle);
    pause(cycle)
    hold off
    clc
end