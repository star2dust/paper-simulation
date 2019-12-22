close all
clear

rb = pkgMechanics.RigidBodyPlanar([1,1,1,1],[-1,-1,1,1;-1,1,1,-1]-4);
rb.translation.velocity = [0.3;0];
velocityDesired = [0.3;0.3];
positionDesired = [14;14];
cycle = 0.05;
count = 0;
figure
for t=0:cycle:60
    count = count+1;
    % display information
    verticePosition = [rb.verticeList.position];
    verticeVelocity = [rb.verticeList.velocity];
    fill(verticePosition(1,:),verticePosition(2,:),'r');hold on
    axis([-5 15 -5 15])
    rb.getStates;
    plot(positionDesired(1),positionDesired(2),'bx')
    % force control
    kGain = 0.1; kIGain = 0.5;
    forceCtrlList = -kGain*(verticeVelocity-velocityDesired);
    saveForceCtrlList(:,count) = forceCtrlList(:);
    forceCtrlList(:) = forceCtrlList(:) + kIGain*sum(saveForceCtrlList,2)*cycle;
    disp(['forceCtrlList: ',pkgMechanics.mat2strDisplay(forceCtrlList,2)])
    % input visualization
    quiver(rb.translation.position(1),rb.translation.position(2),rb.translation.velocity(1),rb.translation.velocity(2))
    quiver(verticePosition(1,:),verticePosition(2,:),verticeVelocity(1,:),verticeVelocity(2,:))
    quiver(verticePosition(1,:),verticePosition(2,:),forceCtrlList(1,:),forceCtrlList(2,:))
    % update information
    saveTranslationPosition(:,count) = rb.translation.position(:);
    plot(saveTranslationPosition(1,:),saveTranslationPosition(2,:),'b')
    rb.updateStates(forceCtrlList,cycle);
    pause(cycle)
    hold off
    clc
end