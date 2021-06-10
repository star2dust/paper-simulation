author: shiyu zhao
date: March 2018

the code has been tested in matlab 2010 and 2015
the code is for the paper "S. Zhao, Affine formation maneuver control of multi-agent systems, IEEE Transactions on Automatic Control, accepted, Sep 2017"

hint:

>>how to use: simply run main.m, then you will see the simulation results and animation

>> in order to obtain accurate simulation result, you can reduce the value of "stepsize". The default value of stepsize is 0.005. Note that there is acceleration feedback, which is delayed by the time of stepsize. The value used in the paper is 0.001

>> you can record video. to do that just search and uncomment the following commands in fcn_Animation_2DObstacle.m

% mov = avifile('video1.avi');
%     frame=getframe(gcf);
%     mov = addframe(mov,frame); 
% mov = close(mov);

>> function fcn_StressMatrix can solve the stress matrix of a given network just for simple cases