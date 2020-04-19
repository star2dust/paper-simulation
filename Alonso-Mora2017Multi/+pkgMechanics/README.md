# Simple-Mechanics-Package
A package for simple mechanic instances, like mass point and rigid body.

## modified 27-03-2019

1. Add class `BodyConfig` as a general tool for all classes (support 0 or 2 input arguments).
2. Change the class names as `MassPointPlanar` and `RigidBodyPlanar` to specify the dimension of inputs.

## modified 28-03-2019

1. Finish the basic properties and methods of `MassPointPlanar` and `RigidBodyPlanar`.
2. Add some useful tool functions to the package.
3. Test `MassPointPlanar` and `RigidBodyPlanar` with example files `test_masspoint.m` and `test_rigidbody.m`. 
4. Add force input comes from keyboard, which requires [Hebi package](<https://github.com/HebiRobotics/MatlabInput>).

## modified 01-04-2019

1. Change `BodyConfig` class as a value class.
2. Keep `MassPointPlanar` & `RigidBodyPlanar` class as handles, but use `BodyConfig` as property type.
3. Restrict the inputs when defining a instance of class and setting the properties.
4. Fix the bug that the rotation.position of `RigidBodyPlanar` is not inside the range [-pi,pi].
5. Add 'setVelocity' method to each class such that 'upstateStates' can be useful when 0 force is applied.
6. Add Hebi package in the directory.

## modified 16-05-2019
1. Version updates. Change `MassPoint`(old name: 'MassPointPlanar') and `RigidCuboid`(old name: `RigidBodyPlanar`) class to 3D model.
2. Delete `BodyConfig` class.
3. Add tool functions `cross2` and `mat2strf`.
4. Add functions `edge2vertb`, `verticesPosition` and `verticesVelocity` in `RigidCuboid` class.
5. Add rotation matrix and homogeneous transformation matrix, which requires [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox).

## modified 17-05-2019
1. update test_Farivarnnerjad.m so that it is suitable for new 3d model.
2. Add `inhull` function to detect collision.
