clear all; close all; clc
% r alpha d theta
DH_params = [0.5   	pi/2	0.25   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];

robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
% body1.addVisual();
joint1 = robotics.Joint('joint1','revolute');
% joint1.HomePosition = 0;
setFixedTransform(joint1, DH_params(1,:),'dh');
body1.Joint = joint1;
addBody(robot,body1,'base');
body2 = robotics.RigidBody('body2');
joint2 = robotics.Joint('joint2','revolute');
% joint2.HomePosition = pi/2;
% trv = trvec2tform([1,0,0]);
setFixedTransform(joint2, DH_params(2,:),'dh');
body2.Joint = joint2;
addBody(robot,body2,'body1');

body3 = robotics.RigidBody('body3');
joint3 = robotics.Joint('joint3','revolute');
setFixedTransform(joint3, DH_params(3,:),'dh');
body3.Joint = joint3;
addBody(robot,body3,'body2');
config = robot.homeConfiguration;
% config = [-pi/2,pi/2,pi];
robot.show();
hold on;
plot3(1,1,1,'r*');
hold on;
robot.show(config);
hold on;
plot3(1,1,1,'r*');