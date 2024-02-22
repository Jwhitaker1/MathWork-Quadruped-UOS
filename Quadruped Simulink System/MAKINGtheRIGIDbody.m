QuadrupedRBT = rigidBodyTree("DataFormat","column");
base = QuadrupedRBT.Base;
body = rigidBody("body")

%% FR
FRshoulder = rigidBody("FRshoulder");
FRtopleg = rigidBody("FRtopleg");
FRbottomleg = rigidBody("FRbottomleg");

%% FL
FLshoulder = rigidBody("FLshoulder");
FLtopleg = rigidBody("FLtopleg");
FLbottomleg = rigidBody("FLbottomleg");

%% RR
RRshoulder = rigidBody("RRshoulder");
RRtopleg = rigidBody("RRtopleg");
RRbottomleg = rigidBody("RRbottomleg");

%% RL
RLshoulder = rigidBody("RLshoulder");
RLtopleg = rigidBody("RLtopleg");
RLbottomleg = rigidBody("RLbottomleg");

%% Collision Objects

collBase = collisionBox(1,0.45,0.15); 
coll1 = collisionBox(0.05,0.05,0.05); % box: length, width, height (x,y,z)
coll2 = collisionBox(0.05,0.05,0.35); % box: length, width, height (x,y,z)
coll2.Pose = trvec2tform([0 0 0.35/2]);
coll3 = collisionBox(0.05,0.05,0.25); % box: length, width, height (x,y,z)
coll3.Pose = trvec2tform([0 0 0.25/2]);

addCollision(body,collBase);

addCollision(FRshoulder,coll1);
addCollision(FRtopleg,coll2);
addCollision(FRbottomleg,coll3);

addCollision(FRshoulder,coll1);
addCollision(FRtopleg,coll2);
addCollision(FRbottomleg,coll3);

addCollision(FRshoulder,coll1);
addCollision(FRtopleg,coll2);
addCollision(FRbottomleg,coll3);

addCollision(FRshoulder,coll1);
addCollision(FRtopleg,coll2);
addCollision(FRbottomleg,coll3);

%% Joints
jntB = rigidBodyJoint("jntB","revolute");
jntFR1 = rigidBodyJoint("jntFR1","revolute");
jntFR2 = rigidBodyJoint("jntFR2","revolute");
jntFR3 = rigidBodyJoint("jntFR3","revolute");
jntFL1 = rigidBodyJoint("jntFL1","revolute");
jntFL2 = rigidBodyJoint("jntFL2","revolute");
jntFL3 = rigidBodyJoint("jntFL3","revolute");
jntRR1 = rigidBodyJoint("jntRR1","revolute");
jntRR2 = rigidBodyJoint("jntRR2","revolute");
jntRR3 = rigidBodyJoint("jntRR3","revolute");
jntRL1 = rigidBodyJoint("jntRL1","revolute");
jntRL2 = rigidBodyJoint("jntRL2","revolute");
jntRL3 = rigidBodyJoint("jntRL3","revolute");

jntB.JointAxis = [0 0 1];
jntFR1.JointAxis = [1 0 0];
jntFR2.JointAxis = [0 1 0];
jntFR3.JointAxis = [0 1 0];
jntFL1.JointAxis = [1 0 0];
jntFL2.JointAxis = [0 1 0];
jntFL3.JointAxis = [0 1 0];
jntRR1.JointAxis = [1 0 0];
jntRR2.JointAxis = [0 1 0];
jntRR3.JointAxis = [0 1 0];
jntRL1.JointAxis = [1 0 0];
jntRL2.JointAxis = [0 1 0];
jntRL3.JointAxis = [0 1 0];

setFixedTransform(jntB,trvec2tform([0 0 0]));

setFixedTransform(jntFR1,trvec2tform([0.375 -0.25 0]));
setFixedTransform(jntFR2,trvec2tform([0 0 0]));
setFixedTransform(jntFR3,trvec2tform([0 0 -0.35]));

setFixedTransform(jntFL1,trvec2tform([0.375 0.25 0]));
setFixedTransform(jntFL2,trvec2tform([0 0 0]));
setFixedTransform(jntFL3,trvec2tform([0 0 -0.35]));

setFixedTransform(jntRR1,trvec2tform([-0.375 -0.25 0]));
setFixedTransform(jntRR2,trvec2tform([0 0 0]));
setFixedTransform(jntRR3,trvec2tform([0 0 -0.35]));

setFixedTransform(jntRL1,trvec2tform([-0.375 0.25 0]));
setFixedTransform(jntRL2,trvec2tform([0 0 0]));
setFixedTransform(jntRL3,trvec2tform([0 0 -0.35]));

%% Assembly
% bodies = {base, body, FRshoulder, FRtopleg, FRbottomleg, FLshoulder, FLtopleg, FLbottomleg, RRshoulder, RRtopleg, RRbottomleg, RLshoulder, RLtopleg, RLbottomleg,};
% joints = {[],jntB,jntFR1,jntFR2,jntFR3,jntFL1,jntFL2,jntFL3,jntRR1,jntRR2,jntRR3,jntRL1,jntRL2,jntRL3};

figure("Name","Assemble Robot","Visible","on")

body.Joint = jntB;
addBody(QuadrupedRBT,body,base.Name)

FRshoulder.Joint = jntFR1;
addBody(QuadrupedRBT,FRshoulder,body.Name)
FRtopleg.Joint = jntFR2;
addBody(QuadrupedRBT,FRtopleg,FRshoulder.Name)
FRbottomleg.Joint = jntFR3;
addBody(QuadrupedRBT,FRbottomleg,FRtopleg.Name)

FLshoulder.Joint = jntFL1;
addBody(QuadrupedRBT,FLshoulder,body.Name)
FLtopleg.Joint = jntFL2;
addBody(QuadrupedRBT,FLtopleg,FLshoulder.Name)
FLbottomleg.Joint = jntFL3;
addBody(QuadrupedRBT,FLbottomleg,FLtopleg.Name)

RRshoulder.Joint = jntRR1;
addBody(QuadrupedRBT,RRshoulder,body.Name)
RRtopleg.Joint = jntRR2;
addBody(QuadrupedRBT,RRtopleg,RRshoulder.Name)
RRbottomleg.Joint = jntRR3;
addBody(QuadrupedRBT,RRbottomleg,RRtopleg.Name)

RLshoulder.Joint = jntRL1;
addBody(QuadrupedRBT,RLshoulder,body.Name)
RLtopleg.Joint = jntRL2;
addBody(QuadrupedRBT,RLtopleg,RLshoulder.Name)
RLbottomleg.Joint = jntRL3;
addBody(QuadrupedRBT,RLbottomleg,RLtopleg.Name)

show(QuadrupedRBT,"Collisions","on","Frames","off");
drawnow;

showdetails(QuadrupedRBT);

figure("Name","Interactive GUI")
gui = interactiveRigidBodyTree(QuadrupedRBT,"MarkerScaleFactor",0.25);
