rosrun xacro xacro.py yumi.urdf.xacro prefix:=VelocityJointInterface > yumi_generated.urdf
gz sdf -p yumi_generated.urdf > yumi_generated.sdf