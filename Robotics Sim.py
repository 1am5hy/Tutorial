from urdfpy import URDF

robot = URDF.load('/home/ubuntu/PycharmProjects/pythonProject/Tutorial/franka_description/robots/franka_panda.urdf')

print(robot)

for link in robot.joints:
    print(link.name)

robot.show(cfg={
     'panda_joint1': -2.0,
     'panda_joint6': 2.0
})



