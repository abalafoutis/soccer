from controller import Robot

from robot1 import Player1




robot = Robot()
name = robot.getName()
robot_number = int(name[1])

if robot_number == 1:
    robot_controller = Player1(robot)
elif robot_number == 2:
    robot_controller = Player1(robot)
else:
    robot_controller = Player1(robot)

robot_controller.run()
