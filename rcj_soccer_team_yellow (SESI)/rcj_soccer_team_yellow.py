from controller import Robot

from robot1 import MyRobot1
from robot2 import MyRobot2
from robot3 import MyRobot3

OP_GOAL = [-0.856, 0]  # Opponent's goal position [x, y]
OWN_GOAL = [0.856, 0]  # Own's goal position [x, y]
counter = 0
score = [0, 0]
GOAL_FLAG = False
TIME_FLAG = False
IMPORT_FLAG = False
GOAL_DEFAULT_DIST = 0.4
goal_time = 0
STOP_NEAR_GOALKEEPER = True
KICK_FLAG = 0
KICK_INTENSITY_DEFAULT = 5
MIRROR_FLAG = True
CONTANDO_FLAG = False
BALL_AREA = [0, 0, 0, 0]
INICIO = 0
ASSISTENCIA = True  # anda na linha da bola no atk
SUPORTE_GOLEIRO = False  # Ajuda o goleiro 'jogo de corpo'


robot = Robot()
name = robot.getName()
robot_number = int(name[1])

if robot_number == 1:
    robot_controller = MyRobot1(robot)
elif robot_number == 2:
    robot_controller = MyRobot2(robot)
else:
    robot_controller = MyRobot3(robot)

robot_controller.run(counter, score, goal_time, GOAL_FLAG, TIME_FLAG, IMPORT_FLAG, KICK_FLAG, CONTANDO_FLAG, BALL_AREA, INICIO)
