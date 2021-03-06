# -------------DEMOCRITUS----------------

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils
from enum import auto
import math

TIME_STEP = 64
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

ATTACK = auto()
ATTACK_BACK = auto()
DEFENDER_GO = auto()
DEFENDER_TURN = auto()
DEFENDER_MOVE = auto()
DEFENDER_STAY = auto()
CENTER_GO = auto()
CENTER_TURN = auto()
CENTER_MOVE = auto()
LEFT_GO = auto()
LEFT_TURN = auto()
LEFT_MOVE = auto()
LOW_LEFT_GO = auto()
LOW_LEFT_TURN = auto()
LOW_LEFT_MOVE = auto()
RIGHT_GO = auto()
RIGHT_TURN = auto()
RIGHT_MOVE = auto()
LOW_RIGHT_GO = auto()
LOW_RIGHT_TURN = auto()
LOW_RIGHT_MOVE = auto()

MIDDLE_ATTACK = auto()
MIDDLE_FOLLOW = auto()
MIDDLE_FOLLOW_REVERSE = auto()
MIDDLE_SELECT = auto()


class MyRobot1(RCJSoccerRobot):
    def __init__(self, robot):
        self.robot = robot
        self.name = self.robot.getName()

        self.attacker = "-"
        self.middle = "-"
        self.defender = "-"
        self.attackerDist = 0
        self.middleDist = 0
        self.defenderDist = 0

        self.state = ATTACK

        self.alarm = 0
        self.alarm2 = 0
        self.alarm3 = 0

        self.lastBall = None
        self.lop = [0 for _ in range(90)]
        self.sumLop = 0
        self.lopFlag = False

        self.flip = False
        self.changeAttack = False
        self.changeMiddleAttack = False
        if self.name[0] == 'B':
            self.flip = True

        self.p1 = self.name[0] + "1"
        self.p2 = self.name[0] + "2"
        self.p3 = self.name[0] + "3"

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float('+inf'))
        self.right_motor.setPosition(float('+inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def assignRoles(self, ball_pos, robot_pos, data):
        self.dist1 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p1]['x'], data[self.p1]['y'])
        self.dist2 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p2]['x'], data[self.p2]['y'])
        self.dist3 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p3]['x'], data[self.p3]['y'])

        if self.dist1 > self.dist2 and self.dist1 > self.dist3:
            self.defender = self.p1
            if self.dist2 > self.dist3:
                self.middle = self.p2
                self.attacker = self.p3
            else:
                self.attacker = self.p2
                self.middle = self.p3
        elif self.dist2 > self.dist1 and self.dist2 > self.dist3:
            self.defender = self.p2
            if self.dist1 > self.dist3:
                self.middle = self.p1
                self.attacker = self.p3
            else:
                self.middle = self.p3
                self.attacker = self.p1
        elif self.dist3 > self.dist1 and self.dist3 > self.dist2:
            self.defender = self.p3
            if self.dist1 > self.dist2:
                self.middle = self.p1
                self.attacker = self.p2
            else:
                self.middle = self.p2
                self.attacker = self.p1

        if self.name == self.attacker:
            if utils.isBallAhead(ball_pos['x'], robot_pos['x']):
                self.state = ATTACK
            else:
                self.state = ATTACK_BACK
        elif self.name == self.middle:
            self.state = MIDDLE_SELECT
        elif self.name == self.defender:
            self.state = DEFENDER_GO

    def run(self):
        self.clock = 0
        while self.robot.step(TIME_STEP) != -1:
            if self.receiver.getQueueLength() > 0:
                self.clock += 1

                data = self.get_new_data()
                if self.flip:
                    data['ball']['x'] *= -1
                    data['ball']['y'] *= -1
                    data[self.p1]['x'] *= -1
                    data[self.p1]['y'] *= -1
                    data[self.p2]['x'] *= -1
                    data[self.p2]['y'] *= -1
                    data[self.p3]['x'] *= -1
                    data[self.p3]['y'] *= -1
                    data[self.name]['orientation'] += math.pi


                robot_pos = data[self.name]
                ball_pos = data['ball']
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

                if self.lastBall == None:
                    self.lastBall = [ball_pos['x'], ball_pos['y']]
                error = math.sqrt((self.lastBall[0]-ball_pos['x'])**2 + (self.lastBall[1]-ball_pos['y'])**2)

                self.lop[self.clock % 90] = error
                self.lastBall = [ball_pos['x'], ball_pos['y']]
                self.sumLop = sum(self.lop)

                if self.clock > 50:
                    self.defenderDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.defender]['x'],
                                                    data[self.defender]['y'])
                    self.middleDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.middle]['x'],
                                                    data[self.middle]['y'])
                    self.attackerDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.attacker]['x'],
                                                    data[self.attacker]['y'])

                if self.clock == 50:
                    self.assignRoles(ball_pos, robot_pos, data)
                elif self.clock > 50 and self.sumLop <= 0.5 and not self.lopFlag:
                    self.lopFlag = True
                    if ball_pos['x'] > 0:
                        if self.name == self.middle:
                            if ball_pos['y'] > 0:
                                self.state = LEFT_GO
                            else:
                                self.state = RIGHT_GO
                        elif self.name == self.defender:
                            self.state = CENTER_GO
                    else:
                        if self.name == self.middle:
                            if ball_pos['y'] > 0:
                                self.state = LOW_RIGHT_GO
                            else:
                                self.state = LOW_LEFT_GO

                        elif self.name == self.defender:
                            self.state = DEFENDER_STAY


                elif self.clock > 50 and self.sumLop > 0.5 and self.lopFlag:
                    self.lopFlag = False
                    if ball_pos['x'] > 0:
                        self.assignRoles(ball_pos, robot_pos, data)
                    else:
                        if self.name == self.middle:
                            self.state = MIDDLE_SELECT
                        elif self.name == self.defender:
                            self.state = DEFENDER_MOVE


                # -----------------STATES BEGIN --------------------------------

                if self.name == self.attacker or self.clock < 50:
                    if self.state == ATTACK:
                        mx = robot_pos['x'] + 0.2
                        my = ball_pos['y']
                        if mx > ball_pos['x']:
                            mx = ball_pos['x']
                        if not utils.isBallAhead(ball_pos['x'], robot_pos['x']):
                            self.state = ATTACK_BACK
                    elif self.state == ATTACK_BACK:
                        mx = robot_pos['x'] - 0.2
                        if ball_pos['y'] - robot_pos['y'] < 0:
                            my = ball_pos['y'] + 0.2
                        else:
                            my = ball_pos['y'] - 0.2
                        if utils.isBallAhead(ball_pos['x'], robot_pos['x']+0.1):
                            self.state = ATTACK
                            if self.changeAttack == True:
                                self.changeAttack = False
                            else:
                                self.changeAttack = True

                    point = {}
                    point['x'] = mx
                    point['y'] = my
                    p1, p2 = self.get_angles(point, robot_pos)
                    if self.changeAttack == False:
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                    else:
                        left_speed, right_speed = utils.followBallFliped(p1)
                        left_speed = left_speed + 10
                        right_speed = right_speed + 10

                # _________________________________________________________________________________
                # ----------------------     MIDDLE    --------------------------------------------
                # _________________________________________________________________________________

                elif self.name == self.middle:
                    if self.state == MIDDLE_SELECT:
                        if utils.isBallAhead(ball_pos['x'], robot_pos['x']) and utils.isBallAhead(ball_pos['x'],data[self.attacker]['x']):
                            if self.middleDist > self.attackerDist:
                                self.state = MIDDLE_FOLLOW
                            else:
                                self.state = MIDDLE_ATTACK
                        elif not utils.isBallAhead(ball_pos['x'], robot_pos['x']):
                            #if self.middleDist >= self.attackerDist:
                            self.state = MIDDLE_FOLLOW_REVERSE



                    elif self.state == MIDDLE_ATTACK:

                        mx = robot_pos['x'] + 0.2
                        my = ball_pos['y']
                        if mx > ball_pos['x']:
                            mx = ball_pos['x']
                        point = {}
                        point['x'] = mx
                        point['y'] = my
                        p1, p2 = self.get_angles(point, robot_pos)
                        if self.changeMiddleAttack == False:
                            left_speed, right_speed = utils.followBall(p1)
                            left_speed = left_speed - 10
                            right_speed = right_speed - 10
                        else:
                            left_speed, right_speed = utils.followBallFliped(p1)
                            left_speed = left_speed + 10
                            right_speed = right_speed + 10
                        self.state = MIDDLE_SELECT


                    elif self.state == MIDDLE_FOLLOW:
                        if data[self.attacker]['y'] > 0:
                            temp = -0.5
                        else:
                            temp = 0.5

                        point = {}
                        point['x'] = data[self.attacker]['x']
                        point['y'] = data[self.attacker]['y'] + temp
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 5
                        right_speed = right_speed - 5
                        self.state = MIDDLE_SELECT

                    elif self.state == MIDDLE_FOLLOW_REVERSE:
                        if ball_pos['y'] > 0:
                            sety = -0.3
                        else:
                            sety = 0.3
                        if ball_pos['x'] <= -0.6:
                            setx = -0.5
                        else:
                            setx = ball_pos['x']
                        point = {}
                        point['x'] = setx
                        point['y'] = ball_pos['y'] + sety
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 6
                        right_speed = right_speed - 6
                        self.state = MIDDLE_SELECT


                    elif self.state == RIGHT_GO:
                        point = {'x': 0.2, 'y': 0.35}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                            self.state = RIGHT_TURN
                    elif self.state == RIGHT_TURN:
                        left_speed, right_speed = utils.turnRobot(135, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = RIGHT_MOVE
                            self.alarm = 0
                    elif self.state == RIGHT_MOVE:
                        self.alarm += 1
                        if self.alarm < 10:
                            left_speed, right_speed = 2, 2
                        elif self.alarm < 20:
                            left_speed, right_speed = -2, -2
                        else:
                            self.alarm = 0
                        if self.middleDist < 0.2:
                            self.state = ATTACK
                    elif self.state == LOW_RIGHT_GO:
                        point = {'x': -0.4, 'y': 0.3}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                            self.state = LOW_RIGHT_TURN
                    elif self.state == LOW_RIGHT_TURN:
                        left_speed, right_speed = utils.turnRobot(45, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = LOW_RIGHT_MOVE
                            self.alarm = 0
                    elif self.state == LOW_RIGHT_MOVE:
                        self.alarm += 1
                        if self.alarm < 10:
                            left_speed, right_speed = 2, 2
                        elif self.alarm < 20:
                            left_speed, right_speed = -2, -2
                        else:
                            self.alarm = 0
                        if self.middleDist < 0.2:
                            self.state = ATTACK
                    elif self.state == LEFT_GO:
                        point = {'x': 0.2, 'y': -0.35}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                            self.state = LEFT_TURN
                    elif self.state == LEFT_TURN:
                        left_speed, right_speed = utils.turnRobot(45, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = LEFT_MOVE
                            self.alarm = 0
                    elif self.state == LEFT_MOVE:
                        self.alarm += 1
                        if self.alarm < 10:
                            left_speed, right_speed = 2, 2
                        elif self.alarm < 20:
                            left_speed, right_speed = -2, -2
                        else:
                            self.alarm = 0
                        if self.middleDist < 0.2:
                            self.state = ATTACK
                    elif self.state == LOW_LEFT_GO:
                        point = {'x': -0.4, 'y': -0.3}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                            self.state = LOW_LEFT_TURN
                    elif self.state == LOW_LEFT_TURN:
                        left_speed, right_speed = utils.turnRobot(45, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = LOW_LEFT_MOVE
                            self.alarm = 0
                    elif self.state == LOW_LEFT_MOVE:
                        self.alarm += 1
                        if self.alarm < 10:
                            left_speed, right_speed = 2, 2
                        elif self.alarm < 20:
                            left_speed, right_speed = -2, -2
                        else:
                            self.alarm = 0
                        if self.middleDist < 0.2:
                            self.state = ATTACK
                    elif self.state == ATTACK:
                        left_speed, right_speed = utils.followBall(ball_angle)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                    else:
                        left_speed = 0
                        right_speed = 0

                # _________________________________________________________________________________
                # ----------------------     DEFENDER    --------------------------------------------
                # _________________________________________________________________________________

                elif self.name == self.defender:

                    if  self.state == DEFENDER_GO:
                        point = {'x': -0.6, 'y': 0}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.1:
                            self.state = DEFENDER_TURN


                    elif self.state == DEFENDER_TURN:
                        left_speed, right_speed = utils.turnRobot(0, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = DEFENDER_MOVE
                            self.alarm = 0


                    elif self.state == DEFENDER_MOVE:
                        speed = 300 * (ball_pos['y'] - robot_pos['y'])
                        left_speed, right_speed = -speed, -speed
                        if abs(-0.6 - robot_pos['x']) > 0.1:
                            self.state = DEFENDER_GO


                    elif self.state == DEFENDER_STAY:
                        if utils.get_ballDist(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y']) > 0.2:
                            if robot_pos['y'] > 0:
                                left_speed, right_speed = -4, -4
                            else:
                                left_speed, right_speed = 4, 4
                        else:
                            left_speed, right_speed = 0, 0
                        if abs(-0.6 - robot_pos['x']) > 0.1 or abs(math.degrees(robot_angle)) > 5:
                            self.state = DEFENDER_GO

                    elif self.state == CENTER_GO:
                        point = {'x': -0.35, 'y': 0}
                        p1, p2 = self.get_angles(point, robot_pos)
                        left_speed, right_speed = utils.followBall(p1)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10
                        if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                            self.state = CENTER_TURN

                    elif self.state == CENTER_TURN:
                        left_speed, right_speed = utils.turnRobot(90, robot_angle)
                        self.alarm += 1
                        if self.alarm > 20:
                            self.state = CENTER_MOVE
                            self.alarm = 0

                    elif self.state == CENTER_MOVE:
                        self.alarm += 1
                        if self.alarm < 10:
                            left_speed, right_speed = 2, 2
                        elif self.alarm < 20:
                            left_speed, right_speed = -2, -2
                        else:
                            self.alarm = 0
                        if self.defenderDist < 0.2:
                            self.state = ATTACK

                    elif self.state == ATTACK:
                        left_speed, right_speed = utils.followBall(ball_angle)
                        left_speed = left_speed - 10
                        right_speed = right_speed - 10


                # -----------------STATES END --------------------------------

                if data['waiting_for_kickoff'] == True:
                    self.state = ATTACK
                    self.clock = 0
                    self.attacker = "-"
                    self.middle = "-"
                    self.defender = "-"
                    self.changeMiddleAttack = False
                    self.changeAttack = False

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
