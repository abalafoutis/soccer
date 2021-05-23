# -------------DEMOCRITUS----------------

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils
from enum import auto
import math

TIME_STEP = 64
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

ATTACK = auto()
DEFENDER_GO = auto()
DEFENDER_TURN = auto()
DEFENDER_MOVE = auto()
DEFENDER_STAY = auto()
MIDDLE_GO = auto()
MIDDLE_TURN = auto()
MIDDLE_STAY = auto()


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

        self.ballPrevY = 0
        self.ballCurrY = 0

        self.flip = False
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

    def assignRoles(self):
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
                    data[self.name]['orientation'] += 3.14159

                robot_pos = data[self.name]
                ball_pos = data['ball']
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

                self.dist1 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p1]['x'], data[self.p1]['y'])
                self.dist2 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p2]['x'], data[self.p2]['y'])
                self.dist3 = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.p3]['x'], data[self.p3]['y'])

                if self.clock == 50:
                    self.assignRoles()
                    if self.name == self.attacker:
                        self.state = ATTACK
                    elif self.name == self.middle:
                        self.state = MIDDLE_GO
                    elif self.name == self.defender:
                        self.state = DEFENDER_GO
                elif self.clock > 50:
                    if utils.isPlayerAhead(data[self.defender]['x'], data[self.middle]['x']):
                        if self.alarm2 == 0:
                            temp = self.defender
                            self.defender = self.middle
                            self.middle = temp
                            if self.name == self.middle:
                                self.state = MIDDLE_GO
                            elif self.name == self.defender:
                                self.state = DEFENDER_GO
                        self.alarm2 += 1
                        if self.alarm2 > 5:
                            self.alarm2 = 5
                    else:
                        self.alarm2 -= 1
                    if utils.isPlayerAhead(data[self.middle]['x'], data[self.attacker]['x']):
                        if self.alarm3 == 0:
                            temp = self.middle
                            self.middle = self.attacker
                            self.attacker = temp
                            if self.name == self.middle:
                                self.state = MIDDLE_GO
                            elif self.name == self.attacker:
                                self.state = ATTACK
                        self.alarm3 += 1
                        if self.alarm3 > 5:
                            self.alarm4 = 5
                    else:
                        self.alarm3 -= 1

                    self.ballCurrY = abs(ball_pos['y'])
                    if self.clock% 21 == 0:
                        self.ballPrevY = self.ballCurrY
                    if self.clock % 31 == 0 and abs(self.ballPrevY - self.ballCurrY) < 0.01:
                        print("STOOOOOOOOOOOOOOOOOOOOOOOOOOOOP")
                        if self.name == self.defender:
                            self.state = DEFENDER_STAY
                        if self.name == self.middle and self.state == ATTACK:
                            print("##################################################")
                            self.state == MIDDLE_GO

                    self.defenderDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.defender]['x'],
                                                    data[self.defender]['y'])
                    self.middleDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.middle]['x'],
                                                    data[self.middle]['y'])
                    self.attackerDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.attacker]['x'],
                                                    data[self.attacker]['y'])

                # -----------------STATES BEGIN --------------------------------

                if self.state == ATTACK:
                    left_speed, right_speed = utils.followBall(ball_angle)
                    left_speed = left_speed - 10
                    right_speed = right_speed - 10
                    if self.name == self.middle:
                        if not utils.isBallAhead(ball_pos['x'], robot_pos['x']):
                            self.state = MIDDLE_GO
                        elif utils.isBallAhead(ball_pos['x'], robot_pos['x']) and utils.isBallAhead(ball_pos['x'],data[self.attacker]['x']):
                            if self.middleDist > self.attackerDist:
                                self.state = MIDDLE_GO

                elif self.state == DEFENDER_GO:
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
                elif self.state == MIDDLE_GO:
                    self.alarm = 0
                    point = {'x': -0.2, 'y': 0}
                    p1, p2 = self.get_angles(point, robot_pos)
                    left_speed, right_speed = utils.followBall(p1)
                    left_speed = left_speed - 10
                    right_speed = right_speed - 10
                    if utils.get_ballDist(point['x'], point['y'], robot_pos['x'], robot_pos['y']) < 0.05:
                        self.state = MIDDLE_TURN
                    if self.name == self.middle:
                        if utils.isBallAhead(ball_pos['x'], robot_pos['x']) and self.middleDist < self.attackerDist:
                            self.state = ATTACK
                        elif utils.isBallAhead(ball_pos['x'], robot_pos['x']) and not utils.isBallAhead(
                                ball_pos['x'], data[self.attacker]['x']):
                            self.state = ATTACK
                elif self.state == DEFENDER_STAY:
                    self.alarm += 1
                    if self.alarm < 5:
                        left_speed, right_speed = -2, -2
                    elif self.alarm < 10:
                        left_speed, right_speed = 2, 2
                    else:
                        self.alarm = 0
                    if abs(ball_pos['y'] - robot_pos['y']) > 0.05 or self.defenderDist < 0.2:
                        self.state = DEFENDER_MOVE
                elif self.state == MIDDLE_TURN:
                    left_speed, right_speed = utils.turnRobot(90, robot_angle)
                    self.alarm += 1
                    if self.alarm > 20:
                        self.state = MIDDLE_STAY
                        self.alarm = 0
                elif self.state == MIDDLE_STAY:
                    self.alarm += 1
                    if self.alarm < 10:
                        left_speed, right_speed = 2, 2
                    elif self.alarm < 20:
                        left_speed, right_speed = -2, -2
                    else:
                        self.alarm = 0
                    if abs(-0.22 - robot_pos['x']) > 0.1:
                        self.state = MIDDLE_GO
                    if self.name == self.middle:
                        if utils.isBallAhead(ball_pos['x'], robot_pos['x']) and self.middleDist < self.attackerDist:
                            self.state = ATTACK
                        elif utils.isBallAhead(ball_pos['x'], robot_pos['x']) and not utils.isBallAhead(
                                ball_pos['x'], data[self.attacker]['x']):
                            self.state = ATTACK


                else:
                    left_speed = 0
                    right_speed = 0

                # -----------------STATES END --------------------------------

                if data['waiting_for_kickoff'] == True:
                    self.state = ATTACK
                    self.clock = 0
                    self.attacker = "-"
                    self.middle = "-"
                    self.defender = "-"

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)