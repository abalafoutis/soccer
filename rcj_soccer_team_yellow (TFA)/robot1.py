from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import struct
import math

''' ------------------------------ Init Constants ------------------------------ '''


ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

# simulation
GOAL_X_LIMIT = 0.745
Y_WALL = 0.65
X_WALL = 0.75
BALL_R = 0.021
ROBOT_R = 0.05

BALL_X_WALL = X_WALL-BALL_R
BALL_Y_WALL = Y_WALL-BALL_R

ROBOT_X_WALL = X_WALL-ROBOT_R
ROBOT_Y_WALL = Y_WALL-ROBOT_R

CORNER_SIDE = 0.05
BALL_X_CORNER = BALL_X_WALL-CORNER_SIDE
BALL_Y_CORNER = BALL_Y_WALL-CORNER_SIDE

GOAL_X_ROBOT = 0.72
ROBOT_NEAR_WALL = 0.6

REL_POS_CLOSE = 0
REL_POS_MID = 1
REL_POS_FAR = 2

ROLE_FOLLOW = 0
ROLE_DEFENCE = 1
ROLE_ATTACK_CENTER = 2
ROLE_ATTACK_SIDE = 3
ROLE_NS_WAIT = 4
ROLE_GOALIE_SIDE = 5
ROLE_GOALIE_CENTER = 6

STATE_NEAR = 0
STATE_BALL = 1
STATE_BEHIND = 2
STATE_WAIT = 3
STATE_STOP = 4
STATE_DEFEND = 5
STATE_TURN_TO_GOAL = 6
STATE_ATTACK = 7
STATE_AWAY = 8

BALL_PREDICT_P_DIST = 1/0.03
BALL_DECEL = 0.5

# distance
NEAR_BALL_DIST = 0.1
BEHIND_BALL_DIST = 0.12
TURN_DIST = 0.2
STOP_WITHIN = math.sqrt(((0.05/2)**2)*2)
BALL_AWAY_DIST = 0.3
DEFEND_BALL_DIST_SIDE = 0.5
DEFEND_BALL_DIST_CENTER = 0.35
NEAR_NS_DIST = 0.15

SPD_P_NORMAL = 2
SPD_P_NEAR = 3
SPD_P_DEFEND = 3
SPD_P_ATTACK = 3

# near ball
NEAR_BALL_ANG = 60
NEAR_BALL_MIN_SPD = 10


class LOP_Checker:
    def __init__(self, COUNTS, MIN_DIST):
        self.COUNTS = COUNTS
        self.MIN_DIST = MIN_DIST
        self.reset()

    def reset(self):
        self.dist = [0 for _ in range(self.COUNTS)]
        self.counter = 0
        self.last_pos = None

    def check_LOP(self, pos):
        if self.last_pos == None:
            self.last_pos = pos

        delta = math.sqrt((self.last_pos[0] - pos[0])**2 + (self.last_pos[1] - pos[1])**2)

        self.dist[self.counter % self.COUNTS] = delta
        self.counter += 1

        self.last_pos = pos

        s = sum(self.dist)

        if self.counter < self.COUNTS:
            return False

        return s < self.MIN_DIST


class Player1(RCJSoccerRobot):

    ''' ------------------------------ Init ------------------------------ '''
    def __init__(self, robot):

        # ---------- robot ----------
        self.robot = robot
        self.name = self.robot.getName()
        self.team = self.name[0]
        self.player_id = int(self.name[1])

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float('+inf'))
        self.right_motor.setPosition(float('+inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # ---------- classes ----------
        self.ball_LOP_checker = LOP_Checker(round(6/(TIME_STEP/1000)), 0.5)

        # ---------- constants ----------

        # simulation
        self.TEAMMATES = []
        for r in ROBOT_NAMES:
            if r[0].upper() != self.team.upper():
                continue
            if r.upper() == self.name.upper():
                continue
            self.TEAMMATES.append(r)

        # motors
        self.MAX_VELOCITY = self.left_motor.getMaxVelocity()

        self.MOTOR_ACCEL = math.pi * 30
        self.left_motor.setAcceleration(self.MOTOR_ACCEL)
        self.right_motor.setAcceleration(self.MOTOR_ACCEL)

        self.left_motor.enableTorqueFeedback(TIME_STEP)
        self.right_motor.enableTorqueFeedback(TIME_STEP)

        self.REVERSE_DIR = False
        self.REVERSE_MOTORS_DIR = True

        # ---------- variables ----------

        self.last_robot_pos = None
        self.sleep_counter = 0

        self.last_ball_pos = None

        self.ROBOT_IN_GOAL = False

        self.scored = False

        self.ball_reset = False
        self.kick_to_side = False

        # timestep
        self.timestep = -1

        # ---------- team based constants/variables ----------

        if self.team.upper() == 'B':
            self.FLIP_X_AXIS = True
            self.FLIP_Y_AXIS = True

            self.OPP_GOAL_POS = [-0.8, 0]
            self.OWN_GOAL_POS = [0.8, 0]

            self.DEFENCE_LINE = [0.4, 0.4]

            self.ATTACK_CENTER_POS = [-0.625, 0.1]
            self.ATTACK_SIDE_POS = [0.175, 0.5]
            self.ATTACK_LINE = -0.25

            self.NS_WAIT_POS = [0.075, 0]

            self.GOALIE_SIDE_POS = [0.7, 0.2]
            self.GOALIE_CENTER_POS = [0.7, 0]
            self.BACK_LINE = 0.25

        else:
            self.FLIP_X_AXIS = False
            self.FLIP_Y_AXIS = False

            self.OPP_GOAL_POS = [0.8, 0]
            self.OWN_GOAL_POS = [-0.8, 0]

            self.DEFENCE_LINE = [-0.4, 0.4]

            self.ATTACK_CENTER_POS = [0.625, 0.1]
            self.ATTACK_SIDE_POS = [-0.175, 0.5]
            self.ATTACK_LINE = 0.25

            self.NS_WAIT_POS = [-0.075, 0]

            self.GOALIE_SIDE_POS = [-0.7, 0.2]
            self.GOALIE_CENTER_POS = [-0.7, 0]

            self.BACK_LINE = -0.25

    ''' ------------------------------ Receiver ------------------------------ '''


    ''' ------------------------------ Motor ------------------------------ '''

    def set_speeds(self, left_spd, right_spd):
        if left_spd > 100: left_spd = 100
        if left_spd < -100: left_spd = -100
        if right_spd > 100: right_spd = 100
        if right_spd < -100: right_spd = -100

        l = left_spd * self.MAX_VELOCITY / 100
        r = right_spd * self.MAX_VELOCITY / 100

        if self.REVERSE_MOTORS_DIR:
            l, r = -l, -r
        if self.REVERSE_DIR:
            l, r = -r, -l
        self.left_motor.setVelocity(l)
        self.right_motor.setVelocity(r)

    def stop(self):
        self.set_speeds(0,0)

    ''' ------------------------------ Other ------------------------------ '''

    def get_angle_to(self, x1, y1, x0, y0):
        diff = [0,0]
        diff[0] = y1 - y0
        diff[1] = x1 - x0
        if self.FLIP_Y_AXIS: diff[0] = -diff[0]
        if self.FLIP_X_AXIS: diff[1] = -diff[1]
        if diff[1] == 0: diff[1] = 0.0000000001
        ang = abs(math.degrees(math.atan(diff[0]/diff[1])))
        if diff[0] > 0 and diff[1] > 0: ang = 360 - ang
        if diff[0] < 0 and diff[1] < 0: ang = 180 - ang
        if diff[0] > 0 and diff[1] < 0: ang = 180 + ang
        return ang

    def get_dist_to(self, x1, y1, x0, y0):
        diff = [0,0]
        diff[0] = y1 - y0
        diff[1] = x1 - x0
        dist = math.sqrt(diff[0]**2 + diff[1]**2)
        return dist

    def get_angle_and_dist_to(self, x1, y1, x0, y0):
        ang = self.get_angle_to(x1, y1, x0, y0)
        dist = self.get_dist_to(x1, y1, x0, y0)
        return ang, dist

    def check_ball_in_corner(self, px, py):
        px = abs(px)
        py = abs(py)

        ay = BALL_Y_CORNER
        bx = BALL_X_WALL
        by = BALL_Y_WALL
        cx = BALL_X_CORNER

        if px < cx or py < ay:
            return False

        tri_ABC = CORNER_SIDE**2/2
        tri_PBA = (bx-px)*CORNER_SIDE/2
        tri_PBC = (by-py)*CORNER_SIDE/2

        if tri_ABC > tri_PBA + tri_PBC:
            return True
        else:
            return False

    def predict_ball(self, x1, y1, x0, y0, t):
        if t < 1:
            return [x1, y1]

        xt = x1
        yt = y1
        xt_1 = x0
        yt_1 = y0

        for _ in range(t):
            x_delta = xt - xt_1
            y_delta = yt - yt_1

            x_pred = xt+x_delta
            y_pred = yt+y_delta

            if self.check_ball_in_corner(x_pred, y_pred):
                xt_1 = xt
                yt_1 = yt

                xd = y_delta*BALL_DECEL
                yd = x_delta*BALL_DECEL

                if abs(x_delta) > abs(y_delta):
                    xd = -xd

                    if yt<0:
                        yd = abs(yd)
                    else:
                        yd = -abs(yd)

                else:
                    yd = -yd

                    if xt<0:
                        xd = abs(xd)
                    else:
                        xd = -abs(xd)

                xt += xd
                yt += yd

                continue

            if abs(x_pred) > BALL_X_WALL:
                if xt > 0:
                    xt_1 = BALL_X_WALL
                    x_dif = abs(BALL_X_WALL - x_pred)*BALL_DECEL
                    xt = BALL_X_WALL - x_dif
                else:
                    xt_1 = -BALL_X_WALL
                    x_dif = abs(-BALL_X_WALL - x_pred)*BALL_DECEL
                    xt = -BALL_X_WALL + x_dif

                yt_1 = yt
                bounce = y_delta*BALL_DECEL
                yt += bounce

                continue

            if abs(y_pred) > BALL_Y_WALL:
                if yt > 0:
                    yt_1 = BALL_Y_WALL
                    y_dif = abs(BALL_Y_WALL - y_pred)*BALL_DECEL
                    yt = BALL_Y_WALL - y_dif
                else:
                    yt_1 = -BALL_Y_WALL
                    y_dif = abs(-BALL_Y_WALL - y_pred)*BALL_DECEL
                    yt = -BALL_Y_WALL + y_dif

                xt_1 = xt
                bounce = x_delta*BALL_DECEL
                xt += bounce

                continue

            xt_1 = xt
            yt_1 = yt

            xt = x_pred
            yt = y_pred

        return [xt, yt]

    def check_scored(self):
        ball_pos = self.data['ball']

        if self.scored:
            self.stop()
            if abs(ball_pos['x']) < 0.01 and abs(ball_pos['y']) < 0.01:
                self.scored = False
            else:
                return True

        if ball_pos['x'] > GOAL_X_LIMIT or ball_pos['x'] < -GOAL_X_LIMIT:
            self.stop()
            self.scored = True
            self.last_robot_pos = None
            self.last_ball_pos = None
            self.REVERSE_DIR = False
            self.ball_LOP_checker.reset()
            return True
        return False

    def check_sudden_movement(self):
        robot_pos = self.data[self.name.upper()]
        if self.sleep_counter > 0:
            self.sleep_counter -= 1
            return True

        d = 0
        if self.last_robot_pos != None:
            d = math.sqrt((robot_pos['x']-self.last_robot_pos[0])**2 + (robot_pos['y']-self.last_robot_pos[1])**2)
        self.last_robot_pos = [robot_pos['x'], robot_pos['y']]
        if d > 0.02:
            self.sleep_counter = 3
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            return True
        return False

    def check_ball_reset(self):
        ball_pos = self.data['ball']

        if self.last_ball_pos == None:
            self.last_ball_pos = [ball_pos['x'], ball_pos['y']]

        d = self.get_dist_to(ball_pos['x'], ball_pos['y'], self.last_ball_pos[0], self.last_ball_pos[1])

        self.ball_reset = False
        if d > 0.1:
            self.last_ball_pos = [ball_pos['x'], ball_pos['y']]
            self.ball_LOP_checker.reset()
            self.ball_reset = True

    def get_rel_pos(self):
        ball_pos = self.data['ball']
        dist_to_ball = {}
        for r in ROBOT_NAMES:
            if r[0].upper() != self.team.upper():
                continue
            robot_pos = self.data[r.upper()]
            ball_dist = self.get_dist_to(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y'])
            dist_to_ball[r] = ball_dist

        if dist_to_ball[self.name.upper()] < dist_to_ball[self.TEAMMATES[0]] and dist_to_ball[self.name.upper()] < dist_to_ball[self.TEAMMATES[1]]:
            return REL_POS_CLOSE
        elif dist_to_ball[self.name.upper()] > dist_to_ball[self.TEAMMATES[0]] and dist_to_ball[self.name.upper()] > dist_to_ball[self.TEAMMATES[1]]:
            return REL_POS_FAR
        else:
            return REL_POS_MID

    def get_role(self, ball_LOP):
        ball_pos = self.data['ball']
        rel_pos = self.get_rel_pos()

        if rel_pos == REL_POS_CLOSE:
            role = ROLE_FOLLOW

            if self.ball_reset:
                robot_pos = self.data[self.name.upper()]
                ball_dist = self.get_dist_to(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y'])

                dist_to_ball = {}
                if ball_dist < NEAR_NS_DIST:
                    for r in ROBOT_NAMES:
                        if r[0].upper() == self.team.upper():
                            continue
                        robot_pos = self.data[r.upper()]
                        ball_dist = self.get_dist_to(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y'])
                        dist_to_ball[r] = ball_dist
                    sort = sorted(dist_to_ball, key=dist_to_ball.get)

                    robot_pos = self.data[sort[0].upper()]
                    robot_goal_dist = self.get_dist_to(self.OPP_GOAL_POS[0], self.OPP_GOAL_POS[1], robot_pos['x'], robot_pos['y'])
                    ball_goal_dist = self.get_dist_to(self.OPP_GOAL_POS[0], self.OPP_GOAL_POS[1], ball_pos['x'], ball_pos['y'])
                    if dist_to_ball[sort[0]] < NEAR_NS_DIST and robot_goal_dist < ball_goal_dist:
                        self.kick_to_side = True

        else:
            if (self.team.upper() == 'B' and ball_pos['x'] < self.ATTACK_LINE) or (self.team.upper() == 'Y' and ball_pos['x'] > self.ATTACK_LINE):
                dist_to_ball = {}
                for r in ROBOT_NAMES:
                    if r[0].upper() != self.team.upper():
                        continue
                    robot_pos = self.data[r.upper()]
                    ball_dist = self.get_dist_to(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y'])
                    dist_to_ball[r] = ball_dist

                sort = sorted(dist_to_ball, key=dist_to_ball.get)
                if sort[0] == self.TEAMMATES[0]:
                    teammate_far = self.TEAMMATES[1]
                else:
                    teammate_far = self.TEAMMATES[0]

                if ball_LOP:
                    tar_pos = self.NS_WAIT_POS.copy()
                else:
                    tar_pos = self.ATTACK_CENTER_POS.copy()
                    if ball_pos['y'] > 0:
                        tar_pos[1] = -tar_pos[1]
                dist_to_tar = {}
                for r in ROBOT_NAMES:
                    if r[0].upper() != self.team.upper():
                        continue
                    robot_pos = self.data[r.upper()]
                    tar_dist = self.get_dist_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])
                    dist_to_tar[r] = tar_dist

                if ball_LOP:
                    if dist_to_tar[self.name.upper()] < dist_to_tar[teammate_far]:
                        role = ROLE_NS_WAIT
                    else:
                        role = ROLE_ATTACK_SIDE
                else:
                    if dist_to_tar[self.name.upper()] < dist_to_tar[teammate_far]:
                        role = ROLE_ATTACK_CENTER
                    else:
                        role = ROLE_DEFENCE

            elif (self.team.upper() == 'B' and ball_pos['x'] > self.BACK_LINE) or (self.team.upper() == 'Y' and ball_pos['x'] < self.BACK_LINE):
                if rel_pos == REL_POS_MID:
                    role = ROLE_GOALIE_SIDE
                else:
                    if ball_LOP:
                        role = ROLE_NS_WAIT
                    else:
                        role = ROLE_GOALIE_CENTER

            else:
                if rel_pos == REL_POS_MID:
                    role = ROLE_FOLLOW
                else:
                    if ball_LOP:
                        role = ROLE_NS_WAIT
                    else:
                        role = ROLE_DEFENCE

        return role

    ''' ------------------------------ Main ------------------------------ '''

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.timestep += 1
            ''' supervisor code start '''
            if self.is_new_data():
                # receive data from supervisor
                self.data = self.get_new_data()
                ''' supervisor code end '''

                if self.check_scored() or self.check_sudden_movement():
                    continue

                self.check_ball_reset()

                # get basic info
                robot_pos = self.data[self.name.upper()]
                ball_pos = self.data['ball']
                ball_LOP = self.ball_LOP_checker.check_LOP([ball_pos['x'], ball_pos['y']])

                # calculate robot angle
                robot_ang_rad = robot_pos['orientation']
                robot_ang = math.degrees(robot_ang_rad)
                if self.team.upper() == 'B':
                    robot_ang += 90
                else:
                    robot_ang -= 90
                robot_ang %= 360

                if self.REVERSE_DIR:
                    robot_ang -= 180
                    robot_ang %= 360

                ball_dist = self.get_dist_to(ball_pos['x'], ball_pos['y'], robot_pos['x'], robot_pos['y'])

                pred_timestep = round(ball_dist * BALL_PREDICT_P_DIST)

                ball_pos_pred = self.predict_ball(ball_pos['x'], ball_pos['y'], self.last_ball_pos[0], self.last_ball_pos[1], pred_timestep)
                self.last_ball_pos = [ball_pos['x'], ball_pos['y']]

                ball_ang_pred, ball_dist_pred = self.get_angle_and_dist_to(ball_pos_pred[0], ball_pos_pred[1], robot_pos['x'], robot_pos['y'])

                ball_ang_diff = ball_ang_pred - robot_ang
                ball_ang_diff %= 360

                role = self.get_role(ball_LOP)

                if self.kick_to_side and ball_dist_pred > NEAR_NS_DIST:
                        self.kick_to_side = False

                goal_ang = self.get_angle_to(self.OPP_GOAL_POS[0], self.OPP_GOAL_POS[1], robot_pos['x'], robot_pos['y'])
                goal_ang_diff = goal_ang - robot_ang
                goal_ang_diff %= 360

                tar_pos = ball_pos_pred.copy()

                if role == ROLE_FOLLOW:
                    if (robot_pos['y'] < 0 and ball_pos_pred[1] < 0) or (robot_pos['y'] > 0 and ball_pos_pred[1] > 0):
                        ball_same_side = True
                    else:
                        ball_same_side = False

                    ball_goal_dist = self.get_dist_to(self.OWN_GOAL_POS[0], self.OWN_GOAL_POS[1], ball_pos_pred[0], ball_pos_pred[1])
                    robot_goal_dist = self.get_dist_to(self.OWN_GOAL_POS[0], self.OWN_GOAL_POS[1], robot_pos['x'], robot_pos['y'])

                    if ((self.team.upper() == 'B' and robot_pos['x'] > ROBOT_NEAR_WALL and ball_pos_pred[0] > ROBOT_NEAR_WALL) or (self.team.upper() == 'Y' and robot_pos['x'] < -ROBOT_NEAR_WALL and ball_pos_pred[0] < -ROBOT_NEAR_WALL)) and ball_same_side and ball_goal_dist < robot_goal_dist:
                        state = STATE_AWAY

                    elif ball_dist_pred < NEAR_BALL_DIST and (ball_ang_diff < NEAR_BALL_ANG or ball_ang_diff > (360-NEAR_BALL_ANG)):
                        state = STATE_NEAR

                    elif ball_dist_pred > TURN_DIST:
                            state = STATE_BEHIND

                    else:
                        state = STATE_BALL

                elif role == ROLE_DEFENCE:
                    tar_pos = [self.DEFENCE_LINE[0], ball_pos_pred[1]]
                    if tar_pos[1] > self.DEFENCE_LINE[1]: tar_pos[1] = self.DEFENCE_LINE[1]
                    if tar_pos[1] < -self.DEFENCE_LINE[1]: tar_pos[1] = -self.DEFENCE_LINE[1]
                    tar_dist = self.get_dist_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])

                    if ball_dist_pred < NEAR_BALL_DIST and (ball_ang_diff < NEAR_BALL_ANG or ball_ang_diff > (360-NEAR_BALL_ANG)):
                        state = STATE_NEAR

                    elif (self.team.upper() == 'B' and ball_pos_pred[0] > 0) or (self.team.upper() == 'Y' and ball_pos_pred[0] < 0):
                        state = STATE_DEFEND

                    elif tar_dist < STOP_WITHIN:
                        state = STATE_STOP

                    else:
                        state = STATE_WAIT

                elif role == ROLE_ATTACK_CENTER or role == ROLE_ATTACK_SIDE:
                    if role == ROLE_ATTACK_CENTER:
                        tar_pos = self.ATTACK_CENTER_POS.copy()
                    else:
                        tar_pos = self.ATTACK_SIDE_POS.copy()
                    if ball_pos_pred[1] > 0:
                        tar_pos[1] = -tar_pos[1]

                    tar_dist = self.get_dist_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])

                    if ball_dist_pred < NEAR_BALL_DIST and (ball_ang_diff < NEAR_BALL_ANG or ball_ang_diff > (360-NEAR_BALL_ANG)):
                        state = STATE_NEAR

                    elif abs(ball_pos_pred[0]) > abs(self.ATTACK_CENTER_POS[0]) and abs(ball_pos_pred[1]) < abs(self.ATTACK_CENTER_POS[1]):
                        state = STATE_ATTACK

                    elif tar_dist < STOP_WITHIN:
                        if (self.team.upper() == 'B' and ball_pos_pred[1] < robot_pos['y']) or (self.team.upper() == 'Y' and ball_pos_pred[1] > robot_pos['y']):
                            goal_ang = 360-30
                        else:
                            goal_ang = 30
                        goal_ang_diff = goal_ang - robot_ang
                        goal_ang_diff %= 360
                        if 5<goal_ang_diff<(360-5):
                            state = STATE_TURN_TO_GOAL
                        else:
                            state = STATE_STOP

                    else:
                        state = STATE_WAIT

                elif role == ROLE_NS_WAIT:
                    tar_pos = self.NS_WAIT_POS.copy()
                    tar_dist = self.get_dist_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])

                    if tar_dist < STOP_WITHIN:
                        goal_ang = self.get_angle_to(self.OPP_GOAL_POS[0], self.OPP_GOAL_POS[1], robot_pos['x'], robot_pos['y'])
                        goal_ang_diff = goal_ang - robot_ang
                        goal_ang_diff %= 360
                        if 5<goal_ang_diff<(360-5):
                            state = STATE_TURN_TO_GOAL
                        else:
                            state = STATE_STOP

                    else:
                        state = STATE_WAIT

                elif role == ROLE_GOALIE_SIDE or role == ROLE_GOALIE_CENTER:
                    if role == ROLE_GOALIE_SIDE:
                        tar_pos = self.GOALIE_SIDE_POS.copy()
                        defend_dist = DEFEND_BALL_DIST_SIDE
                    else:
                        tar_pos = self.GOALIE_CENTER_POS.copy()
                        defend_dist = DEFEND_BALL_DIST_CENTER
                    if ball_pos_pred[1] < 0:
                        tar_pos[1] = -tar_pos[1]
                    tar_dist = self.get_dist_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])

                    ball_goal_dist = self.get_dist_to(self.OWN_GOAL_POS[0], self.OWN_GOAL_POS[1], ball_pos_pred[0], ball_pos_pred[1])

                    if (robot_pos['y'] < 0 and ball_pos_pred[1] < 0) or (robot_pos['y'] > 0 and ball_pos_pred[1] > 0):
                        ball_same_side = True
                    else:
                        ball_same_side = False

                    ball_goal_dist = self.get_dist_to(self.OWN_GOAL_POS[0], self.OWN_GOAL_POS[1], ball_pos_pred[0], ball_pos_pred[1])
                    robot_goal_dist = self.get_dist_to(self.OWN_GOAL_POS[0], self.OWN_GOAL_POS[1], robot_pos['x'], robot_pos['y'])

                    if ((self.team.upper() == 'B' and robot_pos['x'] > ROBOT_NEAR_WALL and ball_pos_pred[0] > ROBOT_NEAR_WALL) or (self.team.upper() == 'Y' and robot_pos['x'] < -ROBOT_NEAR_WALL and ball_pos_pred[0] < -ROBOT_NEAR_WALL)) and ball_same_side and ball_goal_dist < robot_goal_dist:
                        state = STATE_AWAY

                    elif ball_dist_pred < NEAR_BALL_DIST and (ball_ang_diff < NEAR_BALL_ANG or ball_ang_diff > (360-NEAR_BALL_ANG)):
                        state = STATE_NEAR

                    elif ball_goal_dist < defend_dist:
                        state = STATE_DEFEND

                    elif tar_dist < STOP_WITHIN:
                        state = STATE_STOP

                    else:
                        state = STATE_WAIT

                else:
                    state = STATE_STOP
                    tar_ang = 0

                spds = [100,100]
                min_spd = -100
                P = SPD_P_NORMAL

                if state == STATE_NEAR:
                    tar_pos = self.OPP_GOAL_POS.copy()
                    if self.kick_to_side:
                        tar_pos[1] = 0.65
                        sum = 0
                        for r in ROBOT_NAMES:
                            if r[0].upper() == self.team.upper():
                                continue
                            r_pos = self.data[r.upper()]
                            sum += r_pos['y']
                        average = sum / 3
                        if average > 0:
                            tar_pos[1] = -tar_pos[1]

                    if 90 < robot_ang < 270:
                        tar_pos[1] = robot_pos['y']

                    tar_ang = self.get_angle_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])
                    tar_ang -= robot_ang
                    tar_ang %= 360

                    P = SPD_P_NEAR
                    min_spd = NEAR_BALL_MIN_SPD

                elif state == STATE_BEHIND:
                    ball_pos_behind = ball_pos_pred
                    d = BEHIND_BALL_DIST
                    if self.FLIP_X_AXIS: d = -d
                    ball_pos_behind[0] -= d
                    if ball_pos_behind[0] > ROBOT_X_WALL: ball_pos_behind[0] = ROBOT_X_WALL
                    if ball_pos_behind[0] < -ROBOT_X_WALL: ball_pos_behind[0] = -ROBOT_X_WALL
                    if ball_pos_behind[1] > ROBOT_Y_WALL: ball_pos_behind[1] = ROBOT_Y_WALL
                    if ball_pos_behind[1] < -ROBOT_Y_WALL: ball_pos_behind[1] = -ROBOT_Y_WALL
                    ball_behind_ang = self.get_angle_to(ball_pos_behind[0], ball_pos_behind[1], robot_pos['x'], robot_pos['y'])
                    tar_ang = ball_behind_ang - robot_ang
                    tar_ang %= 360

                elif state == STATE_BALL:
                    tar_ang = ball_ang_pred - robot_ang
                    tar_ang %= 360

                elif state == STATE_AWAY:
                    tar_pos = [0.75, 0.65]

                    if robot_pos['x'] < 0:
                        tar_pos[0] = -tar_pos[0]
                    if robot_pos['y'] < 0:
                        tar_pos[1] = -tar_pos[1]

                    away_ball_ang = self.get_angle_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])
                    tar_ang = away_ball_ang - robot_ang
                    tar_ang %= 360

                elif state == STATE_DEFEND:
                    tar_ang = ball_ang_pred - robot_ang
                    tar_ang %= 360

                    P = SPD_P_DEFEND

                elif state == STATE_ATTACK:
                    tar_ang = ball_ang_pred - robot_ang
                    tar_ang %= 360

                    P = SPD_P_ATTACK

                elif state == STATE_STOP:
                    tar_ang = 0

                    spds = [0,0]

                elif state == STATE_TURN_TO_GOAL:
                    tar_ang = 0

                    if 90 < goal_ang_diff < 270:
                        self.REVERSE_DIR = not self.REVERSE_DIR
                        goal_ang_diff -= 180
                        goal_ang_diff %= 360

                    if 30<goal_ang_diff<(360-30):
                        spd = 100
                    else:
                        spd = 20
                    if goal_ang_diff < 180:
                        spds = [-spd, spd]
                    else:
                        spds = [spd, -spd]

                elif state == STATE_WAIT:
                    tar_ang = self.get_angle_to(tar_pos[0], tar_pos[1], robot_pos['x'], robot_pos['y'])
                    tar_ang -= robot_ang
                    tar_ang %= 360

                else:
                    tar_ang = 0
                    spds = [0,0]

                if 90 < tar_ang < 270 and state != STATE_NEAR and not self.ROBOT_IN_GOAL:
                    self.REVERSE_DIR = not self.REVERSE_DIR
                    tar_ang -= 180
                    tar_ang %= 360

                if ball_dist_pred > BALL_AWAY_DIST and abs(robot_pos['x']) > GOAL_X_ROBOT:
                    if not self.ROBOT_IN_GOAL:
                        self.REVERSE_DIR = not self.REVERSE_DIR
                        tar_ang -= 180
                        tar_ang %= 360
                        self.ROBOT_IN_GOAL = True
                else:
                    self.ROBOT_IN_GOAL = False

                if tar_ang < 180:
                    error = tar_ang * P
                    spds[0] -= error
                else:
                    error = (360-tar_ang) * P
                    spds[1] -= error

                if spds[0] < min_spd: spds[0] = min_spd
                if spds[1] < min_spd: spds[1] = min_spd

                self.set_speeds(spds[0], spds[1])

            else:
                pass
