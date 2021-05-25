import math
from typing import Tuple

# Proportional Factor to reach the ball faster
P_FACTOR = 40;
OP_GOAL = [-0.856, 0]  # Opponent's goal position [x, y] (for Blue team)
OWN_GOAL = [0.856, 0]  # Own's goal position [x, y] (for Blue team)
MINIMUM_SPEED = 6
AMP_CURVE = 10
OP_GOAL_ANGLE = 270
OP_GOAL_ANGLE_Y = 90


def get_direction2point(robot_pos, robot_angle, point) -> int:
    # Get the angle between the robot and the ball
    angle = math.atan2(
        point[1] - robot_pos['y'],
        point[0] - robot_pos['x'],
    )

    if angle < 0:
        angle = 2 * math.pi + angle

    if robot_angle < 0:
        robot_angle = 2 * math.pi + robot_angle

    robot_point_angle = math.degrees(angle + robot_angle)

    # Axis Z is forward
    robot_point_angle -= 90
    if robot_point_angle > 360:
        robot_point_angle -= 360

    # print('robot_point_angle = ' + str(robot_point_angle))
    direction = get_direction(robot_point_angle) * -1  # inverte para ficar igual minha logica

    return direction


def get_angle_Goal2Robot(robot_pos, robot_angle, goal_point, own_goal_point, side) -> Tuple[int, float, float]:
    # Returns, in degrees, the angle between the front of object_1 and point which dist2point is calculated
    # robot_pos = robot object
    # robot_angle = robot angle
    # goal_point = desired goal point

    # returns value = -1: indicates to rotate counterclockwise
    #         value = 1: indicates to rotate clockwise
    #         value = 0: indicates to go foward

    # Convert robot_angle (RAD -> DEG)
    robot_angle = math.degrees(robot_angle)

    # Calculates phi angle
    phi = math.atan(abs((goal_point[0] - robot_pos['x']) / robot_pos['y']))
    own_phi = math.atan(abs((own_goal_point[0] - robot_pos['x']) / robot_pos['y']))

    # Choose correctly which equation to use
    if robot_pos['y'] < 0:
        angle2goal = math.degrees(2 * math.pi - phi)
        angle2owngoal = math.degrees(own_phi)
    else:
        angle2goal = math.degrees(math.pi + phi)
        angle2owngoal = math.degrees(math.pi - own_phi)

    # Calculates the angular lag
    d_angle = angle2goal - robot_angle
    d_angle_own = angle2owngoal - robot_angle

    if (robot_angle > 180):  # if it's in the attack side
        # Verifying conditions
        if abs(d_angle) < 10 and side == 1:  # Means that the robot is looking straight forward to desired goal
            # Kick is authorized
            value = 0
        else:
            if d_angle > 0:
                # Counterclockwise
                value = -1
            else:
                # Clockwise
                value = 1
    else:  # If it's in the defense side
        if abs(d_angle_own) < 10 and side == -1:
            value = 0
        else:
            if d_angle_own < 0:
                value = -1
            else:
                value = 1

    return value * side, d_angle, d_angle_own


def theChoosenOne(self, data) -> bool:
    if self.name[0] == 'Y':
        MY_ROBOT_NAME = ["Y1", "Y2", "Y3"]
    else:
        MY_ROBOT_NAME = ["B1", "B2", "B3"]

    robot_name = self.name

    vector = [100, 100, 100]
    i = 0
    for name in MY_ROBOT_NAME:
        robot_pos = data[name]
        ball_pos = data['ball']
        vector[i] = get_distBetweenObjects(robot_pos, ball_pos)
        i += 1
    # print(vector)

    min_value = min(vector)
    min_arg = -1
    for value_i in range(3):
        if min_value == vector[value_i]:
            min_arg = value_i
            break
    # print(min_value)
    # print(min_arg)
    # print('--------')

    return True if MY_ROBOT_NAME[min_arg] == robot_name else False


def theChoosenGoalKeeper(self, data) -> bool:
    if self.name[0] == 'Y':
        MY_ROBOT_NAME = ["Y1", "Y2", "Y3"]
        OWN_GOAL = [-0.856, 0]
    else:
        MY_ROBOT_NAME = ["B1", "B2", "B3"]
        OWN_GOAL = [0.856, 0]

    vector = [100, 100, 100]
    i = 0
    for name in MY_ROBOT_NAME:
        robot_pos = data[name]
        ball_pos = data['ball']
        vector[i] = get_dist_Object2Point(robot_pos, OWN_GOAL)
        i += 1

    min_value = min(vector)
    min_arg = -1
    for value_i in range(3):
        if min_value == vector[value_i]:
            min_arg = value_i
            break

    return True if MY_ROBOT_NAME[min_arg] == self.name else False


def artilheiro(self, data, STOP_NEAR_GOALKEEPER, KICK_FLAG, kick_intensity) -> int:
    if self.name[0] == 'Y':
        MY_ROBOT_NAMES = ["Y1", "Y2", "Y3"]
        OP_GOAL = [0.856, 0]  # Opponent's goal position [x, y]
        OWN_GOAL = [-0.856, 0]  # Own's goal position [x, y]
        INVERT_SIDE = -1
    else:
        MY_ROBOT_NAMES = ["B1", "B2", "B3"]
        OP_GOAL = [-0.856, 0]  # Opponent's goal position [x, y]
        OWN_GOAL = [0.856, 0]  # Own's goal position [x, y]
        INVERT_SIDE = 1

    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)
    # Get distance between current robot and ball
    dist_ball2robot = get_distBetweenObjects(ball_pos, robot_pos)
    # Set Proportional Factor to control the speed proportional
    # to the distance between the robot and the ball
    proportional_speed = -(MINIMUM_SPEED + P_FACTOR * dist_ball2robot)
    # Convert the robot angle to degree
    robot_angle_deg = math.degrees(robot_angle)
    # Compute the direction info to the robot
    direction = get_direction(ball_angle)

    dist1 = get_dist_Object2Point(data[MY_ROBOT_NAMES[0]], OWN_GOAL)
    dist2 = get_dist_Object2Point(data[MY_ROBOT_NAMES[1]], OWN_GOAL)
    dist3 = get_dist_Object2Point(data[MY_ROBOT_NAMES[2]], OWN_GOAL)

    vt = [dist1, dist2, dist3]
    minValue = min(vt)

    min_arg = -1

    for value_i in range(3):
        if minValue == vt[value_i]:
            min_arg = value_i
            break

    dist2goleiro = get_distBetweenObjects(robot_pos,
                                          data[MY_ROBOT_NAMES[min_arg]])  # distancia entre o artilheiro e o goleiro
    dist_ball2goleiro = get_distBetweenObjects(data['ball'],
                                               data[MY_ROBOT_NAMES[min_arg]])  # distancia entre a bola e o goleiro

    print("A DISTANCIA 2 GOLEIRO: " + str(dist2goleiro))

    if dist2goleiro < 0.2 and dist_ball2goleiro < 0.1 and STOP_NEAR_GOALKEEPER:
        left_speed = 0
        right_speed = 0
        # print("Artilheiro PARADO!!!!")
    else:
        # print(dist_ball2robot)
        CLOSE_BALL = False
        # If the ball is near enough to the robot
        if dist_ball2robot <= 0.065:
            if INVERT_SIDE == -1:
                # Get the info about rotation kick and the angle between robot's front and opponent's goal
                kick_rotation, d_angle, d_angle_own = get_angle_Goal2Robot(robot_pos, robot_angle, OWN_GOAL, OP_GOAL,
                                                                           INVERT_SIDE)
            else:
                # Get the info about rotation kick and the angle between robot's front and opponent's goal
                kick_rotation, d_angle, d_angle_own = get_angle_Goal2Robot(robot_pos, robot_angle, OP_GOAL, OWN_GOAL,
                                                                           INVERT_SIDE)
            if KICK_FLAG == 0:
                if kick_rotation == 0:
                    print(self.name + ' CHUTE PARA O GOL!')
                    left_speed = -10
                    right_speed = -10

                elif kick_rotation == -1:
                    # Counterclockwise
                    left_speed = 2
                    right_speed = -10
                    KICK_FLAG = -1
                else:
                    # print('horario')
                    # Clockwise
                    left_speed = -10
                    right_speed = 2
                    KICK_FLAG = 1
        else:  # Se está longe da bola
            change_rotation = True if dist_ball2robot > 0.10 else False
            # Se direction for ze0ro, anda pra frente
            if direction == 0:
                left_speed = proportional_speed if proportional_speed >= -10 else -10
                right_speed = left_speed

            elif direction == -1:
                left_speed = -6 if change_rotation else -10
                right_speed = 6 if change_rotation else -3

            else:
                left_speed = 6 if change_rotation else -3
                right_speed = -6 if change_rotation else -10
    if KICK_FLAG < 0:
        left_speed = 0
        right_speed = -10
        KICK_FLAG -= 1
    elif KICK_FLAG > 0:
        left_speed = -10
        right_speed = 0
        KICK_FLAG += 1
    if KICK_FLAG == kick_intensity + 1 or KICK_FLAG == -kick_intensity - 1:
        KICK_FLAG = 0
        # Set the speed to motors
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)

    return KICK_FLAG


def goleiro(self, data):
    dir = 0
    DistBall = 0
    AnguloABS = 0
    AnguloRobot = 0
    FechaBola = 0
    lado = 0

    robot_pos = data[self.name]

    if robot_pos['x'] < 0:
        lado = -1
    else:
        lado = 1

    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']

    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

    # Compute the speed for motors
    direction = get_direction(ball_angle)

    # If the robot has the ball right in front of it, go forward,
    # rotate otherwise

    # calculo distancia do robo para a bola
    DistBall = math.sqrt((robot_pos['x'] - ball_pos['x']) ** 2 + (robot_pos['y'] - ball_pos['y']) ** 2)

    # Angulo absoluto da loba em relação ao robo
    if ball_angle < 0:
        AnguloABS = 360 + ball_angle
    else:
        AnguloABS = ball_angle

    if AnguloABS > 180:
        AnguloABS = AnguloABS - 360
    # converção da orientação de radiandos para graus
    AnguloRobot = math.degrees(robot_pos['orientation'])

    if ball_pos['x'] * lado > 0.5:

        if robot_pos['y'] > ball_pos['y']:

            if AnguloABS > 0:
                G1 = (AnguloABS - 180) / 10
            else:
                G1 = (AnguloABS + 180) / 10
            left_speed = 8 - G1
            right_speed = 8 + G1

        else:
            G1 = AnguloABS / 10
            left_speed = -8 - G1
            right_speed = -8 + G1

    elif (robot_pos['x'] * lado > 0.48 and robot_pos['x'] * lado < 0.55) or DistBall < 0.15:
        FechaBola = 0
        if (AnguloABS * lado >= 85 and AnguloABS * lado <= 95):
            if dir == 0:
                left_speed = -5
                right_speed = -5
            else:
                left_speed = 5
                right_speed = 5
        else:
            if AnguloABS * lado > 90:
                left_speed = 8 - AnguloRobot / 2
                right_speed = 8 + AnguloRobot / 2
                dir = 1
            else:
                left_speed = -8 - AnguloRobot / 2
                right_speed = -8 + AnguloRobot / 2
                dir = 0
    else:
        if robot_pos['x'] * lado > 0.55:
            dir = -1
        else:
            dir = 1
        if robot_pos['y'] * lado < 0.05 and robot_pos['y'] * lado > -0.05:
            alvo = -90 * lado
            left_speed = dir * 8 - (AnguloRobot - alvo) / 2
            right_speed = dir * 8 + (AnguloRobot - alvo) / 2
        elif robot_pos['y'] > 0:
            alvo = -50 * lado
            left_speed = dir * 8 - (AnguloRobot - alvo) / 2
            right_speed = dir * 8 + (AnguloRobot - alvo) / 2
        else:
            alvo = -140 * lado
            left_speed = dir * 8 - (AnguloRobot - alvo) / 2
            right_speed = dir * 8 + (AnguloRobot - alvo) / 2
            # posiciona em -0,9 e -1,1
    # Set the speed to motors
    if left_speed > 10:
        left_speed = 10
    if left_speed < -10:
        left_speed = -10
    if right_speed > 10:
        right_speed = 10
    if right_speed < -10:
        right_speed = -10

    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)


def lookForward(self, data) -> bool:
    if self.name[0] == 'Y':
        INVERT_SIDE = -1
    else:
        INVERT_SIDE = 1

    ball_angle, robot_angle = self.get_angles(data['ball'], data[self.name])
    robot_angle = math.degrees(robot_angle)
    FORWARD_FLAG = False
    ACEPTED_ERROR = 10
    if INVERT_SIDE == 1:
        if robot_angle < (OP_GOAL_ANGLE - ACEPTED_ERROR) and robot_angle > 90:
            # Anti-horario
            left_speed = 6
            right_speed = -6
        elif robot_angle > (OP_GOAL_ANGLE + ACEPTED_ERROR) or robot_angle < 90:
            # Horario
            left_speed = -6
            right_speed = 6
        else:
            # parado
            FORWARD_FLAG = True
            left_speed = 3
            right_speed = 3
    else:
        if robot_angle > (OP_GOAL_ANGLE_Y + ACEPTED_ERROR) and robot_angle < 270:
            # horario
            left_speed = -6
            right_speed = 6
        elif robot_angle < (OP_GOAL_ANGLE_Y - ACEPTED_ERROR) or robot_angle > 270:
            # Anti-Horario
            left_speed = 6
            right_speed = -6
        else:
            # parado
            FORWARD_FLAG = True
            left_speed = 3
            right_speed = 3

    # Set the speed to motors
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)

    return FORWARD_FLAG


def banheira(self, data):
    BANHEIRA_POINT_CENTER = [0.10, 0]

    if self.name[0] == 'Y':
        BANHEIRA_POINT_CENTER = [-0.10, 0]
    else:
        BANHEIRA_POINT_CENTER = [0.10, 0]

    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

    value = get_direction2point(robot_pos, robot_angle, BANHEIRA_POINT_CENTER)
    dist_robot2point = get_dist_Object2Point(robot_pos, BANHEIRA_POINT_CENTER)

    if dist_robot2point < 0.05:
        is_forward = lookForward(self, data)
    else:
        if value == 0:
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(-10)
        elif value == -1:
            # sentido anti-horario
            self.left_motor.setVelocity(2)
            self.right_motor.setVelocity(-2)
        else:  # value == 1
            # sentido horário
            self.left_motor.setVelocity(-2)
            self.right_motor.setVelocity(2)


def go_to_point(self, data, point, MIRROR_FLAG):
    if MIRROR_FLAG:
        if self.name[0] == 'Y':
            point[0] = - point[0]
            point[1] = - point[1]

    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

    value = get_direction2point(robot_pos, robot_angle, point)
    # value, d_angle = get_angle_point2robot(robot_pos, robot_angle, ball_pos, BANHEIRA_POINT_CENTER)
    dist_robot2point = get_dist_Object2Point(robot_pos, point)

    if dist_robot2point < 0.05:
        # Set the speed to motors
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return True
    else:
        if value == 0:
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(-10)
        elif value == -1:
            # sentido anti-horario
            self.left_motor.setVelocity(6)
            self.right_motor.setVelocity(-6)
        else:  # value == 1
            # sentido horário
            self.left_motor.setVelocity(-6)
            self.right_motor.setVelocity(6)
        return False


def follow_the_ball(self, data):
    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)
    # Compute the speed for motors
    direction = get_direction(ball_angle)
    # If the robot has the ball right in front of it, go forward,
    # rotate otherwise
    if direction == 0:
        left_speed = -10
        right_speed = -10
    else:
        left_speed = direction * 5
        right_speed = direction * -5
    # Set the speed to motors
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)


def face_the_ball(self, data):
    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)
    # Compute the speed for motors
    direction = get_direction(ball_angle)
    # If the robot has the ball right in front of it, go forward,
    # rotate otherwise
    if direction == 0:
        left_speed = 0
        right_speed = 0
    else:
        left_speed = direction * 5
        right_speed = direction * -5
    # Set the speed to motors
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)


def game_time(counter) -> Tuple[int, int]:
    counter += 1
    cross_rule = 10 * counter / 9373
    min = math.floor(cross_rule)  # tempo em segundos
    seg = math.floor((cross_rule - min) * 60)
    game_clock = [9 - min, 59 - seg]
    return game_clock, counter


def countGoal(ball_pos, score, GOAL_FLAG) -> Tuple[int, bool]:
    # vector[1] = blue team score; vector[0] = yellow team score
    gol_b = score[0]
    gol_y = score[1]
    if ball_pos > 0.743:
        gol_b += 1
        GOAL_FLAG = True
    elif ball_pos < -0.743:
        gol_y += 1
        GOAL_FLAG = True

    return [gol_b, gol_y], GOAL_FLAG


def scoreboard(data, SCORE, game_clock, goal_time, GOAL_FLAG, TIME_FLAG) -> Tuple[int, int, bool, bool]:
    clock_sec = game_clock[0] * 60 + game_clock[1]
    if GOAL_FLAG == False:
        SCORE, GOAL_FLAG = countGoal(data['ball']['x'], SCORE, GOAL_FLAG)
    else:
        if not TIME_FLAG:
            goal_time = clock_sec
            TIME_FLAG = True

        if clock_sec < goal_time - 4:  # se passaram-se 4 segundos após o gol
            GOAL_FLAG = False
            TIME_FLAG = False
            print(SCORE)

    return SCORE, goal_time, GOAL_FLAG, TIME_FLAG


def point_between_ball2mygoal(data, my_goal_point, radius, INVERT_SIDE) -> Tuple[float, float]:
    # Returns the desired point between the ball and "my_goal_point"
    # with "radius" away from my_goal_point
    # Also returns the angle to point "m"

    ball_pos = data['ball']
    m = math.atan((ball_pos['y'] - my_goal_point[1]) / (ball_pos['x'] - my_goal_point[0]))
    if INVERT_SIDE == 1:
        ax = my_goal_point[0] - radius * math.cos(m)
        ay = my_goal_point[1] - radius * math.sin(m)
    else:
        ax = my_goal_point[0] + radius * math.cos(m)
        ay = my_goal_point[1] + radius * math.sin(m)

    return [ax, ay], math.degrees(m)


def meu_goleiro(self, data, dist2mygoal):
    if self.name[0] == 'Y':
        OP_GOAL = [0.856, 0]  # Opponent's goal position [x, y]
        OWN_GOAL = [-0.856, 0]  # Own's goal position [x, y]
        INVERT_SIDE = -1
    else:
        OP_GOAL = [-0.856, 0]  # Opponent's goal position [x, y]
        OWN_GOAL = [0.856, 0]  # Own's goal position [x, y]
        INVERT_SIDE = 1

    desired_point, m = point_between_ball2mygoal(data, OWN_GOAL, dist2mygoal, INVERT_SIDE)

    point_reached = go_to_point(self, data, desired_point, False)

    distball = get_distBetweenObjects(data[self.name], data['ball'])
    if point_reached:  # chegou no ponto
        if distball > 0.2:
            left_motor = 2
            right_motor = 2
        else:
            left_motor = 0
            right_motor = 0

        self.left_motor.setVelocity(left_motor)
        self.right_motor.setVelocity(right_motor)


def truncamento(data, game_clock, CONTANDO_FLAG, BALL_AREA, INICIO) -> Tuple[bool, bool, float, int]:
    TRUNCAMENTO_FLAG = False
    if not CONTANDO_FLAG:
        x_mais = abs(data['ball']['x']) + 0.08
        x_menos = abs(data['ball']['x']) - 0.08
        y_mais = abs(data['ball']['y']) + 0.08
        y_menos = abs(data['ball']['y']) - 0.08
        BALL_AREA = [x_mais, x_menos, y_mais, y_menos]
        CONTANDO_FLAG = True
        INICIO = game_clock[0] * 60 + game_clock[1]
    else:
        if abs(data['ball']['x']) <= BALL_AREA[0] and abs(data['ball']['x']) >= BALL_AREA[1] and abs(
                data['ball']['y']) <= BALL_AREA[2] and abs(data['ball']['y']) >= BALL_AREA[3]:
            tempo_passou = INICIO - (game_clock[0] * 60 + game_clock[1])
            if tempo_passou >= 2:
                TRUNCAMENTO_FLAG = True
        else:
            TRUNCAMENTO_FLAG = False
            CONTANDO_FLAG = False
    return TRUNCAMENTO_FLAG, CONTANDO_FLAG, BALL_AREA, INICIO


def more_than_one_adversary(self, data, my_goal_thres) -> bool:
    if self.name[0] == 'Y':
        GK = 'Y1'
        ADVERSARY_NAMES = ['B1', 'B2', 'B3']
        FLAG = False
        if data['ball']['y'] < data[GK]['y']:  # Se a bola acima do goleiro
            i = 0
            for name in ADVERSARY_NAMES:
                if data[name]['x'] <= my_goal_thres and data[name]['y'] < data[GK]['y']:
                    i += 1
                    if i > 1:
                        FLAG = True
        else:
            i = 0
            for name in ADVERSARY_NAMES:
                if data[name]['x'] <= my_goal_thres and data[name]['y'] > data[GK]['y']:
                    i += 1
                    if i > 1:
                        FLAG = True

    else:
        GK = 'B1'
        ADVERSARY_NAMES = ['Y1', 'Y2', 'Y3']
        FLAG = False
        if data['ball']['y'] < data[GK]['y']:  # Se a bola acima do goleiro
            i = 0
            for name in ADVERSARY_NAMES:
                if data[name]['x'] >= my_goal_thres and data[name]['y'] < data[GK]['y']:
                    i += 1
                    if i > 1:
                        FLAG = True
        else:
            i = 0
            for name in ADVERSARY_NAMES:
                if data[name]['x'] >= my_goal_thres and data[name]['y'] > data[GK]['y']:
                    i += 1
                    if i > 1:
                        FLAG = True

    return FLAG


def are_ball_near_GK(self, data) -> bool:
    if self.name[0] == 'Y':
        MY_ROBOTS = ['Y1', 'Y2', 'Y3']
        OP_ROBOTS = ['B1', 'B2', 'B3']
    else:
        MY_ROBOTS = ['B1', 'B2', 'B3']
        OP_ROBOTS = ['Y1', 'Y2', 'Y3']

    dist1 = get_dist_Object2Point(data[MY_ROBOTS[0]], OWN_GOAL)
    dist2 = get_dist_Object2Point(data[MY_ROBOTS[1]], OWN_GOAL)
    dist3 = get_dist_Object2Point(data[MY_ROBOTS[2]], OWN_GOAL)

    vt = [dist1, dist2, dist3]
    minValue = min(vt)

    min_arg = -1

    for value_i in range(3):
        if minValue == vt[value_i]:
            min_arg = value_i
            break

    GK = MY_ROBOTS[min_arg]
    MY_ROBOT_NAMES = MY_ROBOTS

    if GK == MY_ROBOTS[0]:
        MY_ROBOT_NAMES = [MY_ROBOTS[1], MY_ROBOTS[2]]
    if GK == MY_ROBOTS[1]:
        MY_ROBOT_NAMES = [MY_ROBOTS[0], MY_ROBOTS[2]]
    if GK == MY_ROBOTS[2]:
        MY_ROBOT_NAMES = [MY_ROBOTS[0], MY_ROBOTS[1]]

    FLAG = False
    dist_op = [100, 100, 100]
    dist = [100, 100]

    i = 0
    for name in OP_ROBOTS:
        dist_op[i] = get_distBetweenObjects(data[name], data['ball'])
        i += 1

    i = 0
    for name in MY_ROBOT_NAMES:
        dist[i] = get_distBetweenObjects(data[name], data[GK])
        i += 1

    distball = get_distBetweenObjects(data[GK], data['ball'])

    if min(dist) > 0.1 and min(dist_op) < 0.15 and distball < 0.1:
        # print('entrou')
        # print(min(dist))
        FLAG = True

    return FLAG


def condition_for_banheira2(self, data) -> bool:
    if self.name[0] == 'Y':
        my_goal_thres = -0.65
        if more_than_one_adversary(self, data, my_goal_thres) and data['ball'][
            'x'] < my_goal_thres and are_ball_near_GK(self, data):
            return True
    else:
        my_goal_thres = 0.65
        if more_than_one_adversary(self, data, my_goal_thres) and data['ball'][
            'x'] > my_goal_thres and are_ball_near_GK(self, data):
            return True

    return False


def banheira2(self, data, game_clock, CONTANDO_FLAG, BALL_AREA, INICIO, ASSISTENCIA, SUPORTE_GOLEIRO) -> Tuple[
    bool, float, bool]:
    if self.name[0] == 'Y':
        acres = -0.1 if data['ball']['x'] < 0.5 else 0.03
        point = [-0.11, 0]
        offset = -0.05
        INVERT_SIDE = -1
    else:
        acres = 0.1 if data['ball']['x'] > -0.5 else 0.03
        point = [0.11, 0]
        offset = 0.05
        INVERT_SIDE = 1

    TRUNCAMENTO_FLAG, CONTANDO_FLAG, BALL_AREA, INICIO = truncamento(data, game_clock, CONTANDO_FLAG, BALL_AREA, INICIO)

    if INVERT_SIDE == 1:
        if not TRUNCAMENTO_FLAG and data['ball']['x'] < 0 and ASSISTENCIA:
            if data['ball']['y'] > 0:
                chegou = go_to_point(self, data, [data['ball']['x'] + offset, -0.2], False)
            else:
                chegou = go_to_point(self, data, [data['ball']['x'] + offset, 0.2], False)
            if chegou:
                face_the_ball(self, data)
        else:
            if condition_for_banheira2(self, data) and SUPORTE_GOLEIRO:
                if data['ball']['y'] > 0:
                    chegou = go_to_point(self, data, [0.733, 0.188], False)
                else:
                    chegou = go_to_point(self, data, [0.733, -0.188], False)
            else:
                chegou = go_to_point(self, data, point, False)
                if chegou:
                    frente = lookForward(self, data)
                    if frente:
                        self.left_motor.setVelocity(2)
                        self.right_motor.setVelocity(2)
    else:
        if not TRUNCAMENTO_FLAG and data['ball']['x'] > 0 and ASSISTENCIA:
            if data['ball']['y'] > 0:
                chegou = go_to_point(self, data, [data['ball']['x'] + offset, -0.2], False)
            else:
                chegou = go_to_point(self, data, [data['ball']['x'] + offset, 0.2], False)
            if chegou:
                face_the_ball(self, data)
        else:
            if condition_for_banheira2(self, data) and SUPORTE_GOLEIRO:
                if data['ball']['y'] > 0:
                    chegou = go_to_point(self, data, [-0.733, 0.188], False)
                else:
                    chegou = go_to_point(self, data, [-0.733, -0.188], False)
            else:
                chegou = go_to_point(self, data, point, False)
                if chegou:
                    frente = lookForward(self, data)
                    if frente:
                        self.left_motor.setVelocity(2)
                        self.right_motor.setVelocity(2)

    return CONTANDO_FLAG, BALL_AREA, INICIO


def mais_proximo_do_gol_adv(self, data):
    eu_sou_mais_proximo_do_gol_adv = False

    coord = []

    if (self.name[0] == 'Y'):
        coord = [0.856, 0]
    else:
        coord = [-0.856, 0]

    dist1 = get_dist_Object2Point(data[self.name[0] + "1"], coord)
    dist2 = get_dist_Object2Point(data[self.name[0] + "2"], coord)
    dist3 = get_dist_Object2Point(data[self.name[0] + "3"], coord)
    dists = []

    dists.append(dist1)
    dists.append(dist2)
    dists.append(dist3)

    dists = sorted(dists)

    if (get_dist_Object2Point(data[self.name], coord) == dists[0]):  # eu sou o mais próximo!!
        eu_sou_mais_proximo_do_gol_adv = True
    else:
        eu_sou_mais_proximo_do_gol_adv = False

    return dists, eu_sou_mais_proximo_do_gol_adv


def jogador_mais_px(self, data):
    dists = []

    dist1 = get_distBetweenObjects(data[self.name], data[self.name[0] + "1"])
    dist2 = get_distBetweenObjects(data[self.name], data[self.name[0] + "2"])
    dist3 = get_distBetweenObjects(data[self.name], data[self.name[0] + "3"])

    if dist1 == 0:
        dists.append(dist2)
        dists.append(dist3)

    elif dist2 == 0:
        dists.append(dist1)
        dists.append(dist3)

    elif dist3 == 0:
        dists.append(dist1)
        dists.append(dist2)

    if min(dists) < 0.4:
        tabela = True
    else:
        tabela = False

    return min(dists), tabela


def adv_mais_px(self, data):
    dists = []

    adv = "Y"

    if (self.name[0] == 'Y'):
        adv = "B"

    dist1 = get_distBetweenObjects(data[self.name], data[adv + "1"])
    dist2 = get_distBetweenObjects(data[self.name], data[adv + "2"])
    dist3 = get_distBetweenObjects(data[self.name], data[adv + "3"])

    dists.append(dist1)
    dists.append(dist2)
    dists.append(dist3)

    if min(dists) < 0.4:
        ladrao = True
    else:
        ladrao = False

    return min(dists), ladrao


def banheira_movel(self, data, pos):
    BANHEIRA_POINT_CENTER = [pos[0], pos[1]]

    if self.name[0] == 'Y':
        BANHEIRA_POINT_CENTER = [pos[0] * -1, pos[1]]
    else:
        BANHEIRA_POINT_CENTER = [pos[0], pos[1]]

    # Get the position of our robot
    robot_pos = data[self.name]
    # Get the position of the ball
    ball_pos = data['ball']
    # Get angle between the robot and the ball
    # and between the robot and the north
    ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)

    value = get_direction2point(robot_pos, robot_angle, BANHEIRA_POINT_CENTER)
    dist_robot2point = get_dist_Object2Point(robot_pos, BANHEIRA_POINT_CENTER)

    if dist_robot2point < 0.05:
        is_forward = lookForward(self, data)
    else:
        if value == 0:
            self.left_motor.setVelocity(-10)
            self.right_motor.setVelocity(-10)
        elif value == -1:
            # sentido anti-horario
            self.left_motor.setVelocity(2)
            self.right_motor.setVelocity(-2)
        else:  # value == 1
            # sentido horário
            self.left_motor.setVelocity(-2)
            self.right_motor.setVelocity(2)


def qnt_adversarios_meu_campo(self, data):
    counter = 0

    if self.name[0] == 'B':
        if (data["Y1"]['x'] > 0):
            counter += 1
        if (data["Y2"]['x'] > 0):
            counter += 1
        if (data["Y3"]['x'] > 0):
            counter += 1

    if self.name[0] == 'Y':
        if (data["B1"]['x'] < 0):
            counter += 1
        if (data["B2"]['x'] < 0):
            counter += 1
        if (data["B3"]['x'] < 0):
            counter += 1
    return counter


def upg_goleiro(self, data, GOAL_DEFAULT_DIST):
    if self.name[0] == 'Y':
        INVERT_SIDE = -1
    else:
        INVERT_SIDE = 1

    if condition_for_banheira2(self, data):
        if data['ball']['y'] < data[self.name]['y']:
            self.left_motor.setVelocity(3 if INVERT_SIDE == 1 else 10)
            self.right_motor.setVelocity(10 if INVERT_SIDE == 1 else 3)
        else:
            self.left_motor.setVelocity(10 if INVERT_SIDE == 1 else 3)
            self.right_motor.setVelocity(3 if INVERT_SIDE == 1 else 10)
    else:
        meu_goleiro(self, data, GOAL_DEFAULT_DIST)


def get_direction(ball_angle: float) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_angle (float): Angle between the ball and the robot

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if ball_angle >= 345 or ball_angle <= 15:
        return 0
    return -1 if ball_angle < 180 else 1


def get_distBetweenObjects(object_1, object_2) -> float:
    # Returns euclidean distance between object_1 and object_2
    return math.sqrt((object_1['x'] - object_2['x']) ** 2 + (object_1['y'] - object_2['y']) ** 2)


def get_dist_Object2Point(object_1, point_vector) -> float:
    # Returns euclidean distance between object_1 and point_vector
    return math.sqrt((object_1['x'] - point_vector[0]) ** 2 + (object_1['y'] - point_vector[1]) ** 2)


def condition2artilheiro(self, data):
    # Eu sou artilheiro

    if self.name[0] == 'Y':
        GK = 'Y1'
    else:
        GK = 'B1'
    # Se a bola está acima do meio do campo e se eu estou abaixo do goleiro OU (se a bola está abaixo do meio campo e se eu estou acima do goleiro)
    if (data['ball']['y'] < 0 and data[self.name]['y'] > data[GK]['y']) or (
            data['ball']['y'] > 0 and data[self.name]['y'] < data[GK]['y']):
        return True

    return False


# -------------------------------------------------------------------------- ARMADILLOS

def mais_proximo_do_meio(self, data):
    eu_sou_mais_proximo_do_centro = False

    dist1 = get_dist_Object2Point(data[self.name[0] + "1"], [0, 0])
    dist2 = get_dist_Object2Point(data[self.name[0] + "2"], [0, 0])
    dist3 = get_dist_Object2Point(data[self.name[0] + "3"], [0, 0])
    dists = []

    dists.append(dist1)
    dists.append(dist2)
    dists.append(dist3)

    dists = sorted(dists)

    if (get_dist_Object2Point(data[self.name], [0, 0]) == dists[0]):  # eu sou o mais próximo!!
        eu_sou_mais_proximo_do_centro = True
    else:
        eu_sou_mais_proximo_do_centro = False

    return dists, eu_sou_mais_proximo_do_centro