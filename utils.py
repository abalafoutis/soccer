import math

def get_ballDist(x1, y1, x0, y0):
    y = y1 - y0
    x = x1 - x0
    dist = math.sqrt(x**2 + y**2)
    return dist

def turnRobot(target_deg, robot_angle):
    kp = 0.2

    robot_angle = math.degrees(robot_angle)

    if target_deg - robot_angle > 180:
        robot_angle += 360
    elif target_deg - robot_angle < -180:
        robot_angle -= 360

    error = target_deg - robot_angle
    return error*kp, -error*kp

def followBallFliped(ball_angle):
    kp = 0.2

    if 180 - ball_angle > 180:
        ball_angle += 360
    elif 180 - ball_angle < -180:
        ball_angle -= 360

    error = 180 - ball_angle
    return error*kp, -error*kp

def followBall(ball_angle):
    kp = 0.2

    if 0 - ball_angle > 180:
        ball_angle += 360
    elif 0 - ball_angle < -180:
        ball_angle -= 360

    error = 0 - ball_angle
    return error*kp, -error*kp

def isBallAhead(x1, x0):
    distx1 = 1 - x1
    distx0 = 1 - x0
    if distx1 > distx0:
        return False
    else:
        return True

def isPlayerAhead(x1, x0):
    distx1 = 1 - x1
    distx0 = 1 - x0
    if distx1 > distx0 - 0.1:
        return False
    else:
        return True