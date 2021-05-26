# -------------DEMOCRITUS----------------

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import utils
from enum import auto
import math

TIME_STEP = 64
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

ATTACK = auto()         #  Επίθεση όταν η μπάλα είναι μπροστά από το ρομπότ
ATTACK_BACK = auto()    #  Επίθεση όταν η μπάλα είναι πίσω από το ρομπότ
DEFENDER_GO = auto()    #  Ο Αμυντικός κινείται για να φτάσει έξω από την περιοχή του
DEFENDER_TURN = auto()  #  Ο Αμυντικός στρίβει για να γυρίσει παράλληλα στο τέρμα
DEFENDER_MOVE = auto()  #  Ο Αμυντικός παρακολουθεί τη μπάλα (κινείται στο ίδιο y)
DEFENDER_STAY = auto()  #  Ο Αμυντικός μένει ακίνητος
CENTER_GO = auto()      #  Ο Αμυντικός (όταν μπλοκάρει η μπάλα) πάει στο κέντρο
CENTER_TURN = auto()    #  Ο Αμυντικός (όταν μπλοκάρει η μπάλα) στρίβει στις 90 μοίρες όταν φτάσει στο κέντρο
CENTER_MOVE = auto()    #  Ο Αμυντικός (όταν μπλοκάρει η μπάλα) κινείται ελαφρά μπρος πίσω όταν φτάσει στο κέντρο και στρίψει στις 90 μοίρες
LEFT_GO = auto()        # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) πάει αριστερά έξω από την περιοχή
LEFT_TURN = auto()      # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) στρίβει στις 90 μοίρες
LEFT_MOVE = auto()      # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) κινείται ελαφρά μπρος πίσω
LOW_LEFT_GO = auto()    # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) πάει αριστερά έξω από την περιοχή του
LOW_LEFT_TURN = auto()  # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) στρίβει στις 90 μοίρες
LOW_LEFT_MOVE = auto()  # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) κινείται ελαφρά μπρος πίσω
RIGHT_GO = auto()       # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) πάει δεξιά έξω από την περιοχή
RIGHT_TURN = auto()     # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) στρίβει στις 90 μοίρες
RIGHT_MOVE = auto()     # Ο Μέσος (όταν μπλοκάρει η μπάλα στην επίθεση) κινείται ελαφρά μπρος πίσω
LOW_RIGHT_GO = auto()   # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) πάει δεξιά έξω από την περιοχή του
LOW_RIGHT_TURN = auto() # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) στρίβει στις 90 μοίρες
LOW_RIGHT_MOVE = auto() # Ο Μέσος (όταν μπλοκάρει η μπάλα στην άμυνα) κινείται ελαφρά μπρος πίσω

MIDDLE_ATTACK = auto()  # Ο Μέσος κυνηγά τη μπάλα, όταν είναι μπροστά του και είναι πιο κοντά σε αυτή απ΄ ότι ο επιθετικός
MIDDLE_FOLLOW = auto()  # Ο Μέσος όταν η μπάλα είναι μπροστά του, αλλά πιο κοντά στον επιθετικό, ακολουθεί τον επιθετικό (πιο αργά)
MIDDLE_FOLLOW_REVERSE = auto() # Ο Μέσος όταν η μπάλα είναι πίσω του, αλλά πιο κοντά στον επιθετικό, ακολουθεί τη μπάλα (πιο αργά)
MIDDLE_SELECT = auto() # Βοηθητικό state για την επιλογή της λειτουργίας του Μέσου


class MyRobot1(RCJSoccerRobot):
    def __init__(self, robot):
        self.robot = robot
        self.name = self.robot.getName()

        self.attacker = "-"     # To όνομα του ρομπότ που πήρε το ρόλο Επιθετικός
        self.middle = "-"       # Το όνομα του ρομπότ που πήρε το ρόλο Μέσος
        self.defender = "-"     # Το όνομα του ρομπότ που πήρε το ρόλο Αμυντικός
        self.attackerDist = 0   # Η απόσταση του Επιθετικού από τη μπάλα
        self.middleDist = 0     # Η απόσταση του Μέσου από τη μπάλα
        self.defenderDist = 0   # Η απόσταση του Αμυντικού από τη μπάλα

        self.state = ATTACK     # Αρχικά όλοι θα παίξουν επίθεση

        self.alarm = 0          # βοηθητική μεταβλητή που μετρά χρόνο (αριθμό επαναλήψεων)

        self.lastBall = None    # Η θέση της μπάλας στην προηγούμενη επανάληψη
        self.lop = [0 for _ in range(90)]   # Λίστα 90 θέσεων (με μηδενικά στην αρχή). Θα χρησιμοποιηθεί για να ανιχνεύουμε αν η μπάλα κόλλησε.
        self.sumLop = 0         # Το άθροισμα όλως των στοιχείων της λίστας lop
        self.lopFlag = False    # Έχει κολλήσει η μπάλα;

        self.flip = False       # True όταν έχουμε τη μπλε ομάδα, για να αντιστρέψουμε τα πρόσιμα στις συντεταγμένες.
        self.changeAttack = False   # True όταν ο Επιθετικός παίζει ανάποδα.
        self.changeMiddleAttack = False # True όταν ο Μέσος παίζει ανάποδα.
        if self.name[0] == 'B':
            self.flip = True

        self.p1 = self.name[0] + "1"    # p1 =  B1 ή Υ1
        self.p2 = self.name[0] + "2"    # p2 =  B2 ή Υ2
        self.p3 = self.name[0] + "3"    # p3 =  B3 ή Υ3

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float('+inf'))
        self.right_motor.setPosition(float('+inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    # Η συνάρτηση assignRoles αναθέτει ρόλους στους παίκτες
    # Αυτός που είναι πιο κοντά στη μπάλα γίνεται Επιθετικός
    # Αυτός που είναι πιο μακριά από τη μπάλα γίνεται Αμυντικός
    # O τρίτος γίνεται Μέσος

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
        self.clock = 0  # O χρόνος του παιχνιδιού (αριθμός επαναλήψεων)
        while self.robot.step(TIME_STEP) != -1:
            if self.receiver.getQueueLength() > 0:
                self.clock += 1     # σε κάθε επανάληψη προσθέτω 1

                data = self.get_new_data()  # στη μεταβλητή data είναι όλα τα δεδομένα για παίκτες και μπάλα (x,y)
                if self.flip:               # Αν έχω τη μπλε ομάδα αντιστρέφω τις συντεταγμένες.
                    data['ball']['x'] *= -1
                    data['ball']['y'] *= -1
                    data[self.p1]['x'] *= -1
                    data[self.p1]['y'] *= -1
                    data[self.p2]['x'] *= -1
                    data[self.p2]['y'] *= -1
                    data[self.p3]['x'] *= -1
                    data[self.p3]['y'] *= -1
                    data[self.name]['orientation'] += math.pi


                robot_pos = data[self.name]     # Οι συντεταγμένες (x,y) αυτού το ρομπότ
                ball_pos = data['ball']         # Οι συντεταγμένες (x,y) της μπάλας
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos) # Οι γωνίες ανάμεσα στη μπάλα και στο ρομπότ, και η γωνία που κοιτά το ρομπότ.

                # -------------- Υπολογισμοί για να βρω αν κόλλησε η μπάλα --------------------

                if self.lastBall == None:
                    self.lastBall = [ball_pos['x'], ball_pos['y']]
                error = math.sqrt((self.lastBall[0]-ball_pos['x'])**2 + (self.lastBall[1]-ball_pos['y'])**2)

                self.lop[self.clock % 90] = error
                self.lastBall = [ball_pos['x'], ball_pos['y']]
                self.sumLop = sum(self.lop)
                # ----------------------------------------------------------------------------

                # Υπολογίσω τις αποστάσεις μετά το clock=50. Δεν το κάνω νωρίτερα γιατί στο clock=50 αναθέτω τους ρόλους
                if self.clock > 50:
                    self.defenderDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.defender]['x'],
                                                    data[self.defender]['y'])
                    self.middleDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.middle]['x'],
                                                    data[self.middle]['y'])
                    self.attackerDist = utils.get_ballDist(ball_pos['x'], ball_pos['y'], data[self.attacker]['x'],
                                                    data[self.attacker]['y'])

                # Για clock=50 αναθέτω ρόλους στους παίκτες
                if self.clock == 50:
                    self.assignRoles(ball_pos, robot_pos, data)
                elif self.clock > 50 and self.sumLop <= 0.5 and not self.lopFlag:  # Αν κολλήσει η μπάλα
                    self.lopFlag = True
                    print("**********************************************************")
                    print("**********************************************************")
                    print("**********************************************************")
                    if ball_pos['x'] > 0:   #  Αν η μπάλα είναι στην επίθεση
                        if self.name == self.middle:
                            if ball_pos['y'] > 0:
                                self.state = RIGHT_GO
                            else:
                                self.state = LEFT_GO
                        elif self.name == self.defender:
                            self.state = CENTER_GO
                    else:                   #  Αν η μπάλα είναι στην άμυνα
                        if self.name == self.middle:
                            if ball_pos['y'] > 0:
                                self.state = LOW_LEFT_GO
                            else:
                                self.state = LOW_RIGHT_GO

                        elif self.name == self.defender:
                            self.state = DEFENDER_STAY


                elif self.clock > 50 and self.sumLop > 0.5 and self.lopFlag: #  Αν η μπάλα ξεκολλήσει
                    self.lopFlag = False
                    if ball_pos['x'] > 0:   #  Αν η μπάλα είναι στην επίθεση, αναθέτω πάλι νέους ρόλους
                        self.assignRoles(ball_pos, robot_pos, data)
                    else:                   # Αν η μπάλα είναι στην άμυνα
                        if self.name == self.middle:
                            self.state = MIDDLE_SELECT
                        elif self.name == self.defender:
                            self.state = DEFENDER_GO


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
                            setx = -0.6
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
                        point = {'x': -0.3, 'y': 0.3}
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
                        point = {'x': -0.3, 'y': -0.3}
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