
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
# -----------------------------------------------------------------------------------------
# ----------------------------------------------------------------IMPORT BUILT_IN LIBRARIES
# Feel free to import built-in libraries here

# ----------------------------------------------------------------- DO NOT CHANGE OR DELETE
import math

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


# ------------------------------------------------------------------------------------------

class MyRobot3(RCJSoccerRobot):
    def run(self, counter, score, goal_time, GOAL_FLAG, TIME_FLAG, IMPORT_FLAG, KICK_FLAG, CONTANDO_FLAG, BALL_AREA,
            INICIO):
        # ----------------------------------------------------------------------------
        if not IMPORT_FLAG:
            if self.name[0] == 'Y':
               import my_functions
            else:
                import my_functions
            IMPORT_FLAG = True
        # ----------------------------------------------------------------------------
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()
                # --------------------------------------------------- INITS CRIADAS POR ARMADILLOS
                game_clock, counter = my_functions.game_time(counter)
                score, goal_time, GOAL_FLAG, TIME_FLAG = my_functions.scoreboard(data, score, game_clock, goal_time,
                                                                                 GOAL_FLAG, TIME_FLAG)

                mais_proximo_do_gol = my_functions.theChoosenGoalKeeper(self, data)
                mais_proximo_da_bola = my_functions.theChoosenOne(self, data)
                distancias_do_meio, eu_sou_mais_proximo_do_meio = my_functions.mais_proximo_do_meio(self, data)
                distancias_do_gol_adv, eu_sou_mais_proximo_do_gol_adv = my_functions.mais_proximo_do_gol_adv(self, data)

                adv_mais_proximo, ladrao = my_functions.adv_mais_px(self, data)
                jogador_mais_proximo, tabela = my_functions.jogador_mais_px(self, data)
                TRUNCAMENTO_FLAG, CONTANDO_FLAG, BALL_AREA, INICIO = my_functions.truncamento(data, game_clock,
                                                                                              CONTANDO_FLAG, BALL_AREA,
                                                                                              INICIO)

                # ********************************************************************
                # WRITE YOUR CODE HERE ***********************************************

                if self.name[0] == 'Y':
                    OP_GOAL = [0.75, 0]  # Own's goal position [x, y]
                    OWN_GOAL = [-0.75, 0]  # Opponent's goal position [x, y]

                    meus_gols = score[0]
                    adv_gols = score[1]
                else:
                    OP_GOAL = [-0.75, 0]  # Opponent's goal position [x, y]
                    OWN_GOAL = [0.75, 0]  # Own's goal position [x, y]
                    meus_gols = score[1]
                    adv_gols = score[0]

                # print(self.name + " COD1")
                if (mais_proximo_do_gol == True and mais_proximo_da_bola == True):
                    if (jogador_mais_proximo == False and mais_proximo_da_bola == False and adv_mais_proximo == True):
                        # print(self.name + " COD2")
                        KICK_FLAG = my_functions.artilheiro(self, data, STOP_NEAR_GOALKEEPER, KICK_FLAG,
                                                            KICK_INTENSITY_DEFAULT)
                    else:
                        # print(self.name +" CONTINUO GOLEIRO")
                        # print(self.name + " COD3")
                        my_functions.upg_goleiro(self, data, GOAL_DEFAULT_DIST)
                elif (mais_proximo_do_gol == True and mais_proximo_da_bola == False):
                    my_functions.upg_goleiro(self, data, GOAL_DEFAULT_DIST)
                    # print(self.name + " COD4")
                elif ((mais_proximo_do_gol == False and mais_proximo_da_bola == True) or (
                        eu_sou_mais_proximo_do_gol_adv == True and mais_proximo_da_bola == True)):  # verificar antes ou depois dp centro
                    # print(self.name + " COD5")
                    KICK_FLAG = my_functions.artilheiro(self, data, STOP_NEAR_GOALKEEPER, KICK_FLAG,
                                                        KICK_INTENSITY_DEFAULT)
                elif mais_proximo_do_gol == False and mais_proximo_da_bola == False and eu_sou_mais_proximo_do_meio == True:
                    # print(self.name + " COD6")
                    CONTANDO_FLAG, BALL_AREA, INICIO = my_functions.banheira2(self, data, game_clock, CONTANDO_FLAG,
                                                                              BALL_AREA, INICIO, ASSISTENCIA,
                                                                              SUPORTE_GOLEIRO)
                elif mais_proximo_do_gol == False and mais_proximo_da_bola == True and eu_sou_mais_proximo_do_meio == True:
                    KICK_FLAG = my_functions.artilheiro(self, data, STOP_NEAR_GOALKEEPER, KICK_FLAG,
                                                        KICK_INTENSITY_DEFAULT)
                    # print(self.name + " COD7")
                else:
                    KICK_FLAG = my_functions.artilheiro(self, data, STOP_NEAR_GOALKEEPER, KICK_FLAG,
                                                        KICK_INTENSITY_DEFAULT)
                    # print(self.name + " COD8")

                # ********************************************************************
