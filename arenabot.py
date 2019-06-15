import sys
import random
import numpy as np
import math


'''This function will calculate the distance between the robot and the objective.'''
def calcul_distance(robotX, robotY, objectiveX, objectiveY):
    distance = abs(robotX - objectiveX)
    if robotX<=40:
        distance = distance + 160 + abs(robotY - objectiveY)
    elif robotX<=80:
        distance = distance + 80 + abs(robotY - 10)
    else:
        distance = distance + abs(robotY - objectiveY)
    return distance


'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False):
    # the Arena is a 100 x 100 pixel space
    arenaLength = 100
    arenaWidth = 100
    # print(listOfCommands)
    # let's also put a couple of walls in the arena; walls are described by a set of 4 (x,y) corners (bottom-left, top-left, top-right, bottom-right)
    walls = []

    wall1 = dict()
    wall1["x"] = 30
    wall1["y"] = 0
    wall1["width"] = 10
    wall1["height"] = 80

    wall2 = dict()
    wall2["x"] = 70
    wall2["y"] = 20
    wall2["width"] = 10
    wall2["height"] = 80

    walls.append(wall1)
    walls.append(wall2)

    # initial position and orientation of the robot
    startX = robotX = 10
    startY = robotY = 10
    Degree= 90  # 90°

    # position of the objective
    objectiveX = 90
    objectiveY = 90

    # this is a list of points that the robot will visit; used later to visualize its path
    positions = []
    positions.append([robotX, robotY])

    # TODO move robot, check that the robot stays inside the arena and stop movement if a wall is hit
    
    for command in listOfCommands:
        if command.startswith('rotate'):
            Degree = Degree + int(command[6:len(command)])
            # print('Degree :',Degree)
        else:
            distance = int(command[4:len(command)])
            # print(distance)
            destiX = robotX + distance*math.cos(Degree/180*math.pi)
            destiY = robotY + distance*math.sin(Degree/180*math.pi)
            if move_possible(walls, robotX, robotY, destiX, destiY):
                robotX = destiX
                robotY = destiY
                positions.append([robotX, robotY])
            else:
                break
            # print("robotX : ", robotX)
            # print("robotY : ", robotY)
            
    # measure distance from objective

    distanceFromObjective = calcul_distance(robotX, robotY, objectiveX, objectiveY)

    # this is optional, argument "visualize" has to be explicitly set to "True" when function is called
    if visualize:

        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        figure = plt.figure()
        ax = figure.add_subplot(111)

        # plot initial position and objective
        ax.plot(startX, startY, 'r^', label="Initial position of the robot")
        ax.plot(objectiveX, objectiveY, 'gx', label="Position of the objective")

        # plot the walls
        for wall in walls:
            ax.add_patch(patches.Rectangle((wall["x"], wall["y"]), wall["width"], wall["height"]))

        # plot a series of lines describing the movement of the robot in the arena
        for i in range(1, len(positions)-1):
            ax.plot([positions[i - 1][0], positions[i][0]], [positions[i - 1][1], positions[i][1]], 'r-')
        ax.plot([positions[len(positions) - 2][0], positions[len(positions)-1][0]], [positions[len(positions) - 2][1], positions[len(positions)-1][1]], 'r-',label="Robot path")


        ax.set_title("Movements of the robot inside the arena")
        ax.legend(loc='best')
        plt.show()

    return distanceFromObjective

def move_possible(walls, a0, b0, a, b):
    for wall in walls:
        wall_x1 = wall['x']
        wall_y1 = wall['y']
        wall_x2 = wall['x'] + wall['width']
        wall_y2 = wall['y'] + wall['height']
        if a >= 0 and a <= 100 and b >= 0 and b <= 100:
            if a > wall_x1 and a < wall_x2 and b > wall_y1 and b < wall_y2:
                return False
            else:
                if max(a0,a)>=min(wall_x1,wall_x2) and max(wall_x1,wall_x2)>=min(a0,a) and max(b0,b)>=min(wall_y1,wall_y2) and max(wall_y1,wall_y2)>=min(b0,b):
                    return False
        else:
            return False
    return True

def tournamentSelection(population, size):
    tourn=np.random.choice(population,size, replace=False) # Selection du nombre d'individu pour le tournoi
    tournSorted=sorted(tourn, key=lambda k: k['Fitness']) # Classement des individus selon la fitness
    return tournSorted[0] # Renvoi du meilleur individu


def croisement(genome1, genome2):
    n_commands = len(genome1)
    enfant = []
    pCros = random.uniform(0, 1)
    for i in range(n_commands):
        ifCros = random.uniform(0, 1)
        if ifCros<pCros:
            enfant.append(genome1[i])
        else:
            enfant.append(genome2[i])
    return enfant

# can't change frome rotate to move or from move to rotate
def mutation(genome, mu, tau_move, tau_rotation):
    n_commands = len(genome)
    enfant = []
    for i in range(n_commands):
        pMu = random.uniform(0, 1)
        if pMu < mu:
            if genome[i][0] == 'r':
                angle = int(genome[i][6:len(genome[i])])
                angle = angle + random.randint(-tau_rotation, tau_rotation)
                enfant.append( 'rotate' + str(angle))
            else:
                length = int(genome[i][4:len(genome[i])])
                length = length + random.randint(-tau_move, tau_move)
                length = max(0, min(length, 40))
                enfant.append('move' + str(length))
        else:
            enfant.append(genome[i])
    return enfant


'''TAUX DE RENOUVELLEMENT FIXE '''
'''This function will generate the next generation with the first method: mu-lambda best parents are conserved, and lambda new children are produced.'''
def nextGeneration_1(population, popSize, p_croisement, p_enfant):
    n_enfant = int(popSize * p_enfant)
    n_parent = popSize - n_enfant
    mergedPopulation = population[0:n_parent]
    n_croisement = int(n_enfant * p_croisement)
    n_mutation = n_enfant - n_croisement
    tournSize = 2 # Taille de tournoi
    # generate children by croisement
    for i in range(n_croisement):
        newIndividual = {}
        # Selection de l'individu à croiser
        indv1 = tournamentSelection(population, tournSize)
        indv2 = tournamentSelection(population, tournSize)
        newIndividual['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        mergedPopulation.append(newIndividual)
    # generate children by mutation
    for i in range(n_mutation):
        indv = tournamentSelection(population, tournSize)
        newIndividual = {}
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 10, 10)
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        mergedPopulation.append(newIndividual)
    mergedPopulation = sorted(mergedPopulation, key=lambda k:k['Fitness'])
    return mergedPopulation


'''TAUX DE RENOUVELLEMENT VARIABLE'''
'''This function will generate the next generation with the second method: all parents are conserved, and lambda (for now lambda=popSize) new children are produced, and we select popSize best individuals in the merged population.'''
def nextGeneration_2(population, popSize, p_croisement):
    mergedPopulation = population
    n_enfant = popSize
    n_croisement = int(n_enfant * p_croisement)
    n_mutation = n_enfant - n_croisement
    tournSize = 2 # Taille de tournoi
    # generate children by croisement
    for i in range(n_croisement):
        newIndividual = {}
        # Selection de l'individu à croiser
        indv1 = tournamentSelection(population, tournSize)
        indv2 = tournamentSelection(population, tournSize)
        newIndividual['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        mergedPopulation.append(newIndividual)
    # generate children by mutation
    for i in range(n_mutation):
        indv = tournamentSelection(population, tournSize)
        newIndividual = {}
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 10, 10)
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        mergedPopulation.append(newIndividual)
    mergedPopulation = sorted(mergedPopulation, key=lambda k:k['Fitness'])
    return mergedPopulation[0: popSize]


'''L'ENCHAÎNEMENT DES OPÉRATIONS : en parallèle '''
'''This function will generate the next generation with the third method: elite parents are conserved, and p_croisement of the rest will do the croisement and then p_mutation of them mutate.'''
def nextGeneration_3(population, popSize, p_croisement, p_change):
    n_change = int(popSize * p_change)
    n_parent = popSize - n_change
    mergedPopulation = population[0: n_parent]
    Rpop = population[n_parent: popSize]
    n_croisement = int(n_change * p_croisement)
    n_mutation = n_change - n_croisement
    tournSize = 2 # Taille de tournoi
    # generate children by croisement
    for i in range(int(n_croisement/2)):
        newIndividual1 = {}
        newIndividual2 = {}
        # Selection de l'individu à croiser
        indv1 = tournamentSelection(Rpop, tournSize)
        indv2 = tournamentSelection(Rpop, tournSize)
        newIndividual1['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual1['Fitness'] = fitnessRobot(newIndividual1['Genome'], False)
        newIndividual2['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual2['Fitness'] = fitnessRobot(newIndividual2['Genome'], False)
        Rpop[Rpop.index(indv1)] = newIndividual1
        Rpop[Rpop.index(indv2)] = newIndividual2
    # generate children by mutation
    for i in range(n_mutation):
        indv = tournamentSelection(Rpop, tournSize)
        newIndividual = {}
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 10, 10)
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        Rpop[Rpop.index(indv)] = newIndividual
    mergedPopulation = mergedPopulation + Rpop
    mergedPopulation = sorted(mergedPopulation, key=lambda k:k['Fitness'])
    return mergedPopulation


################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
    # listOfCommands = []
    # fitnessRobot(listOfCommands, visualize=True)
    
    popSize = 200 # population
    n_commands = 90 # length of the command list of each person
    n_rotate = 0.5 # proportion of rotate commands
    n_gen = 200 # number of generations
    
    # generation 0
    population = []
    for i in range(popSize):
        newIndividual = {}
        commands = []
        for j in range(n_commands):
            n_random = random.randint(1, 100)
            if n_random<=100*n_rotate:
                # rotate angle should be between -90 and 90
                command = 'rotate' + str(random.randint(-90, 90))
            else:
                # move length should be between 1 and 40
                command = 'move' + str(random.randint(1,40))
            commands.append(command)
        newIndividual['Genome'] = commands
        newIndividual['Fitness'] = fitnessRobot(commands, False)
        population.append(newIndividual)
    
    #Tri de la population en fonction de la Fitness des individus
    population = sorted(population, key=lambda k:k['Fitness'])
    
    #Calcul des informations sur la population : meilleure fitness, fitness moyenne, pire fitness et déviation standard des fitness
    fitnesses = [p['Fitness'] for p in population]
    infosFitnesses = []
    bestFit = fitnesses[0]
    meanFit = np.mean(fitnesses)
    worstFit = fitnesses[-1]
    stdFit = np.std(fitnesses)
    infosFitnesses.append([bestFit, meanFit, worstFit, stdFit])
    print('Gen ', 0, ': Best: ', bestFit, ' Mean: ', meanFit, ' Worst:', worstFit, 'Std: ', stdFit)

    p_croisement = 0.8 # proportion of croisement, 1-p_croisement the proportion of mutatoion
    p_enfant = 0.6 # p_enfant * popSize the number of children produced
    
    for it in range(n_gen): # un tour de boucle = une génération
        # 3 ways to generate the next generation
        population = nextGeneration_1(population, popSize, p_croisement, p_enfant)
        # population = nextGeneration_2(population, popSize, p_croisement)
        # population = nextGeneration_3(population, popSize, p_croisement, p_enfant)
        fitnesses = [p['Fitness'] for p in population]
        infosFitnesses = []
        bestFit = fitnesses[0]
        meanFit = np.mean(fitnesses)
        worstFit = fitnesses[-1]
        stdFit = np.std(fitnesses)
        infosFitnesses.append([bestFit, meanFit, worstFit, stdFit])
        print('Gen ', it+1, ': Best: ', bestFit, ' Mean: ', meanFit, ' Worst:', worstFit, 'Std: ', stdFit)
    fitnessRobot(population[0]['Genome'], True)
    print(population[0]['Fitness'])
    return 0


if __name__ == "__main__" :
	sys.exit( main() )

