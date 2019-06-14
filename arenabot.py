# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import random
import numpy as np


'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False) :
    
    # the Arena is a 100 x 100 pixel space
    arenaLength = 100
    arenaWidth = 100
    
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
    startDegrees = 90 # 90°
    
    # position of the objective
    objectiveX = 90
    objectiveY = 90
    
    # this is a list of points that the robot will visit; used later to visualize its path
    positions = []
    positions.append( [robotX, robotY] )

    # TODO move robot, check that the robot stays inside the arena and stop movement if a wall is hit
    # TODO measure distance from objective
    distanceFromObjective = 0
    
    # this is optional, argument "visualize" has to be explicitly set to "True" when function is called
    if visualize :
        
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        figure = plt.figure()
        ax = figure.add_subplot(111)
        
        # plot initial position and objective
        ax.plot(startX, startY, 'r^', label="Initial position of the robot")
        ax.plot(objectiveX, objectiveY, 'gx', label="Position of the objective")
        
        # plot the walls
        for wall in walls :
            ax.add_patch(patches.Rectangle( (wall["x"], wall["y"]), wall["width"], wall["height"] ))
        
        # plot a series of lines describing the movement of the robot in the arena
        for i in range(1, len(positions)) :
            ax.plot( [ positions[i-1][0], positions[i][0] ], [ positions[i-1][1], positions[i][1] ], 'r-', label="Robot path" )
        
        ax.set_title("Movements of the robot inside the arena")
        ax.legend(loc='best')
        plt.show()
    
    return distanceFromObjective


def tournamentSelection(population, size):
    tourn=np.random.choice(population,size, replace=False) # Selection du nombre d'individu pour le tournoi
    tournSorted=sorted(tourn, key=lambda k: k['Fitness']) # Classement des individus selon la fitness
    return tournSorted[0] # Renvoi du meilleur individu


def croisement(genome1, genome2):
    n_commands = len(genome1)
    pCros = random.uniforme(0, 1)
    for i in range(n_commands):
        ifCros = random.uniforme(0, 1)
        if ifCros<pCros:
            genome1[i] = genome2[i]
    return genome1

# can't change frome rotate to move or from move to rotate
def mutation(genome, mu, tau_move, tau_rotation):
    n_commands = len(genome)
    for i in range(n_commands):
        pMu = random.uniforme(0, 1)
        if pMu<mu:
            if genome[i][0]=='r':
                angle = filter(str.isdigit, genome[i])
                angle = angle + random.randint(-tau_rotation, tau_rotation)
                genome[i] = 'rotate' + str(angle)
            else:
                length = filter(str.isdigit, genome[i])
                length = length + randint(-tau_move, tau_move)
                length = max(0, min(length, 40))
                genome[i] = 'move' + length
    return genome

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
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 20, 10)
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
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 20, 10)
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        mergedPopulation.append(newIndividual)
    mergedPopulation = sorted(mergedPopulation, key=lambda k:k['Fitness'])
    return mergedPopulation[0: popSize]


'''L'ENCHAÎNEMENT DES OPÉRATIONS : en parallèle '''
'''This function will generate the next generation with the third method: elite parents are conserved, and p_croisement of the rest will do the croisement and then p_mutation of them mutate.'''
def nextGeneration_3(population, popSize, p_croisement, p_change):
    n_change = int(popSize * p_change)
    n_parent = popSize - n_change
    n_croisement = int(n_change * p_croisement)
    n_mutation = n_change - n_croisement
    tournSize = 2 # Taille de tournoi
    # generate children by croisement
    for i in range(int(n_croisement/2)):
        newIndividual1 = {}
        newIndividual2 = {}
        # Selection de l'individu à croiser
        indv1 = tournamentSelection(population[n_parent:popSize], tournSize)
        indv2 = tournamentSelection(population[n_parent:popSize], tournSize)
        newIndividual1['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual1['Fitness'] = fitnessRobot(newIndividual1['Genome'], False)
        newIndividual2['Genome'] = croisement(indv1['Genome'], indv2['Genome'])
        newIndividual2['Fitness'] = fitnessRobot(newIndividual2['Genome'], False)
        population[population.index(indv1)] = newIndividual1
        population[population.index(indv2)] = newIndividual2
    # generate children by mutation
    for i in range(n_mutation):
        indv = tournamentSelection(population[n_parent:popSize], tournSize)
        newIndividual = {}
        newIndividual['Genome'] = mutation(indv['Genome'], 0.3, 20, 10)
        newIndividual['Fitness'] = fitnessRobot(newIndividual['Genome'], False)
        population[population.index(indv)] = newIndividual
    population = sorted(population, key=lambda k:k['Fitness'])
    return population


################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
    # listOfCommands = []
    # fitnessRobot(listOfCommands, visualize=True)
    
    popSize = 50 # population
    n_commands = 20 # length of the command list of each person
    n_rotate = 0.2 # proportion of rotate commands
    n_gen = 20 # number of generations
    
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

    p_croisement = 0.5 # proportion of croisement, 1-p_croisement the proportion of mutatoion
    p_enfant = 0.5 # p_enfant * popSize the number of children produced
    
    for it in range(n_gen): # un tour de boucle = une génération
        # 3 ways to generate the next generation
        poplulation = nextGeneration_1(population, popSize, p_croisement, p_enfant)
        # poplulation = nextGeneration_2(population, popSize, p_croisement)
        # poplulation = nextGeneration_3(population, popSize, p_croisement, p_enfant)
        fitnesses = [p['Fitness'] for p in population]
        infosFitnesses = []
        bestFit = fitnesses[0]
        meanFit = np.mean(fitnesses)
        worstFit = fitnesses[-1]
        stdFit = np.std(fitnesses)
        infosFitnesses.append([bestFit, meanFit, worstFit, stdFit])
        print('Gen ', it+1, ': Best: ', bestFit, ' Mean: ', meanFit, ' Worst:', worstFit, 'Std: ', stdFit)
    fitnessRobot(population, True)
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

