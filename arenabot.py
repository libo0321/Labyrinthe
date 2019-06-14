# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import random
import numpy

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

################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
    # listOfCommands = []
    # fitnessRobot(listOfCommands, visualize=True)
    
    popSize = 20 # population
    n_commands = 20 # length of the command list of each person
    n_rotate = 0.2 # proportion of rotate commands
    n_gen = 20 # number of generations
    
    # generation 0
    population = []
    for i in range(popSize):
        newIndividual = {}
        commands = []
        for j in range(n_commands):
            n_random = random(1, 100)
            if n_random<=n_random*n_rotate:
                command[] = 'rotate' + str(random.randint(-90, 90))
            else:
                command = 'move' + str(random.randint(1,90))
            commands.append(command)
        newIndividual['Genome'] = commmands
        newIndividual['Fitness'] = fitnessRobot(commands, False)
        population.append(newIndividual)
    
    #Tri de la population en fonction de la Fitness des individus
    population = sorted(population, key=lambda k:k['Fitness'], reverse=Direction=='Minimisation')
    
    #Calcul des informations sur la population : meilleure fitness, fitness moyenne, pire fitness et déviation standard des fitness
    fitnesses = [p['Fitness'] for p in population]
    infosFitnesses = []
    bestFit = fitnesses[0]
    meanFit = np.mean(fitnesses)
    worstFit = fitnesses[-1]
    stdFit = np.std(fitnesses)
    infosFitnesses.append([bestFit, meanFit, worstFit, stdFit])
    print('Gen ', 0, ': Best: ', bestFit, ' Mean: ', meanFit, ' Worst:', worstFit, 'Std: ', stdFit)

    
    for it in range(n_gen): # un tour de boucle = une génération
        poplulation = nextGeneration(population)
        
	
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

