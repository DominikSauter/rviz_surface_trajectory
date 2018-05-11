#!/usr/bin/python
# -*- coding: latin-1 -*-

import numpy as np
import matplotlib.pyplot as plt
import sys


# parse command line arguments
args = [arg for arg in sys.argv]
arg1 = int(args[1])
arg2 = args[2]
arg3 = float(args[3])
arg4 = float(args[4])


# read in data
with open(arg2) as f:
    data = f.read()

data = data.split('\n')

x1 = [row.split(' ')[8] for row in data]
x2 = [row.split(' ')[9] for row in data]
x3 = [row.split(' ')[10] for row in data]
x4 = [row.split(' ')[11] for row in data]


tickLabelx4 = [unicode('⌀ ', 'utf-8') + x4n for x4n in x4]
tickLabelx1 = [' / ' + x1n for x1n in x1]
tickLabelx2 = ['\n(' + x2n + ')' for x2n in x2]
tickLabels = [tickLabelx4[i] + tickLabelx1[i] + tickLabelx2[i] for i in xrange(0, len(tickLabelx1))]

# choose mesh according to command line argument
if arg1 == 0:
    meshTitle = 'Mesh: Kugel (960 Dreiecke)'
elif arg1 == 1:
    meshTitle = 'Mesh: Stanford Bunny (86632 Dreiecke)'
elif arg1 == 2:
    meshTitle = 'Mesh: T-Rex-Oberkiefer (167766 Dreiecke)'

plotTitles = (unicode(meshTitle + '\nAlgorithmus: projektionstreue Oberflächen-Trajektorie', 'utf-8'), unicode(meshTitle + '\nAlgorithmus: oberflächentreue Oberflächen-Trajektorie', 'utf-8'),
            unicode(meshTitle + '\nAlgorithmus: projektionstreue Offset-Trajektorie', 'utf-8'), unicode(meshTitle + '\nAlgorithmus: oberflächentreue Offset-Trajektorie', 'utf-8'))


averagedDataSEC = [[i] for i in range(0, 4)]
averagedDataOP = [[i] for i in range(0, 4)]


# set most of the plot data
def createPlot(counter):

    y = [float(row.split(' ')[counter * 2]) for row in data]
    surfacePoints = [float(row.split(' ')[(counter * 2) + 1]) for row in data]


    # calculate the average seconds per raycast-point and the average surface points per raycast-point
    averagedDataSEC[counter] = [yn / float(x4n) for yn, x4n in zip(y, x4)]
    averagedDataOP[counter] = [surfacePointsn / float(x4n) for x4n, surfacePointsn in zip(x4, surfacePoints)]


    # set up plot and bar settings
    fig, ax = plt.subplots()

    N = len(data)
    index = np.arange(N)
    bar_width = 0.35
    opacity = 0.4

    rects1 = ax.bar(index, y, bar_width,
                     alpha=opacity,
                     color='b')
                     #label='Men')


    ax.set_xlabel(unicode('⌀ ', 'utf-8') + 'Raycast-Punkte / Trajektionsmusterpunkte\n(Musterwiederholungen)')
    ax.set_ylabel(unicode('⌀ ', 'utf-8') + 'Berechnungszeit in Sekunden')
    ax.set_title(plotTitles[counter])
    ax.set_xticks(index + bar_width / 2.)
    ax.set_xticklabels(tickLabels)

    ax.legend()


    # create description text for the bars
    def autolabel(rects):
        # attach some text labels
        for rect, surfacePoint in zip(rects, surfacePoints):
            height = rect.get_height()
            # offset text different for projection and surface loyal algorithms
            if counter == 0 or counter == 2:
                ax.text(rect.get_x()+rect.get_width()/2., height + arg3, unicode('⌀ ', 'utf-8') + str(surfacePoint) + ' OP', ha='center', va='center')
            else:
                ax.text(rect.get_x()+rect.get_width()/2., height + arg4, unicode('⌀ ', 'utf-8') + str(surfacePoint) + ' OP', ha='center', va='center')


    autolabel(rects1)

    plt.tight_layout()

    plt.savefig('PerformanceTests' + str(counter) +'.png')


# create all plots
i = 0
while i < 4:
    createPlot(i)
    i = i + 1


# print average data
for i in xrange(0, len(averagedDataSEC[0])):
    print str(averagedDataSEC[0][i]) + ' s/RP (' + str(averagedDataOP[0][i]) + ' OP/RP) ' + str(averagedDataSEC[1][i]) + ' s/RP (' + str(averagedDataOP[1][i]) + ' OP/RP) ' + str(averagedDataSEC[2][i]) + ' s/RP (' + str(averagedDataOP[2][i]) + ' OP/RP) ' + str(averagedDataSEC[3][i]) + ' s/RP (' + str(averagedDataOP[3][i]) + ' OP/RP) '

# calculate and print average of average data
averagedAveragedDataSEC = [0 for i in range(0, 4)]
averagedAveragedDataOP = [0 for i in range(0, 4)]
for j in xrange(0, len(averagedDataSEC)):
    for i in xrange(0, len(averagedDataSEC[0])):
        averagedAveragedDataSEC[j] += averagedDataSEC[j][i]
        averagedAveragedDataOP[j] += averagedDataOP[j][i]
    averagedAveragedDataSEC[j] = averagedAveragedDataSEC[j] / float(len(averagedDataSEC[0]))
    averagedAveragedDataOP[j] = averagedAveragedDataOP[j] / float(len(averagedDataOP[0]))
    print str(averagedAveragedDataSEC[j]) + ' s/RP (' + str(averagedAveragedDataOP[j]) + ' OP/RP)'



plt.show()
