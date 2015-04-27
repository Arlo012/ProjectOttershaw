import graphics as gpy
import string

def DrawCircles():
    #xrange 1,4 for 6 circles and just 4 for 8 circles
    for i in xrange(1, 4):
        centers.append(gpy.Point(200, 200*i))
        centers.append(gpy.Point(500, 200*i))

    #Create center points for circle indicators
    for i, point in enumerate(centers):
        #Uncomment the appropriate line based on your microcontroller board
        #legStatusIndicators[54+i] = gpy.Circles(point, 5)
        legStatusIndicators[14+i] = gpy.Circle(point, 50)
        
    #Draw circles with red fill
    for indicator in legStatusIndicators.keys():
        legStatusIndicators[indicator].setOutline('red')
        legStatusIndicators[indicator].setFill('red')
        legStatusIndicators[indicator].draw(canvas)

def UpdateIndicatorStatus():
    for leg in legValues.keys():
            for indicator in legStatusIndicators.keys():
                if indicator == leg:
                    if legValues[leg] > noiseThreshold and legValues[leg] < stepThreshold:
                        legStatusIndicators[indicator].setOutline('red')
                        legStatusIndicators[indicator].setFill('red')
                    if legValues[leg] > stepThreshold:
                        legStatusIndicators[indicator].setOutline('green')
                        legStatusIndicators[indicator].setFill('green')
                    if legValues[leg] < noiseThreshold:
                        legStatusIndicators[indicator].setOutline('blue')
                        legStatusIndicators[indicator].setFill('blue')

centers = []
canvas = gpy.GraphWin('Leg Step Detection', 1000,1000)
legStatusIndicators = {}

legValues = {14:0, 15:0, 16:0, 17:0, 18:0, 19:0} 
# dataFile = open('ottershaw_piezoData.txt', 'r')

stepThreshold = 40   #threshold for leg step completion
noiseThreshold = 30 #noise from other legs moving, servo jitter, etc.

if __name__ == '__main__':

    DrawCircles()
    while True:
        dataFile = open('ottershaw_piezoData.txt', 'r')
        piezoReadings = dataFile.readline()
        # for line in piezoReadings:
        data = piezoReadings.strip('\n').split()
        for i, value in enumerate(data):
            legValues[14+i] = int(value)
        print legValues
        UpdateIndicatorStatus()
        dataFile.close()





    