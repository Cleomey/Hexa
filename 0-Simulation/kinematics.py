import math
from constants import *

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.18
# constL3 = 0.250

def alkashi (a, b, c, sign = 1):
    if a * b == 0:
        print ("a ou b = null")
        return 0
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2 ) / (2 * a * b))))


def alkashi2 (a, b, theta, sign = -1):
    if a * b == 0:
        print ("a ou b = null")
        return 0
    return sign * math.sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(theta))




def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    #offsetthehta2 = theta1 / (360/(2*math.pi))
    #offsetthehta3 = theta1 / (360/(2*math.pi))
    #x = ((l1 + l2 * math.cos(theta2) )* math.cos(theta1) ) 
    #y = ((l1 + l2 * math.cos(theta2) ) * math.sin(theta1))
    #z = (-(l2 * math.sin(theta2 )))
    # la je suis dans le cas ou j'ai juste p3
    x = ((l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.cos(theta1)) 
    y = ((l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.sin(theta1))
    z = (-(l3 * math.sin(theta2 + theta3) + l2 * math.sin(theta2)) )

    return [x, y, z]


def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):

    d13 = (math.sqrt((x * x) + (y * y))) - l1
    d = (math.sqrt((z * z) + (d13 * d13 )))
    

    theta1 = math.atan2(y, x)
    theta2 = (math.atan2(-z,d13)) + alkashi (l2, d, l3)
    theta3 = alkashi(l2, l3, d) + math.pi

    if d > l2 + l3:
        print ("**** ta mère tu peux pas *****")
    elif d < l2 + l3:
        print ("**** choisit t'as 2 possibilitrucs")
        if x == 0 and y == 0 : 
            theta1 = 0 # fais un saut, essayer d'enlever ce saut avec un bolé1
            print  ("Hé vazy *** t'as tro de possibi****")
        

    return [theta1, theta2, theta3]


def computeDKsimple(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    # A completer
    theta1 = theta1 / (360/(2*math.pi))
    theta2 = theta2 / (360/(2*math.pi))
    theta3 = theta3 / (360/(2*math.pi))
    x = (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.cos(theta1)
    y = (l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)) * math.sin(theta1)
    z = (l3 * math.sin(theta2 + theta3) + l2 * math.sin(theta2))

    return [x, y, z]


def circle ( x, z, r, t, duration ) :
    w = (2 * math.pi) / duration
    y = math.cos (w * t) * r
    z = math.sin (w * t) * r + z


    return computeIK (x, y, z)

def segment (x1, y1, z1, x2, y2, z2, t, duration) :
    k = t / duration
    x = k * (x2 - x1) + x1
    y = k * (y2 - y1) + y1
    z = k * (z2 - z1) + z1

    if t > duration:
        return computeIK(x2, y2, z2)

    return computeIK (x, y, z)

def triangle (x, z, h, w, t):
    duration = 12
    t = t % duration
    x1 = x
    x2 = x
    x3 = x
    y1 = - w / 2
    y2 = w / 2
    y3 = 0
    z1 = 0
    z2 = 0
    z3 = h + z

    if t <= duration / 3 :
        return segment (x1, y1, z1, x2, y2, z2, t, duration/3)
    elif t <= 2 * (duration / 3) : 
        return segment (x2, y2, z2, x3, y3, z3, t - (duration / 3) , duration/3)
    else:
        return  segment (x3, y3, z3, x1, y1, z1, t - 2*(duration / 3), duration/3)
    
    

def main():
    print("Testing the kinematic funtions...")
    print(
        "computeDK(0, 0, 0) = {}".format(
            computeDKsimple(0, 0, 0, l1=constL1, l2=constL2, l3=constL3)
           
        )
    )
    print("")
    print(
        "computeDKsimple(0, 0, 0) = {}".format(
            computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3)
           
        )
    )


if __name__ == "__main__":
    main()