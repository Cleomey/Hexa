import math
import pybullet as p
from constants import *


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


def computeDK(theta1,theta2,theta3,l1=constL1,l2=constL2,l3=constL3,use_rads=USE_RADS_INPUT,use_mm=USE_MM_OUTPUT,):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )
    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)
    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit
    return [x, y, z]


def computeDKP1(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    # theta1 = radians(theta1)
    # theta2 = radians(theta2)
    # theta3 = radians(theta3)
    x = l1*math.cos(theta1)
    y = l1*math.sin(theta1)
    z = 0
    return [x, y, z]


def computeDKP2(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    # theta1 = radians(theta1)
    # theta2 = radians(theta2)
    # theta3 = radians(theta3)
    x = (math.cos(theta1) * (l1+l2*math.cos(theta2)))
    y = (math.sin(theta1) * (l1+l2*math.cos(theta2)))
    z = -math.sin(theta2)*l2 
    return [x, y, z]


def computeDKDetailed(theta1,theta2,theta3,l1=constL1,l2=constL2,l3=constL3,use_rads=USE_RADS_INPUT,use_mm=USE_MM_OUTPUT):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # TODO: terminer cette fonction
    p0 = [0,0,0]
    p1 = computeDKP1(theta1_verif,theta2_verif,theta3_verif,l1,l2,l3)
    p2 = computeDKP2(theta1_verif,theta2_verif,theta3_verif,l1,l2,l3)
    p3 = computeDK(theta1_verif,theta2_verif,theta3_verif,l1,l2,l3)
    return [p0, p1, p2, p3]


def rotaton_2D(x, y, z, leg_angle):

    x2 = math.cos(leg_angle) * x - math.sin(leg_angle) * y
    y2 = math.sin(leg_angle) * x + math.cos(leg_angle) * y
    return [x2, y2, z]



def computeIK(x,y,z,l1=constL1,l2=constL2,l3=constL3,verbose=False,use_rads=USE_RADS_OUTPUT,sign=-1,use_mm=USE_MM_INPUT):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit


    if y == 0 and x == 0:
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)


    xp = math.sqrt(x * x + y * y) - l1
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))

    theta2 = alkashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alkashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )

    return result
    
def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)
    

def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


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
    print(
        "rotaton_2D(0, 0, 0) = {}".format(
            rotaton_2D(0, 0, 0,leg_angle = 0)
           
        )
    )


if __name__ == "__main__":
    main()