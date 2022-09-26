import argparse
from math import cos, sin, pi

D = 1.0

def driveMotors(inL, inR, outL_1, outR_1, theta_1, x_1, y_1):
    outL = 0.5*inL + 0.5*outL_1
    outR = 0.5*inR + 0.5*outR_1

    lin = (outL + outR)/2
    x = x_1 + lin*cos(theta_1)
    y = y_1 + lin*sin(theta_1)

    rot = (outR - outL)/D
    theta = theta_1 + rot

    return x, y, theta, outL, outR

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('inL', type=float)
    parser.add_argument('inR', type=float)
    args = parser.parse_args()

    inL, inR = args.inL, args.inR

    counter = 0
    theta = 0.0
    outL = 0.0
    outR = 0.0
    x = 0.0
    y = 0.0
    while theta < pi/2:
        counter += 1
        x, y, theta, outL, outR = driveMotors(inL, inR, outL, outR, theta, x, y)
        print(theta)
    counter -= 1

    print('x:', x, 'y:', y)
    print("Counter:", counter)
