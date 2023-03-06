import concurrent
import time
from concurrent import futures

from controller import Robot
import math
import threading


# TIME_STEP = 64
# robot = Robot()
# ds = []
# dsNames = ['ds_left', 'ds_right']
# for i in range(len(dsNames)):
#     ds.append(robot.getDevice(dsNames[i]))
#     ds[i].enable(TIME_STEP)
#
# wheels = []
# wheelsNames = ['wheel_f_l', 'wheel_f_r', 'wheel_b_l', 'wheel_b_r']
#
#
# baseMotor = robot.getDevice('armBaseMotor')
# baseMotor.setPosition(float('inf'))
# baseMotor.setVelocity(0.0)
#
# baseAngleSensor = robot.getDevice('baseAngleSensor')
# baseAngleSensor.enable(TIME_STEP)
#
# armBaseMotor = robot.getDevice('upperArmMotor')
# armBaseMotor.setPosition(float('inf'))
# armBaseMotor.setVelocity(0.0)
#

#python function to caclulate the cross points of 2 circles
def circle_circle_intersection(circle1, circle2):
    x1, y1, r1 = circle1
    x2, y2, r2 = circle2
    d = math.hypot(x2 - x1, y2 - y1)
    if d > r1 + r2:
        print("no solution")
        return None
    if d < abs(r1 - r2):
        print("no solution")
        return None
    if d == 0 and r1 == r2:
        print("no solution")
        return None
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)
    xm = x1 + a * (x2 - x1) / d
    ym = y1 + a * (y2 - y1) / d
    xs1 = xm + h * (y2 - y1) / d
    xs2 = xm - h * (y2 - y1) / d
    ys1 = ym - h * (x2 - x1) / d
    ys2 = ym + h * (x2 - x1) / d
    # return (xs1, ys1), (xs2, ys2)
    if ys1 > ys2:
        return xs1, ys1
    else:
        return xs2, ys2

def getAngle(x1, y1, x2, y2, x3, y3):
    #calculating the length of the sides of the triangle
    a = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    b = math.sqrt((x3-x2)**2 + (y3-y2)**2)
    c = math.sqrt((x3-x1)**2 + (y3-y1)**2)

    #calculating the angle
    angle = math.acos((a**2 + b**2 - c**2)/(2*a*b))
    return math.degrees(angle)

#python function to calculate the angle between two points relative to an origin without giving zerodivision error
def angle_between_points(x1, y1, x2, y2, x3, y3):
    #calculate the angle between the two points
    angle1 = math.atan2(y1-y3, x1-x3)
    angle2 = math.atan2(y2-y3, x2-x3)
    angle = math.degrees(angle1 - angle2)
    #normalize the angle
    return angle % 360

def list_to_matrix(lst):
    matrix = []
    for i in range(64):
        row = lst[i*64:(i+1)*64]
        matrix.append(row)
    return matrix

# armAngleSensor = robot.getDevice('upperArmPositionSensor')
# armAngleSensor.enable(TIME_STEP)
#
# lowerArmMotor = robot.getDevice('lowerArmMotor')
# lowerArmMotor.setPosition(float('inf'))
# lowerArmMotor.setVelocity(0.0)
#
# lowerArmPositionSensor = robot.getDevice('lowerArmPositionSensor')
# lowerArmPositionSensor.enable(TIME_STEP)
#
# wristMotor = robot.getDevice('wristMotor')
# wristMotor.setPosition(float('inf'))
# wristMotor.setVelocity(0.0)
#
# wristPositionSensor = robot.getDevice('wristPositionSensor')
# wristPositionSensor.enable(TIME_STEP)
#
# handMotor = robot.getDevice('handMotor')
# handMotor.setPosition(float('inf'))
# handMotor.setVelocity(0.0)
#
# handPositionSensor = robot.getDevice('handPositionSensor')
# handPositionSensor.enable(TIME_STEP)
#
# rightFingerMotor = robot.getDevice('rightFingerMotor')
# rightFingerMotor.setPosition(0.04)
#
# leftFingerMotor = robot.getDevice('leftFingerMotor')
# leftFingerMotor.setPosition(-0.04)
#
#
# def move(speed):
#     wheels[0].setVelocity(speed)
#     wheels[1].setVelocity(speed)
#     wheels[2].setVelocity(speed)
#     wheels[3].setVelocity(speed)
#
#
# def turnLeft():
#     wheels[0].setVelocity(0)
#     wheels[1].setVelocity(10)
#     wheels[2].setVelocity(0)
#     wheels[3].setVelocity(10)
#
#
#
#
#
# def radians_to_degrees(radians):
#     degrees = math.degrees(radians) % 360
#     return degrees if degrees >= 0 else degrees + 360
#
#
#
# for i in range(len(wheelsNames)):
#     wheels.append(robot.getDevice(wheelsNames[i]))
#     wheels[i].setPosition(float('inf'))
#     wheels[i].setVelocity(0.0)


def radians_to_degrees(radians):
    degrees = math.degrees(radians) % 360
    return degrees if degrees >= 0 else degrees + 360


def add_subtract_angles(angle1, angle2):
    return (angle1 + angle2) % 360

def angle_difference(angle1, angle2):
    diff = abs(angle1 - angle2) % 360
    x = min(diff, 360 - diff)
    if 0 <= angle2 < 180 and 180 <= angle1 < 360:
        return -1 * x
    elif 0 <= angle1 < 180 and 180 <= angle2 < 360:
        return x
    elif angle1 > angle2:
        return x
    else:
        return -1*x

def get_row(matrix, row_num):
    # Get the size of the matrix
    matrix_size = int(len(matrix) ** 0.5)

    # Calculate the starting and ending index of the specified row
    start_index = row_num * matrix_size
    end_index = start_index + matrix_size

    # Return the list of numbers in the specified row
    return matrix[start_index:end_index]

def map_angle(angle):
    if angle == 0:
        return 0
    elif angle == 180 or angle == -180:
        return 180
    elif angle >= 0:
        return (360 - angle) % 180 + 180
    else:
        return (-angle) % 180


def wheel_distance(angle, radius):
    # convert angle to radians

    # calculate linear distance moved
    distance = angle * radius

    return distance

# def run_functions_concurrently(functions):
#     """Run a list of functions concurrently using threads."""
#     threads = []
#     for function in functions:
#         thread = threading.Thread(target=function[0], args=function[1])
#         threads.append(thread)
#         thread.start()
#
#     for thread in threads:
#         thread.join()


class MyRobot:
    def __init__(self):
        self.robot = Robot()
        self.timeStep = 32
        self.leftFrontMotor = self.robot.getDevice('wheel_f_l')
        self.rightFrontMotor = self.robot.getDevice('wheel_f_r')
        self.leftBackMotor = self.robot.getDevice('wheel_b_l')
        self.rightBackMotor = self.robot.getDevice('wheel_b_r')
        self.leftBackMotor.setPosition(float('inf'))
        self.rightBackMotor.setPosition(float('inf'))
        self.leftFrontMotor.setPosition(float('inf'))
        self.rightFrontMotor.setPosition(float('inf'))
        self.leftBackMotor.setVelocity(0.0)
        self.rightBackMotor.setVelocity(0.0)
        self.leftFrontMotor.setVelocity(0.0)
        self.rightFrontMotor.setVelocity(0.0)
        self.ds = []
        self.dsNames = ['ds_left', 'ds_right']
        for i in range(len(self.dsNames)):
            self.ds.append(self.robot.getDevice(self.dsNames[i]))
            self.ds[i].enable(self.timeStep)
        self.rangeFinder = self.robot.getDevice('rangeFinder')
        self.rangeFinder.enable(self.timeStep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timeStep)
        self.positionSensorLeft = self.robot.getDevice('positionSensorFrontLeft')
        self.positionSensorRight = self.robot.getDevice('positionSensorFrontRight')
        self.positionSensorLeft.enable(self.timeStep)
        self.positionSensorRight.enable(self.timeStep)


    def getDistanceSensorReading(self, sensor):
        if sensor == 'left':
            left = 0
            while self.robot.step(self.timeStep) != -1:
                left = self.ds[0].getValue()
                break
            return left
        else:
            right = 0
            while self.robot.step(self.timeStep) != -1:
                right = self.ds[1].getValue()
                break
            return right


    def getMoveDistance(self):
        radius = 3.5
        left = 0
        right = 0
        while self.robot.step(self.timeStep) != -1:
            left = self.positionSensorLeft.getValue()
            right = self.positionSensorRight.getValue()
            break

        return (left + right) / 2

    def move(self, leftSpeed, rightSpeed):
        self.leftFrontMotor.setVelocity(leftSpeed)
        self.rightFrontMotor.setVelocity(rightSpeed)
        self.leftBackMotor.setVelocity(leftSpeed)
        self.rightBackMotor.setVelocity(rightSpeed)

    def get_bearing_in_degrees(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0.0:
            bearing += 360.0
        return bearing

    def rotateLeft(self,angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = self.get_bearing_in_degrees()
            break
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(self.get_bearing_in_degrees(),-current))
            if difference > 180:
                difference = 360 - difference
            print(difference)
            if angle - 5 < difference < angle:
                self.move(-0.5,0.5)
            else:
                self.move(-1,1)
            if difference >= angle:
                self.move(0,0)
                break

    def rotateRight(self,angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = self.get_bearing_in_degrees()
            break
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(self.get_bearing_in_degrees(),-current))
            if difference > 180:
                difference = 360 - difference
            print(difference)
            if angle - 5 < difference < angle:
                self.move(0.5,-0.5)
            else:
                self.move(4,-4)
            if difference >= angle:
                self.move(0,0)
                break

    def getPixelDistance(self,x,y):
        currentRow = 0
        while self.robot.step(self.timeStep) != -1:
            currentRow = get_row(self.rangeFinder.getRangeImage(),32)
            break
        return currentRow[y] * 100

    def getBoxCenter(self):
        while robot.robot.step(robot.timeStep) != -1:
            range_values = list_to_matrix(robot.rangeFinder.getRangeImage())[48]
            indexes = [i for i, x in enumerate(range_values) if x < 0.06]
            if len(indexes) == 0:
                return None
            distances = [range_values[i] for i in indexes]
            x = sum(distances) / len(distances)
            p = x / 32
            ia = min(indexes)
            ib = max(indexes)
            z = (((ia + ib) / 2) - 32) * p
            return ((x + 0.075) * 100, z * 100)

    def getBoxType(self):
        while robot.robot.step(robot.timeStep) != -1:
            range_values = list_to_matrix(robot.rangeFinder.getRangeImage())[48]
            indexes = [i for i, x in enumerate(range_values) if x < 0.06]
            if len(indexes) == 0:
                return None
            distances = [range_values[i] for i in indexes]
            x = sum(distances) / len(distances)
            p = x / 32
            ia = min(indexes)
            ib = max(indexes)
            za = ia - 32
            zb = ib - 32
            z = (zb - za) * p * 100
            if 0 < z < 3.5:
                return 's'
            elif 3.5 <= z < 4.5:
                return 'm'
            else:
                return 'l'

    def stop(self):
        self.move(0,0)

    def moveDistance(self,d):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = self.getMoveDistance()
            break
        while self.robot.step(self.timeStep) != -1:
            print(wheel_distance(self.getMoveDistance() - current,3.5))
            if wheel_distance(self.getMoveDistance() - current,3.5) > d:
                self.move(0,0)
                break
            else:
                self.move(4,4)

class MyRobotArm:
    def __init__(self, myRobot):
        self.robot = myRobot
        self.timeStep = 32
        self.baseMotor = self.robot.getDevice('armBaseMotor')
        self.baseMotor.setPosition(float('inf'))
        self.baseMotor.setVelocity(0.0)
        self.baseAngleSensor = self.robot.getDevice('baseAngleSensor')
        self.baseAngleSensor.enable(self.timeStep)
        self.upperArmMotor = self.robot.getDevice('upperArmMotor')
        self.upperArmMotor.setPosition(float('inf'))
        self.upperArmMotor.setVelocity(0.0)
        self.upperArmPositionSensor = self.robot.getDevice('upperArmPositionSensor')
        self.upperArmPositionSensor.enable(self.timeStep)
        self.lowerArmMotor = self.robot.getDevice('lowerArmMotor')
        self.lowerArmMotor.setPosition(float('inf'))
        self.lowerArmMotor.setVelocity(0.0)
        self.lowerArmPositionSensor = self.robot.getDevice('lowerArmPositionSensor')
        self.lowerArmPositionSensor.enable(self.timeStep)
        self.wristMotor = self.robot.getDevice('wristMotor')
        self.wristMotor.setPosition(float('inf'))
        self.wristMotor.setVelocity(0.0)
        self.wristPositionSensor = self.robot.getDevice('wristPositionSensor')
        self.wristPositionSensor.enable(self.timeStep)
        self.handMotor = self.robot.getDevice('handMotor')
        self.handMotor.setPosition(float('inf'))
        self.handMotor.setVelocity(0.0)
        self.handPositionSensor = self.robot.getDevice('handPositionSensor')
        self.handPositionSensor.enable(self.timeStep)
        self.rightFingerMotor = self.robot.getDevice('rightFingerMotor')
        self.rightFingerMotor.setPosition(0.0)
        self.leftFingerMotor = self.robot.getDevice('leftFingerMotor')
        self.leftFingerMotor.setPosition(0.0)
        self.touchSensorLeft = self.robot.getDevice('touchSensorLeft')
        self.touchSensorLeft.enable(self.timeStep)
        self.touchSensorRight = self.robot.getDevice('touchSensorRight')
        self.touchSensorRight.enable(self.timeStep)
        self.left = 0.0
        self.right = 0.0
        self.currentBox = None

    def moveFingersIn(self):
        while self.robot.step(self.timeStep) != -1:
            leftTouch = self.touchSensorLeft.getValue()
            rightTouch = self.touchSensorRight.getValue()
            if leftTouch == 0 and rightTouch == 0:
                self.left -= 0.001
                self.right += 0.001
                self.leftFingerMotor.setPosition(self.left)
                self.rightFingerMotor.setPosition(self.right)
            elif leftTouch == 1 and rightTouch == 0:
                self.right += 0.001
                self.rightFingerMotor.setPosition(self.right)
            elif leftTouch == 0 and rightTouch == 1:
                self.left -= 0.001
                self.leftFingerMotor.setPosition(self.left)
            else:
                break

            # if self.left < -0.04 or self.right > 0.04:
            #     break
            # self.left -= 0.001
            # self.right += 0.001
            # self.leftFingerMotor.setPosition(self.left)
            # self.rightFingerMotor.setPosition(self.right)

    def moveFingersOut(self):
        while self.robot.step(self.timeStep) != -1:
            if self.left > 0.0 or self.right < 0.0:
                break
            self.left += 0.001
            self.right -= 0.001
            self.leftFingerMotor.setPosition(self.left)
            self.rightFingerMotor.setPosition(self.right)

    def rotateHandAnticlockwise(self, angle):
        self.handMotor.setVelocity(1.0)
        while self.robot.step(self.timeStep) != -1:
            if radians_to_degrees(self.handPositionSensor.getValue()) > angle:
                break
        self.handMotor.setVelocity(0.0)

    def resetHand(self):
        self.handMotor.setVelocity(-1.0)
        while self.robot.step(self.timeStep) != -1:
            if radians_to_degrees(self.handPositionSensor.getValue()) < 5:
                self.handMotor.setVelocity(-0.05)
            val = radians_to_degrees(self.handPositionSensor.getValue())
            if round(val, 1) == 0.1:
                break
        self.handMotor.setVelocity(0.0)

    def moveWrists(self, angle):
        if angle > 0:
            self.wristMotor.setVelocity(1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.wristPositionSensor.getValue()) > angle:
                    break
            self.wristMotor.setVelocity(0.0)
        else:
            self.wristMotor.setVelocity(-1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.wristPositionSensor.getValue()) < 360 + angle:
                    break
            self.wristMotor.setVelocity(0.0)

    def resetWrist(self):
        if radians_to_degrees(self.wristPositionSensor.getValue()) > 180:
            self.wristMotor.setVelocity(1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.wristPositionSensor.getValue()) > 355:
                    self.wristMotor.setVelocity(0.05)
                if round(radians_to_degrees(self.wristPositionSensor.getValue()), 1) == 0.1:
                    break
            self.wristMotor.setVelocity(0.0)
        else:
            self.wristMotor.setVelocity(-1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.wristPositionSensor.getValue()) < 5:
                    self.wristMotor.setVelocity(-0.05)
                val = radians_to_degrees(self.wristPositionSensor.getValue())
                if round(val, 1) == 0.1:
                    break
            self.wristMotor.setVelocity(0.0)

    def moveLowerArm(self, angle):
        if angle > 0:
            self.lowerArmMotor.setVelocity(1.0)
            while self.robot.step(self.timeStep) != -1:
                print(radians_to_degrees(self.lowerArmPositionSensor.getValue()))
                if radians_to_degrees(self.lowerArmPositionSensor.getValue()) > angle:
                    break
            self.lowerArmMotor.setVelocity(0.0)
        else:
            self.lowerArmMotor.setVelocity(-1.0)
            while self.robot.step(self.timeStep) != -1:
                print(radians_to_degrees(self.lowerArmPositionSensor.getValue()))
                if radians_to_degrees(self.lowerArmPositionSensor.getValue()) < 360 + angle:
                    break
            self.lowerArmMotor.setVelocity(0.0)

    def resetLowerArm(self):
        if radians_to_degrees(self.lowerArmPositionSensor.getValue()) > 180:
            self.lowerArmMotor.setVelocity(1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.lowerArmPositionSensor.getValue()) > 355:
                    self.lowerArmMotor.setVelocity(0.05)
                if round(radians_to_degrees(self.lowerArmPositionSensor.getValue()), 1) == 0.1:
                    break
            self.lowerArmMotor.setVelocity(0.0)
        else:
            self.lowerArmMotor.setVelocity(-1.0)
            while self.robot.step(self.timeStep) != -1:
                if radians_to_degrees(self.lowerArmPositionSensor.getValue()) < 5:
                    self.lowerArmMotor.setVelocity(-0.05)
                val = radians_to_degrees(self.lowerArmPositionSensor.getValue())
                if round(val, 1) == 0.1:
                    break
            self.lowerArmMotor.setVelocity(0.0)

    def moveUpperArm(self, angle):
        angle = angle * -1
        self.upperArmMotor.setVelocity(-1.0)
        while self.robot.step(self.timeStep) != -1:
            print(radians_to_degrees(self.upperArmPositionSensor.getValue()))
            if radians_to_degrees(self.upperArmPositionSensor.getValue()) < 360 + angle:
                break
        self.upperArmMotor.setVelocity(0.0)

    def resetUpperArm(self):
        self.upperArmMotor.setVelocity(1.0)
        while self.robot.step(self.timeStep) != -1:
            if radians_to_degrees(self.upperArmPositionSensor.getValue()) > 355:
                self.upperArmMotor.setVelocity(0.05)
            if round(radians_to_degrees(self.upperArmPositionSensor.getValue()), 1) == 0.1:
                break
        self.upperArmMotor.setVelocity(0.0)

    def moveBase(self, angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = radians_to_degrees(self.baseAngleSensor.getValue())
            break
        k = -1
        if angle < 0:
            k = 1
            angle = angle * -1
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(current, -1*radians_to_degrees(self.baseAngleSensor.getValue())))
            if difference > 180:
                difference = 360 - difference
            if difference < angle:
                if difference > angle - 5:
                    self.baseMotor.setVelocity(0.05*k)
                else:
                    self.baseMotor.setVelocity(0.5*k)
            else:
                self.baseMotor.setVelocity(0.0)
                break

    def rotateUpperArm(self, angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = radians_to_degrees(self.upperArmPositionSensor.getValue())
            break
        k = -1
        if angle < 0:
            k = 1
            angle = angle * -1
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(current, -1*radians_to_degrees(self.upperArmPositionSensor.getValue())))
            if difference > 180:
                difference = 360 - difference
            if difference < angle:
                if difference > angle - 5:
                    self.upperArmMotor.setVelocity(0.05*k)
                else:
                    self.upperArmMotor.setVelocity(0.5*k)
            else:
                self.upperArmMotor.setVelocity(0.0)
                break




        # current = 0
        # while self.robot.step(self.timeStep) != -1:
        #     current = radians_to_degrees(self.upperArmPositionSensor.getValue())
        #     if 0 < current < 2:
        #         current = 360
        #     break
        # alpha = current - angle
        # if alpha < current:
        #     self.upperArmMotor.setVelocity(-0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.upperArmPositionSensor.getValue())
        #         if val < alpha + 5:
        #             self.upperArmMotor.setVelocity(-0.05)
        #         if round(val) <= round(alpha):
        #             break
        #     self.upperArmMotor.setVelocity(0.0)
        # else:
        #     self.upperArmMotor.setVelocity(0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.upperArmPositionSensor.getValue())
        #         if val > alpha - 5:
        #             self.upperArmMotor.setVelocity(0.05)
        #         if round(val) >= round(alpha):
        #             break
        #     self.upperArmMotor.setVelocity(0.0)

    def rotateLowerArm(self, angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = radians_to_degrees(self.lowerArmPositionSensor.getValue())
            break
        k = -1
        if angle < 0:
            k = 1
            angle = angle * -1
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(current, -1*radians_to_degrees(self.lowerArmPositionSensor.getValue())))
            if difference > 180:
                difference = 360 - difference
            if difference < angle:
                if difference > angle - 5:
                    self.lowerArmMotor.setVelocity(k*0.05)
                else:
                    self.lowerArmMotor.setVelocity(k*0.5)
            else:
                self.lowerArmMotor.setVelocity(0.0)
                break


        # angle = angle * -1
        # current = 0
        # while self.robot.step(self.timeStep) != -1:
        #     current = radians_to_degrees(self.lowerArmPositionSensor.getValue())
        #     break
        # alpha = add_subtract_angles(current, angle)
        # if angle < 0:
        #     self.lowerArmMotor.setVelocity(-0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.lowerArmPositionSensor.getValue())
        #         if val < add_subtract_angles(alpha, 5):
        #             self.lowerArmMotor.setVelocity(-0.05)
        #         if round(val) <= round(alpha):
        #             break
        #     self.lowerArmMotor.setVelocity(0.0)
        # else:
        #     self.lowerArmMotor.setVelocity(0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.lowerArmPositionSensor.getValue())
        #         if val > add_subtract_angles(alpha, -5):
        #             self.lowerArmMotor.setVelocity(0.05)
        #         if round(val) >= round(alpha):
        #             break
        #     self.lowerArmMotor.setVelocity(0.0)

    def rotateWrist(self, angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = radians_to_degrees(self.wristPositionSensor.getValue())
            break
        k = -1
        if angle < 0:
            k = 1
            angle = angle * -1
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(current, -1*radians_to_degrees(self.wristPositionSensor.getValue())))
            if difference > 180:
                difference = 360 - difference
            if difference < angle:
                if difference > angle - 5:
                    self.wristMotor.setVelocity(k*0.05)
                else:
                    self.wristMotor.setVelocity(k*0.5)
            else:
                self.wristMotor.setVelocity(0.0)
                break

        # angle = angle * -1
        # current = 0
        # while self.robot.step(self.timeStep) != -1:
        #     current = radians_to_degrees(self.wristPositionSensor.getValue())
        #     break
        # alpha = add_subtract_angles(current, angle)
        # if angle < 0:
        #     self.wristMotor.setVelocity(-0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.wristPositionSensor.getValue())
        #         if val < add_subtract_angles(alpha, 5):
        #             self.wristMotor.setVelocity(-0.05)
        #         if round(val) <= round(alpha):
        #             break
        #     self.wristMotor.setVelocity(0.0)
        # else:
        #     self.wristMotor.setVelocity(0.5)
        #     while self.robot.step(self.timeStep) != -1:
        #         val = radians_to_degrees(self.wristPositionSensor.getValue())
        #         if val > add_subtract_angles(alpha, -5):
        #             self.wristMotor.setVelocity(0.05)
        #         if round(val) >= round(alpha):
        #             break
        #     self.wristMotor.setVelocity(0.0)

    def rotateHand(self,angle):
        current = 0
        while self.robot.step(self.timeStep) != -1:
            current = radians_to_degrees(self.handPositionSensor.getValue())
            break
        k = -1
        if angle < 0:
            k = 1
            angle = angle * -1
        while self.robot.step(self.timeStep) != -1:
            difference = abs(add_subtract_angles(current, -1*radians_to_degrees(self.handPositionSensor.getValue())))
            if difference > 180:
                difference = 360 - difference
            if difference < angle:
                if difference > angle - 5:
                    self.handMotor.setVelocity(k*0.05)
                else:
                    self.handMotor.setVelocity(k*0.5)
            else:
                self.handMotor.setVelocity(0.0)
                break

    def take_actions(self, functions):
        threads = []
        for function in functions:
            thread = threading.Thread(target=function[0], args=(function[1],))
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

    def bendArm(self, angles):
        actions = [self.moveBase, self.rotateUpperArm, self.rotateLowerArm,
                   self.rotateWrist, self.rotateHand]
        result = []
        for i in range(len(angles)):
            if angles[i] != 0:
                result.append((actions[i], angles[i]))
        self.take_actions(result)

    def bendArmTo(self,angles):
        move = [0,0,0,0,0]
        currentUpperArm = 0
        currentLowerArm = 0
        currentWrists = 0
        currentHand = 0
        currentBase = 0
        while self.robot.step(self.timeStep) != -1:
            currentUpperArm = radians_to_degrees(self.upperArmPositionSensor.getValue())
            currentLowerArm = radians_to_degrees(self.lowerArmPositionSensor.getValue())
            currentWrists = radians_to_degrees(self.wristPositionSensor.getValue())
            currentHand = radians_to_degrees(self.handPositionSensor.getValue())
            currentBase = radians_to_degrees(self.baseAngleSensor.getValue())

            break
        move[0] = angle_difference(currentBase, map_angle(angles[0]))
        move[1] = angle_difference(currentUpperArm, map_angle(angles[1]))
        move[2] = angle_difference(currentLowerArm, map_angle(angles[2]))
        move[3] = angle_difference(currentWrists, map_angle(angles[3]))
        move[4] = angle_difference(currentHand, map_angle(angles[4]))
        print(move)
        self.bendArm(move)

    def moveArm(self, x, y, z, direction):
        k = 1
        if y < 0:
            k = -1
        theta = math.degrees(math.atan2(abs(z),x))
        theta = theta if z > 0 else -theta
        x1, y1, r1 = 0, 0, 7.5
        x2, y2, r2 = x, y, 7.5
        x, y = circle_circle_intersection((x1, y1, r1), (x2, y2, r2))
        alpha = abs(getAngle(x2, y2, x1, y1, x, y)) + k * getAngle(x2, 0, 0, 0, x2, y2)
        beta = abs(180 - getAngle(x2, y2, x, y, x1, y1))
        gamma = abs(180 - getAngle(x2, -100, x2, y2, x, y))
        delta = abs(90 - gamma)
        if direction == 'v':
            self.bendArmTo([theta, alpha, -beta, -gamma, theta])
        else:
            self.bendArmTo([theta, alpha, -beta, delta, 0])

    def moveToDefault(self):
        self.moveArm(3, 5, 0, 'h')

    def pickUpBoxFromFloor(self):
        center = robot.getBoxCenter()
        print(center)
        if center is None:
            return
        boxType = robot.getBoxType()
        x,z = center
        y = 0
        if boxType == 'l':
            x+=2
            self.currentBox = 'l'
        elif boxType == 'm':
            x+=1
            y = -1.5
            self.currentBox = 'm'
        else:
            y = -1.5
            x+=0.5
            self.currentBox = 's'
        direction = 'v'
        self.moveArm(x,y,z,direction)
        self.moveFingersIn()
        self.moveArm(x,y+2,z,'v')
        self.moveToDefault()

    def putBoxOnFloor(self,x,z):
        y = 0
        if self.currentBox != 'l':
            y = -1.5
        self.moveArm(x,y,z,'v')
        self.moveFingersOut()
        self.moveArm(x,y+5,z,'v')
        self.moveToDefault()

robot = MyRobot()
robotArm = MyRobotArm(robot.robot)
robotArm.pickUpBoxFromFloor()
robot.moveDistance(100)
robot.rotateLeft(90)
robot.moveDistance(100)
robotArm.putBoxOnFloor(13,0)
# robotArm.bendArm()
# x,z = robot.getBoxCenter()
# x+=2
# y = 0
# direction = 'v'
# robotArm.moveArm(x,y,z,direction)




# robot.rotateRight(45)

# while robot.robot.step(robot.timeStep) != -1:
#     robot.move(5,5)
#     print(robot.get_bearing_in_degrees())


# robotArm.rotateUpperArm(45)
# robotArm.rotateLowerArm(-45)
# robotArm.rotateWrist(-90)

# robotArm.moveBase(-45)


# robotArm.take_actions([(robotArm.rotateLowerArm,(-30)), (robotArm.rotateWrist,(30))])
# robotArm.moveFingersIn()
# robotArm.take_actions([(robotArm.rotateLowerArm,(30)), (robotArm.rotateWrist,(-30))])
#
# robot.move(5)


# functions1 = [(robotArm.rotateLowerArm, (-30)), (robotArm.rotateWrist, (30))]
# functions2 = [(robotArm.rotateLowerArm, (30)), (robotArm.rotateWrist, (-30))]
# run_functions_concurrently(functions1)
# robotArm.moveFingersIn()
# run_functions_concurrently(functions2)
# robot.move(5)

# thread1 = threading.Thread(target=robotArm.rotateLowerArm, args=(-30,))
# thread2 = threading.Thread(target=robotArm.rotateWrist, args=(30,))
#
# thread1.start()
# thread2.start()
#
# thread1.join()
# thread2.join()
# robotArm.moveFingersIn()
#
# thread1 = threading.Thread(target=robotArm.rotateLowerArm, args=(30,))
# thread2 = threading.Thread(target=robotArm.rotateWrist, args=(-30,))
#
# thread1.start()
# thread2.start()
#
# thread1.join()
# thread2.join()
#
# robot.move(5)

# robotArm.moveUpperArm(20)
# robotArm.moveLowerArm(20)
# robotArm.moveWrists(90)
# robotArm.moveFingersIn()
# start_time = time.time()  # Get the current time in seconds
# time_limit = 5.0  # Set the time limit for the loop (in seconds)
# while robot.robot.step(robot.timeStep) != -1:
#     robot.move(-5)
#     # Check if time limit has been reached
#     if time.time() - start_time >= time_limit:
#         print("Time limit reached, breaking loop")
#         break
# robot.move(0)
# robotArm.resetLowerArm()
# robotArm.resetUpperArm()
# robotArm.moveFingersOut()
# robotArm.handMotor.setVelocity(1.0)

# while robot.robot.step(robot.timeStep) != -1:
#     print(round(radians_to_degrees(robotArm.handPositionSensor.getValue()),2))
