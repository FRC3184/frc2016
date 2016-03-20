import math
import config
from networktables import NumberArray, NetworkTable


class Contour:
    def __init__(self, centerX, centerY, area, height, width):
        self.centerX = centerX
        self.centerY = centerY
        self.area = area
        self.height = height
        self.width = width


def calculateShooterParams(armAngle):
    hullTable = NetworkTable.getTable("GRIP/goalConvexHulls")
    centerXs = NumberArray()
    centerYs = NumberArray()
    areas = NumberArray()
    heights = NumberArray()
    widths = NumberArray()
    hullTable.retrieveValue("centerX", centerXs)
    hullTable.retrieveValue("centerY", centerYs)
    hullTable.retrieveValue("area", areas)
    hullTable.retrieveValue("height", heights)
    hullTable.retrieveValue("width", widths)

    avgLen = (len(centerXs) + len(centerYs) + len(areas) + len(heights) + len(widths))/5
    if round(avgLen) != avgLen:  # It happens. I don't know why.
        return None

    contours = []
    for i in range(len(centerXs)):
        contours.append(Contour(centerXs[i], centerYs[i], areas[i], heights[i], widths[i]))
    if len(contours) < 1:
        return None  # Couldn't find any vision targets

    contours = sorted(contours, key=lambda x: x.area, reverse=True)  # Sort contours by area in descending size
    largest = contours[0]                                            # Maybe use width?

    # Calculations taken from TowerTracker
    y = largest.centerY + largest.height / 2.0
    y = -((2*(y/config.image_height))-1)
    distance = (config.top_target_height - calculateCameraHeight(armAngle)) / \
               (math.tan(math.radians(y * config.vertical_fov / 2.0 + calculateCameraAngle(armAngle))))
    x = largest.centerX + largest.width / 2.0
    x = (2 * (x / config.image_width)) - 1
    azimuth = x*config.horiz_fov / 2.0

    return shootAngle(distance), azimuth, distance, 0


def calculateCameraAngle(armAngle):
    return armAngle - config.camera_angle_offset


def calculateCameraHeight(armAngle):
    return math.sin(math.radians(armAngle)) * config.arm_len_to_camera + config.arm_height


def angleFromGoalWidth(goalw):
    return -0.000020941*(goalw**3) + 0.0088104*(goalw**2) - 0.95513*goalw + 66.004


def distanceFromTower(goalw):
    return 0.0068478*(goalw**2) - 2.9095*goalw + 346.76


def shootAngle(distance):
    return 0.001458*(distance**2) - 0.48272*distance + 75.393
