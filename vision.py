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
    hullTable = None
    try:
        hullTable = NetworkTable.getTable("GRIP/goalConvexHulls")
    except KeyError:
        return None
    centerXs = NumberArray()
    centerYs = NumberArray()
    areas = NumberArray()
    heights = NumberArray()
    widths = NumberArray()
    try:
        hullTable.retrieveValue("centerX", centerXs)
        hullTable.retrieveValue("centerY", centerYs)
        hullTable.retrieveValue("area", areas)
        hullTable.retrieveValue("height", heights)
        hullTable.retrieveValue("width", widths)
    except KeyError:
        return None

    avgLen = (len(centerXs) + len(centerYs) + len(areas) + len(heights) + len(widths))/5
    if round(avgLen) != avgLen:  # It happens. I don't know why.
        return None

    contours = []
    for i in range(len(centerXs)):
        contours.append(Contour(centerXs[i], centerYs[i], areas[i], heights[i], widths[i]))
    if len(contours) < 1:
        return None  # Couldn't find any vision targets

    contours = sorted(contours, key=lambda x: x.area, reverse=True)  # Sort contours by area in descending size
    largest = contours[0]          # Maybe use width?

    dy = (config.image_height / 2) - largest.centerY

    # Calculations taken from TowerTracker

    x = largest.centerX
    x = (2 * (x / config.image_width))
    azimuth = (x*config.horiz_fov / 2.0) - 30

    dist = distanceFromTower(largest.width)
    return trigShootAngle(armAngle, dist), azimuth, dist, 0


def trigShootAngle(armAngle, dist):
    u = config.arm_len_to_camera * math.degrees(math.sin(armAngle))
    return math.degrees(math.atan2(config.center_target_height - config.arm_height - u, dist))


def calculateCameraAngle(armAngle):
    return armAngle - config.camera_angle_offset


def calculateCameraHeight(armAngle):
    return math.sin(math.radians(armAngle)) * config.arm_len_to_camera + config.arm_height


def angleFromGoalWidth(goalw):
    return -0.000020941*(goalw**3) + 0.0088104*(goalw**2) - 0.95513*goalw + 66.004


def distanceFromTower(goalw):
    goalw *= 2
    return 0.0068478*(goalw**2) - 2.9095*goalw + 346.76


def shootAngle(distance):
    return 0.001458*(distance**2) - 0.48272*distance + 75.393
