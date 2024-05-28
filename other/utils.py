import math

def euclidean_distance(pose1, pose2):
    return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)

def angle_difference(angle1, angle2):
    diff = angle1 - angle2
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff
