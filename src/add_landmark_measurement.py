import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2). L(2) is at 4.0, 2.0, so:
   
    poses=gtsam.utilities.allPose2s(result)
    poseX4=poses.atPose2(X(4))

    landmarks : np.ndarray = gtsam.utilities.extractPoint2(result) 
    pointL2=landmarks[1]
    pointL2_x=pointL2[0]
    pointL2_y=pointL2[1]
    landmark_point=gtsam.Point2(pointL2_x, pointL2_y) 
    
    rotation=poseX4.bearing(landmark_point)
    distance=poseX4.range(landmark_point)
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), rotation, distance, MEASUREMENT_NOISE))
    return graph