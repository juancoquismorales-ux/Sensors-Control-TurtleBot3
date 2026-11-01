import numpy as np
import mujoco


#Return a ideal scan with 6m lim
def read_lidar(model, data, max_range=6.0):
# read all object positions to gather a curent sim state
    mujoco.mj_kinematics(model, data)
# find the frame from which the lidar scans on the turtlebot
    scanner = data.site("lidar_origin") 
    #beam origin world coords
    origin = scanner.xpos.copy()
    # sensor to world origin
    rotation = scanner.xmat.reshape(3, 3).copy()
    #just 1 beam per degree
    angles = np.deg2rad(np.arange(0, 360, 1))
    #create this to see if detection is in range
    ranges = np.full(angles.shape, np.inf)

#
    groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    base_id = model.body("base").id #identify robot base for exclusion
    hit_id = np.empty(1, dtype=np.int32)# storage for intersected obj ID

# Measure each direnction around the scanner
    for i, angle in enumerate(angles): 
        #week 4 horizontal beam
        direction_sensor = np.array([np.cos(angle), np.sin(angle), 0.0])
        # Transform beam into world coords
        direction_world = rotation @ direction_sensor 

        # find the nearest surface and include fixed objects in the room
        #whilst excluding the tb base
        distance = mujoco.mj_ray(model, data, origin, direction_world, groups,
                                  1, base_id, hit_id)
        # detections must be within out specified range
        if 0.0 <= distance <= max_range: 
            ranges[i] = distance # store the measured distance 

    return angles, ranges, float(data.time) # return angles, distances 
# and return sim timestamp

