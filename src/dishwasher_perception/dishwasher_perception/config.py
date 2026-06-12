
#create_single_depth_slice_mask CONSTANTS
NEAR_M = 0.3  # nearest distance from base_link, in meters.
FAR_M = 0.65  # farthest distance from base_link, in meters.
MIN_Z_M = 0.07  # minimum threshold for the height the door is after its opened.
MAX_Z_M = 0.2   # maximum threshold for the height the door is after its opened.

#create_filtered_depth_image CONSTANT
DISHWASHER_THICKNESS = 0.15   # thichness of dishwasher's body

#create_filtered_depth_image  CONSTANTS   
HEIGHT_MIN_M = 0.45  # how high is the bottom of rack?
HEIGHT_MAX_M = 0.7   # how high is the top of rack?

#create_depth_mask_from_min CONSTANTS  - created at the last step (after complete point cloud fitering)
MAX_DEPTH_MM = 50   # How far, from the nearest point do you want to create a mask?

#for markers (basket dimensions)
LEFT_DX  = 0.3
RIGHT_DX = 0.3
BACK_DY  = 0.6
FRONT_DY = 0.0
DOWN_DZ  = 0.2
UP_DZ    = 0.0

#for rack percentage
RACK_AND_DISTANCE_OFFSET = 0.06       #tuck the rack fully in and find how out it is when compared to distance measured by LIdar depth. Here is a detailed explanation: tuck the basket fully in, now measure the distance of dishwasher with LIDar, next get the depth of rack using get_pose, now, subtract 'pose.postion.x' from 'distance from LIDar' 
DEPTH_OF_DISHWASHER = 0.47   #depth of dishwasher's rack (width of rack, when measured from side)




# important stuff at iki robolab
# door length = 0.72
# stand robot at : 0.85
# ideal distance to capture door width: 0.3 to 0.65