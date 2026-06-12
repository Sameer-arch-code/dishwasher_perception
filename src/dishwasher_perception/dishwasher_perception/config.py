#create_single_depth_slice_mask CONSTANTS

#important stuff
# door length = 0.72
# stand robot at : 0.85

#ideal distance to capture door width: 0.3 to 0.65

NEAR_M = 0.3
FAR_M = 0.65
MIN_Z_M = 0.07
MAX_Z_M = 0.2



#create_filtered_depth_image CONSTANT

DISHWASHER_THICKNESS = 0.15


#create_filtered_depth_image  CONSTANTS   


HEIGHT_MIN_M = 0.45
HEIGHT_MAX_M = 0.7

#create_depth_mask_from_min CONSTANTS  - created at the last step (after complete point cloud fitering)

MAX_DEPTH_MM = 50


#for markers (basket dimensions)

LEFT_DX  = 0.3
RIGHT_DX = 0.3
BACK_DY  = 0.6
FRONT_DY = 0.0
DOWN_DZ  = 0.2
UP_DZ    = 0.0


#for rack percentage

RACK_AND_DISTANCE_OFFSET = 0.06       #tuck the rack fully in and find how out it is, when compared to inner most LIdar depth (hard to describe XD. Check the readMe)
DEPTH_OF_DISHWASHER = 0.47