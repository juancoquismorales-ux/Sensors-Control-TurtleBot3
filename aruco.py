import cv2 # using week3s method of creating an aruco marker
from pathlib import Path

#Code is based on week 3 aruco marker detection code, 
# but now we are using the mujoco camera to get the image 
#the aruco marker dictionary is set to 6x6 with 250 markers 
# which is different to the 4x4 with 50 markers used in week 3
_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

def detect_aruco_markers(marker_id =7, px=600,pad=60): # same as lab defaults
    tag = cv2.aruco.generateImageMarker(_dictionary, marker_id, px,) # marker pattern

    #white margin around the marker to make it easier to detect
    return cv2.copyMakeBorder(tag, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255), px, pad 

#save mujoco texture assets to a folder so that they can be used in the mujoco xml file
image, px, pad = detect_aruco_markers() # generate the marker immage

# set the output path for the marker image
output_path = Path(__file__).parent / "robotis_tb3" / "assets" / "aruco_marker_7.png" 

if not cv2.imwrite(str(output_path), image):  # save and check if the image was saved successfully
# report an error if the image could not be saved
    raise RuntimeError(f"Failed to save the aruco marker image at {output_path}") # failed attempt

print(f"aruco marker image saved at {output_path}") # successful attempt

    