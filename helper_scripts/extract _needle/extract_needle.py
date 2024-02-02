import nibabel as nib
import argparse
import os.path
# from typing import Path
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from skspatial.objects import Line, Points, Plane, Vector
from skspatial.plotting import plot_3d

def analyze_segmentation(analyze_error = False, ids = {}):
    segmentation, nifti_header = read_nifti()

    tool_center_point = get_needle_holder_center(segmentation, ids["needle_holder"])

    if analyze_error == True:
        angulation_error = error_angle_marker_artifact()
        position_error = error_angle_marker_artifact()



def read_nifti(file_path: str) ->np.ndarray:
    nifit_img = nib.load(file_path)
    nifti_data = nifit_img.get_fdata()
    nifti_header = nifit_img.header
    return nifti_data, nifti_header

def get_needle_holder_pose(image_segmented, needleholder_id):
    points = get_points(image_segmented, needleholder_id)
    centroid = points.centroid()
    return centroid

def get_needle_holder_orientation():

def get_points(image_segmented, class_id):
    indices = np.argwhere(image_segmented == class_id)
    points = Points(indices)
    return points

def get_orientation_from_markers(image_segmented, marker_id):
    points = get_points(image_segmented, marker_id)
    line_fit = Line.best_fit(points)
    orientation = line_fit.direction
    return orientation

def error_angle_marker_artifact(line_fit_artifact, line_fit_marker):
    artifact_direction_vector = line_fit_artifact.direction
    marker_direction_vector = line_fit_marker.direction
    cosine_similarity = artifact_direction_vector.cosine_similarity(marker_direction_vector)
    
## nnUnet_results
In this directory the trained model of nnUnet is stored, which is used to predict the segmentations.

## piezobot_tracking

- `piezobot_pose_correction.py`: Calculates the pose_error between the target pose of the robot and the from the segmentation of the predicted_pose (from which wesigned_angle = artifact_direction_vector.angle_signed_3d(marker_direction_vector)
    return cosine_similarity, signed_angle

def position_error(centroid_needleholder, center_needleholder):
    error_vector = Vector.from_points(centroid_needleholder, center_needleholder)
    return error_vector




def get_needle_path(image_segmented, needle_id):
    fig = plt.figure()
    print(image_segmented.shape)
    indices = np.argwhere(image_segmented == 1)
    ax = fig.add_subplot(111, projection='3d')
    points = Points(indices)
    line_fit = Line.best_fit(points)

    plot_3d(
        line_fit.plotter(t_1=-30, t_2=30, c='k'),
        points.plotter(c='b', depthshade=False),
    )

    orthotogonal_plane = Plane(points.centroid, line_fit.direction)

    # indices = np.argwhere(image_segmented ==2)
    # ax.scatter(indices[:,0], indices[:,1], indices[:,2],c="g")
    # indices = np.argwhere(image_segmented ==3)
    # ax.scatter(indices[:,0], indices[:,1], indices[:,2],c="b")
    # fig.show()
def get_needle_tip(points):
    needle_tip = 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument("--path_to_image", type=str, help="Absolute path to image")
    args = parser.parse_args()

    path_to_image = args.path_to_image

    nifti_data, nifti_header = read_nifti(path_to_image)

    get_needle_path(nifti_data,2)
    print(nifti_data.shape)
