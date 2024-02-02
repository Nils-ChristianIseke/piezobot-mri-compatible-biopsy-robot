import nibabel as nib
import argparse
import os.path
import numpy as np
from skspatial.objects import Line, Points, Vector, Point
from skspatial.plotting import plot_3d
from scipy.spatial.distance import pdist
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.ndimage import center_of_mass
import pandas as pd
from nibabel.affines import apply_affine
from scipy.spatial.transform import Rotation
from scipy.ndimage import label
from copy import deepcopy


def extract_poses(path_to_image, needle_holder_complete=True) -> {}:
    poses = {}
    nifti_image = nib.load(path_to_image)
    needle_id = 1
    needle_holder_id = 5
    if needle_holder_complete == True:
        segmentation = merge_clusters(nifti_image, needle_id, needle_holder_id)
    else:
        segmentation = nifti_image.get_fdata()

    for id in [needle_id, needle_holder_id]:
        segmentation_voxel_coordinates = np.argwhere(segmentation == id)

        if len(segmentation_voxel_coordinates) == 0:
            continue

        segmentation_mri = apply_affine(
            nifti_image.affine, segmentation_voxel_coordinates
        )
        points_mri = Points(segmentation_mri)
        centroid_ras_plus = points_mri.centroid()
        orientation_vector_ras_plus = Line.best_fit(points_mri).vector
        poses[id] = {
            "centroid": centroid_ras_plus,
            "orientation_vector": orientation_vector_ras_plus,
        }
    return poses


def merge_clusters(nifi, needle_id, needle_holder_id):
    segmentaion = nifi.get_fdata().copy()
    segmentaion[segmentaion == needle_id] = needle_holder_id
    segmentaion[segmentaion != needle_holder_id] = 0
    labeled_array, num_features = label(segmentaion)
    sizes = np.bincount(labeled_array.ravel())[1:]
    largest_component = np.argmax(sizes) + 1
    segmentaion[labeled_array != largest_component] = nifi.get_fdata()[
        labeled_array != largest_component
    ]
    return segmentaion


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument(
        "--path_to_image",
        type=str,
        help="Path to directory containing images to analyze",
    )

    args = parser.parse_args()
    path_to_image_dir = args.path_to_image

    extract_poses(path_to_image_dir)
