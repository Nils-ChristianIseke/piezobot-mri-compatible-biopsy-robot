import nibabel as nib
import argparse
import os.path
from typing import List, Dict, Tuple
import numpy as np
import matplotlib.pyplot as plt
from skspatial.objects import Line, Points, Point, Plane, Vector
from skspatial.plotting import plot_3d
from scipy.spatial.distance import pdist, squareform
from scipy.cluster.hierarchy import linkage, fcluster
import pandas as pd


def analyze_segmentation(path_to_source_directory) -> {}:
    path_to_image_dir = os.path.join(path_to_source_directory, "image")
    path_to_validation_dir = os.path.join(path_to_source_directory, "validation")

    poses = {
        "image": [],
        "position_needle": [],
        "orientation_needle": [],
        "position_needle_holder": [],
        "orientation_needle_holder": [],
        "position_marker": [],
        "orientation_marker": [],
        "error_position_needleholder_marker": [],
        "error_orientation_needleholder_marker_angular": [],
        "cosine_similarity_needleholder_marker": [],
        "error_position_needleholder_needle": [],
        "error_orientation_needleholder_needle_angular": [],
        "cosine_similarity_needleholder_needle": [],
    }
    ids_image = {"needle_holder": 5, "needle": 1}
    ids_marker = {"markers": 1}

    for image in os.listdir(path_to_image_dir):
        path_to_image = os.path.join(path_to_image_dir, image)
        path_to_validation = os.path.join(
            path_to_validation_dir, image[:-7] + "_validation.nii.gz"
        )

        nifti_image_segmentation = read_nifti_file(path_to_image)
        nifti_image_validation = read_nifti_file(path_to_validation)

        object_poses_image = get_object_pose(nifti_image_segmentation, ids_image, 0)
        object_poses_marker = get_object_pose(nifti_image_validation, ids_marker, 0)
        (
            position_error_nh_marker,
            angulation_error_nh_marker,
            cosine_similarity_nh_marker,
        ) = get_pose_error(
            object_poses_image["needle_holder"], object_poses_marker["markers"]
        )
        (
            postion_error_nh_needle,
            angulation_error_nh_needle,
            cosine_similarity_nh_needle,
        ) = get_pose_error(
            object_poses_image["needle_holder"], object_poses_image["needle"]
        )
        poses["image"].append(image)
        poses["position_needle"].append(object_poses_image["needle"][0])
        poses["orientation_needle"].append(object_poses_image["needle"][1])
        poses["position_needle_holder"].append(object_poses_image["needle_holder"][0])
        poses["orientation_needle_holder"].append(
            object_poses_image["needle_holder"][1]
        )
        poses["position_marker"].append(object_poses_marker["markers"][0])
        poses["orientation_marker"].append(object_poses_marker["markers"][1])
        poses["error_position_needleholder_marker"].append(position_error_nh_marker)
        poses["error_orientation_needleholder_marker_angular"].append(
            angulation_error_nh_marker
        )
        poses["cosine_similarity_needleholder_marker"].append(
            cosine_similarity_nh_marker
        )
        poses["error_position_needleholder_needle"].append(postion_error_nh_needle)
        poses["error_orientation_needleholder_needle_angular"].append(
            angulation_error_nh_needle
        )
        poses["cosine_similarity_needleholder_needle"].append(
            cosine_similarity_nh_needle
        )

    dataframe = pd.DataFrame(poses)
    dataframe.boxplot("error_orientation_needleholder_needle_angular")
    print(dataframe["cosine_similarity_needleholder_needle"].mean())
    # else:
    # warnings.warn("Validation file was not found")
    return poses


def get_needle_size(segmentation, id):
    len(segmentation)


def cluster_indices_and_keep_biggest_cluster(segmentaion, id):
    """Postprocesses the segmentaion of the validation images. This is necessary because validation images contain two markers, which are typically segmented as class
    NeedleHolder by the trained Unet (because it has not be trained to segment markers). But the markers should not be used when calculating the orientation of the segmented needle_holder.
    Hence, they are excluded by this function.

    Args:
        segmentaion (_type_): Segmentation, which was output by the nnUnet

    Returns:
        _type_: _description_
    """
    indices = np.argwhere(segmentaion == id)

    points = Points(indices)
    plot_3d(
        points.plotter(c="b", depthshade=False),
    )
    distance_matrix = pdist(indices)
    linkage_matrix = linkage(distance_matrix, method="single")
    cluster_ids = fcluster(linkage_matrix, t=0.5)
    biggest_cluster_id = np.unique(cluster_ids, return_counts=True)[0][
        np.unique(cluster_ids, return_counts=True)[1].argmax()
    ]
    indices = indices[cluster_ids == biggest_cluster_id]
    points = Points(indices)
    plot_3d(
        points.plotter(c="b", depthshade=False),
    )
    return indices


def read_nifti_file(file_path: str) -> np.ndarray:
    """Reads the nifti file.

    Args:
        file_path (str): _description_. Defaults to "".
        path_to_marker_label (str, optional): _description_. Defaults to "".

    Returns:
        SpatialImage: _description_
    """
    nifti_image = nib.load(file_path)
    nifti_data = nifti_image.get_fdata()
    nifti_header = nifti_image.header
    return nifti_image


def get_pose_error(needle_holder_pose, expected_needle_holder_pose):
    needle_holder_position, needle_holder_orientation = needle_holder_pose
    (
        excpected_needle_holder_position,
        expected_needle_holder_orientation,
    ) = expected_needle_holder_pose

    position_error = position_deviation(
        needle_holder_position, excpected_needle_holder_position
    )
    angulation_error, cosine_similarity = angular_deviation(
        needle_holder_orientation, expected_needle_holder_orientation
    )

    return position_error, angulation_error, cosine_similarity


def get_object_pose(
    nifti_image,
    ids: Dict[str, int],
    offset: float = 0,
    union_needle_holder_needle_artifact: bool = False,
) -> Dict[str, Tuple[Point, Vector]]:
    object_poses = {}
    nifti_data = nifti_image.get_fdata()

    for class_id in ids:
        if class_id == "needle_holder":
            segmentation_voxel_coordinates = cluster_indices_and_keep_biggest_cluster(
                nifti_data, ids[class_id]
            )
        else:
            segmentation_voxel_coordinates = get_points(
                nifti_image.get_fdata(), ids[class_id]
            )

        if len(segmentation_voxel_coordinates) == 0:
            continue

        segmentation_voxel_coordinates_homogeneous = np.column_stack(
            (
                segmentation_voxel_coordinates,
                np.ones(segmentation_voxel_coordinates.shape[0]),
            )
        )

        # Apply the dot product to all points at once
        segmentation_ras_plus_coordinates = np.dot(
            segmentation_voxel_coordinates_homogeneous, nifti_image.affine.T
        )[:, :3]
        points = Points(segmentation_ras_plus_coordinates)

        centroid: Point = points.centroid()
        best_fit_line: Line = Line.best_fit(points)

        centroid = best_fit_line.to_point(offset)
        centroid_homogeneous = np.append(centroid, 1)
        centroid_image_coordinates = Point(
            np.dot(nifti_image.affine, centroid_homogeneous.T)[:3]
        )

        orientation = best_fit_line.vector
        orientation_homogeneous = np.append(orientation, 1)
        orientation_image_coordinates = Vector(
            np.dot(nifti_image.affine, orientation_homogeneous)[:3]
        )

        object_poses[class_id] = (
            centroid_image_coordinates,
            orientation_image_coordinates,
        )

    return object_poses


def get_points(image_segmented: np.ndarray, class_id: int) -> Points:
    indices = np.argwhere(image_segmented == class_id)
    points = Points(indices)
    return indices


def angular_deviation(artifact_direction_vector, marker_direction_vector):
    artifact_direction_vector = artifact_direction_vector
    marker_direction_vector = marker_direction_vector
    cosine_similarity = artifact_direction_vector.cosine_similarity(
        marker_direction_vector
    )
    error_angular_rad = artifact_direction_vector.angle_signed_3d(
        marker_direction_vector,
        direction_positive=artifact_direction_vector.cross(marker_direction_vector),
    )
    return error_angular_rad, cosine_similarity


def position_deviation(centroid_needleholder, center_needleholder):
    error_vector = Vector.from_points(centroid_needleholder, center_needleholder)
    return error_vector


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument(
        "--path_to_images",
        type=str,
        help="Path to directory containing images to analyze",
    )

    args = parser.parse_args()
    path_to_images = args.path_to_images

    analyze_segmentation(path_to_images)
