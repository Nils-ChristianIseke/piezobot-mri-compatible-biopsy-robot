import nibabel as nib
import argparse
import os.path
import numpy as np
from skspatial.objects import Line, Points, Vector
from skspatial.plotting import plot_3d
from scipy.spatial.distance import pdist
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.ndimage import center_of_mass
import pandas as pd
from nibabel.affines import apply_affine
from scipy.spatial.transform import Rotation
from scipy.ndimage import label


def analyze_segmentation(path_to_image_dir) -> {}:
    poses = {
        "image": [],
        "position_needle": [],
        "orientation_needle": [],
        "position_needle_holder": [],
        "orientation_needle_holder": [],
        "error_position_needleholder_needle": [],
        "error_orientation_needleholder_needle_angular": [],
        "frame_orientation": [],
        "error_needle_frame": [],
        "error_needle_holder_frame": [],
    }

    ids_image = {"needle_holder": 5, "needle": 1}

    for image in os.listdir(path_to_image_dir):
        path_to_image = os.path.join(path_to_image_dir, image)
        nifti_image_segmentation = nib.load(path_to_image)
        object_poses_image = get_object_pose(
            nifti_image_segmentation,
            ids_image,
            image,
            poses,
            needle_holder_complete=False,
        )

    dataframe = pd.DataFrame(poses)
    dataframe.set_index("image")
    dataframe[["image", "error_needle_frame", "error_needle_holder_frame"]].to_csv(
        "test.csv"
    )
    # ax = dataframe.boxplot("error_orientation_needleholder_needle_angular")
    # fig = ax.get_figure()
    # fig.savefig("boxplot.png")

    return poses


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


def merge_clusters(nifi, ids):
    segmentaion = nifi.get_fdata().copy()
    segmentaion[segmentaion == ids["needle"]] = ids["needle_holder"]
    segmentaion[segmentaion != ids["needle_holder"]] = 0
    labeled_array, num_features = label(segmentaion)
    sizes = np.bincount(labeled_array.ravel())[1:]
    largest_component = np.argmax(sizes) + 1
    segmentaion[labeled_array != largest_component] = nifi.get_fdata()[
        labeled_array != largest_component
    ]
    nib.save(nib.Nifti1Image(segmentaion, nifi.affine), "test.nii.gz")
    return segmentaion


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


def get_frame_orientation_from_image_name(image_name):
    splitted_image_name = image_name.split("_")
    frame_roll = -int(splitted_image_name[1])
    frame_pitch = int(splitted_image_name[0])
    unit_z = Vector([0, 0, 1])
    rotation = Rotation.from_euler("YXZ", [frame_roll, frame_pitch, 0], degrees=True)
    frame_orientation = rotation.apply(unit_z)
    return frame_orientation


def get_object_pose(
    nifti_image, label_ids, image_name, object_poses, needle_holder_complete=False
):
    if needle_holder_complete == True:
        segmentation = merge_clusters(nifti_image, label_ids)
    else:
        segmentation = nifti_image.get_fdata()
    object_pose = {"needle_holder": (), "needle": ()}
    for id in label_ids:
        segmentation_voxel_coordinates = get_points(segmentation, label_ids[id])
        if len(segmentation_voxel_coordinates) == 0:
            return

        segmentation_ras_plus_coordinates = apply_affine(
            nifti_image.affine, segmentation_voxel_coordinates
        )

        points = Points(segmentation_ras_plus_coordinates)
        centroid = points.centroid()
        print(id, points.size)
        best_fit_line = Line.best_fit(points)
        orientation = best_fit_line.vector.norm()
        object_pose[id] = (centroid, orientation)

    frame_orientation = Vector(get_frame_orientation_from_image_name(image_name))
    object_poses["frame_orientation"].append(frame_orientation)
    object_poses["image"].append(image_name)
    object_poses["position_needle"].append(object_pose["needle"][0])
    object_poses["orientation_needle"].append(object_pose["needle"][1])
    object_poses["position_needle_holder"].append(object_pose["needle_holder"][0])
    object_poses["orientation_needle_holder"].append(object_pose["needle_holder"][1])

    (
        postion_error_nh_needle,
        angulation_error_nh_needle,
        cosine_similarity_nh_needle,
    ) = get_pose_error(object_pose["needle_holder"], object_pose["needle"])

    angulation_error_needle, cosine_similarity_needle = angular_deviation(
        object_poses["frame_orientation"][-1], object_poses["orientation_needle"][-1]
    )
    if angulation_error_needle < 0:
        angulation_error_needle = min(
            angulation_error_needle, 180 + angulation_error_needle
        )
    else:
        angulation_error_needle = min(
            angulation_error_needle, 180 - angulation_error_needle
        )
    (
        angulation_error_needle_holder,
        cosine_similarity_needle_holder,
    ) = angular_deviation(
        object_poses["frame_orientation"][-1],
        object_poses["orientation_needle_holder"][-1],
    )
    if angulation_error_needle_holder < 0:
        angulation_error_needle_holder = min(
            angulation_error_needle_holder, 180 + angulation_error_needle_holder
        )
    else:
        angulation_error_needle_holder = min(
            angulation_error_needle_holder, 180 - angulation_error_needle_holder
        )
    object_poses["error_needle_holder_frame"].append(angulation_error_needle_holder)
    object_poses["error_needle_frame"].append(angulation_error_needle)
    object_poses["error_position_needleholder_needle"].append(postion_error_nh_needle)
    object_poses["error_orientation_needleholder_needle_angular"].append(
        angulation_error_nh_needle
    )
    print(image_name)
    print("Error Needle_Frame", object_poses["error_needle_frame"][-1])
    print("Error Needleholder_Frame", object_poses["error_needle_holder_frame"][-1])

    return object_poses


def get_points(image_segmented: np.ndarray, class_id: int) -> Points:
    indices = np.argwhere(image_segmented == class_id)
    return indices


def angular_deviation(vector_1, vector_2):
    cosine_similarity = vector_1.cosine_similarity(vector_2)
    error_angular_degree = np.degrees(
        vector_1.angle_signed_3d(vector_2, vector_1.cross(vector_2))
    )
    return error_angular_degree, cosine_similarity


def position_deviation(centroid_needleholder, center_needleholder):
    error_vector = Vector.from_points(centroid_needleholder, center_needleholder)
    return error_vector


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument(
        "--path_to_image_dir",
        type=str,
        help="Path to directory containing images to analyze",
    )

    args = parser.parse_args()
    path_to_image_dir = args.path_to_image_dir

    analyze_segmentation(path_to_image_dir)
