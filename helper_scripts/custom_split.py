import os
import json
import argparse
from typing import List, Dict
from collections import defaultdict


def list_files_without_extension(directory_path, file_extension: str = ".nii.gz") -> None:
    files = os.listdir(directory_path)
    file_extension_length = len(file_extension)
    file_names_without_extension = [file[:-file_extension_length]
                                    for file in files if file.endswith(file_extension)]
    return file_names_without_extension


def create_folds(grouped_images: List[str], number_of_folds: int) -> List[Dict]:
    folds = [{"train": [], "val": []} for _ in range(number_of_folds)]
    validation = 0
    for key in grouped_images.keys():
        for index, fold in enumerate(folds):
            if index != validation:
                fold["train"].extend(grouped_images[key])

            else:
                fold["val"].extend(grouped_images[key])
        validation += 1
        validation %= number_of_folds
    return folds


def save_folds(splits: List[Dict], path_to_splits_final: str) -> None:
    with open(path_to_splits_final, "w") as json_file:
        json.dump(splits, json_file, indent=4)


def group_files(file_names: List[str]) -> Dict:
    """Groups files by the last two literals of the file name. Assuming three digit number at the end of the file name, e.g.:"_001".
    The first digit (0)01 is used to distinguish different images of the same object. Please adapt this function to your needs.


    Args:
        file_names (List[str]): _description_

    Returns:
        Dict: _description_
    """
    grouped_images = defaultdict(list)
    for file in file_names:
        sample = int(file[-2:])
        if sample not in grouped_images.keys():
            grouped_images[sample] = []
        grouped_images[int(file[-2:])].append(file)
    return grouped_images


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script with flags")
    parser.add_argument("--number_of_folds", type=int,
                        default=5, help="Number of Folds")
    parser.add_argument("--file_extension", type=str, default=".nii.gz",
                        help="File extension that should be removed")
    parser.add_argument("--path_to_raw_files", type=str,
                        help="Path where the files are listed")
    parser.add_argument("--path_to_splits_file", type=str, default="splits_final.json",
                        help="Path where the split_final.json should be stored")
    args = parser.parse_args()

    number_of_folds = args.number_of_folds
    path_to_raw_files = args.path_to_raw_files
    path_to_splits_final = args.path_to_splits_file
    file_extension = args.file_extension

    file_names = list_files_without_extension(
        path_to_raw_files, file_extension)
    grouped_images = group_files(file_names=file_names)
    folds = create_folds(grouped_images, number_of_folds)
    save_folds(folds, path_to_splits_final)
