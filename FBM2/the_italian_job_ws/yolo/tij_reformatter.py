#!/bin/python3

# The Italian Job,
# METRIC RAMI 2021, FBM2 competition
# Copyright 2021 Gerardo Puga

import json
import sys

IMAGE_WIDTH_PX = 448
IMAGE_HEIGHT_PX = 448


def load_json_file(filepath):
    with open(filepath) as file_desc:
        file_data = json.load(file_desc)
    return file_data


def reformat_competition_data(file_structured_data):
    output_data = []
    for item in file_structured_data:
        full_filename = item["filename"]
        objects_list = item["objects"]

        no_path_filename = full_filename.split(sep="/")[-1]

        for detection in objects_list:
            center_x = detection["relative_coordinates"]["center_x"]
            center_y = detection["relative_coordinates"]["center_y"]
            width = detection["relative_coordinates"]["width"]
            height = detection["relative_coordinates"]["height"]
            confidence = detection["confidence"]

            tlx = max(round((center_x - 0.5 * width) * IMAGE_WIDTH_PX), 0)
            tly = max(round((center_y - 0.5 * height) * IMAGE_HEIGHT_PX), 0)
            brx = min(round((center_x + 0.5 * width) *
                      IMAGE_WIDTH_PX), IMAGE_WIDTH_PX - 1)
            bry = min(round((center_y + 0.5 * height) *
                      IMAGE_HEIGHT_PX), IMAGE_HEIGHT_PX - 1)

            output_data.append((no_path_filename, tlx, tly, brx, bry))
    return output_data


def save_output_data(filepath, output_data):
    with open(filepath, "w") as file_desc:
        for line_data in output_data:
            line = " ".join([str(x) for x in line_data])
            file_desc.writelines(line + "\n")
            print("writing: " + line)


def main():
    try:
        if len(sys.argv) < 3:
            print("Insuficient arguments.")
            print(
                "Usage: {} <full_darknet_json_path> <full_result_file_path>", sys.argv[0])
            exit(1)

        full_darknet_json_path = sys.argv[1]
        full_result_file_path = sys.argv[2]

        input_data = load_json_file(full_darknet_json_path)
        output_data = reformat_competition_data(input_data)
        save_output_data(full_result_file_path, output_data)

    except Exception as e:
        print("Exception thrown!: " + e.with_traceback())


if __name__ == '__main__':
    main()
