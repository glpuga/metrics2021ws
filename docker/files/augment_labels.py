#!/bin/python3

import random
import sys

#
# This script augments data in some of the images where the label has a large x/y or y/x ratio, 
# creating labels for segments within the original label aligned with the long side. This assumes
# the label will be aligned with the crack, and that we are then creating labels for parts of it
#


RATIO_THRESHOLD = 5
AUGMENTATION_FACTOR = 3


def load_labels_file(filepath):
    with open(filepath) as file_desc:
        file_data = file_desc.readlines()
    return file_data


def split_and_convert(data):
    split_strings = [x.split(" ") for x in data]
    fp_data = [
        [int(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4])] for x in split_strings
    ]
    return fp_data


def augment_data(data):
    def swap(x, y, w, h): return (y, x, h, w)

    augmented_data = []

    for cl, xc, yc, w, h in data:
        sample_augmentation = []
        org_sample = (cl, xc, yc, w, h)

        x1 = xc - w/2
        y1 = yc - h/2

        ratio = w / h

        rotated = False
        if ratio < 1/RATIO_THRESHOLD:
            ratio = 1 / ratio
            rotated = True
            x1, y1, w, h = swap(x1, y1, w, h)

        # if the ratio is not adequate, ignore the image
        if ratio < RATIO_THRESHOLD:
            print("Not augmenting : ", org_sample)
            continue

        for n in range(AUGMENTATION_FACTOR):
            w_factor = random.random() * 0.5 + 0.49
            o_factor = random.random() * 0.5 + 0.49

            adj_x1 = x1 + w * (1 - w_factor) * o_factor
            adj_y1 = y1
            adj_w = w * w_factor
            adj_h = h

            if rotated:
                adj_x1, adj_y1, adj_w, adj_h = swap(
                    adj_x1, adj_y1, adj_w, adj_h)

            sample_augmentation.append(
                [cl, adj_x1 + adj_w / 2, adj_y1 + adj_h / 2, adj_w, adj_h])

        print("Augmenting : ", org_sample)
        for new_entry in sample_augmentation:
            print("           : ", new_entry)
        augmented_data.append(org_sample)
        augmented_data.extend(sample_augmentation)
    return augmented_data


def overwrite_labels_file(filepath, output_data):
    with open(filepath, "w") as file_desc:
        print("Writing to " + filepath)
        for line_data in output_data:
            line = " ".join([str(x) for x in line_data])
            file_desc.writelines(line + "\n")
            print("writing: " + line)


def main():
    try:
        if len(sys.argv) < 2:
            print("Insuficient arguments.")
            print(
                "Usage: {} <full_result_file_path>", sys.argv[0])
            exit(1)
        filepath = sys.argv[1]
        input_data = load_labels_file(filepath)
        labels = split_and_convert(input_data)
        augmented_labels = augment_data(labels)
        overwrite_labels_file(filepath, augmented_labels)

    except Exception as e:
        print("Exception thrown!: " + e.with_traceback())


if __name__ == '__main__':
    main()
