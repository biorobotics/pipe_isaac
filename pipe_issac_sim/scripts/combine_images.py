#!/usr/bin/env python3
import os
import math
import cv2
import numpy as np
import argparse

OUTPUT_FILE = "../../combined_grid.png"
COLS = 8
ROWS = 20
CELL_WIDTH = 200
CELL_HEIGHT = 200


def get_image_files(folder_path):
    supported_formats = ".png"  # only allow pngs for now
    image_files = []

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(supported_formats):
            image_files.append(os.path.join(folder_path, filename))

    return sorted(image_files)


def calculate_grid_size(num_images, cols=None, rows=None):
    if cols and rows:
        return cols, rows
    elif cols:
        rows = math.ceil(num_images / cols)
        return cols, rows
    elif rows:
        cols = math.ceil(num_images / rows)
        return cols, rows
    else:
        cols = math.ceil(math.sqrt(num_images))
        rows = math.ceil(num_images / cols)
        return cols, rows


def combine_images(
    directory_path,
    output_path=OUTPUT_FILE,
    cols=None,
    rows=None,
    cell_width=300,
    cell_height=300,
):
    image_files = get_image_files(directory_path)

    if not image_files:
        print(f"No image files found in {directory_path}")
        return

    print(f"Found {len(image_files)} images (should be 160)")

    cols, rows = calculate_grid_size(len(image_files), cols, rows)
    print(f"Combining images: {cols} columns x {rows} rows")

    combined_width = cols * cell_width
    combined_height = rows * cell_height

    combined = np.full(
        (combined_height, combined_width, 3), (255, 255, 255), dtype=np.uint8
    )

    for i, image_path in enumerate(image_files):
        try:
            img = cv2.imread(image_path)

            if img is None:
                print(f"Can't read image: {image_path}")
                continue

            img_height, img_width = img.shape[:2]

            col = i % cols
            row = i // cols

            x_offset = col * cell_width + (cell_width - img_width) // 2
            y_offset = row * cell_height + (cell_height - img_height) // 2

            x_end = min(x_offset + img_width, combined_width)
            y_end = min(y_offset + img_height, combined_height)

            if x_offset < 0:
                x_offset = 0
            if y_offset < 0:
                y_offset = 0

            img_x_end = x_end - x_offset
            img_y_end = y_end - y_offset

            if x_offset < combined_width and y_offset < combined_height:
                combined[y_offset:y_end, x_offset:x_end] = img[:img_y_end, :img_x_end]

            print(
                f"processed {i + 1}/{len(image_files)}: {os.path.basename(image_path)}"
            )

        except Exception as e:
            print(f"ERROR while processing {image_path}: {e}")
            continue

    cv2.imwrite(output_path, combined)
    print(f"Combined image saved as: {output_path}")
    print(f"Final dim: {combined_width} x {combined_height}")


def main():
    parser = argparse.ArgumentParser(description="Image combiner")
    parser.add_argument("-d", "--directory", help="Source image directory")

    args = parser.parse_args()

    if not os.path.isdir(args.directory):
        print(f"ERROR: can't find directory {args.directory}")
        return

    combine_images(
        directory_path=args.directory,
        output_path=OUTPUT_FILE,
        cols=COLS,
        rows=ROWS,
        cell_width=CELL_WIDTH,
        cell_height=CELL_HEIGHT,
    )


if __name__ == "__main__":
    main()
