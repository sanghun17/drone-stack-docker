#!/usr/bin/env python3
"""Create a compact labelled preview from generated square pad textures."""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("images", nargs="+")
    args = parser.parse_args()

    tile_side = 520
    title_height = 62
    gap = 24
    columns = 3
    rows = (len(args.images) + columns - 1) // columns
    canvas_width = columns * tile_side + (columns + 1) * gap
    canvas_height = rows * (tile_side + title_height) + (rows + 1) * gap
    canvas = np.full((canvas_height, canvas_width, 3), 245, dtype=np.uint8)

    for index, image_path in enumerate(args.images):
        source = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if source is None:
            raise RuntimeError("failed to read " + image_path)
        tile = cv2.resize(source, (tile_side, tile_side), interpolation=cv2.INTER_AREA)
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        row, column = divmod(index, columns)
        x = gap + column * (tile_side + gap)
        y = gap + row * (tile_side + title_height + gap)
        canvas[y + title_height:y + title_height + tile_side, x:x + tile_side] = tile
        label = "Pad %d  (L = 0.70 m)" % (index + 1)
        cv2.putText(
            canvas, label, (x + 8, y + 40), cv2.FONT_HERSHEY_SIMPLEX,
            0.86, (25, 25, 25), 2, cv2.LINE_AA,
        )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas, [cv2.IMWRITE_PNG_COMPRESSION, 9]):
        raise RuntimeError("failed to write " + str(output))
    print(output)


if __name__ == "__main__":
    main()
