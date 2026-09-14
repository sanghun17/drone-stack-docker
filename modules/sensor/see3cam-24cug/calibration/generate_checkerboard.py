#!/usr/bin/env python3
"""Generate an exact-scale A4 checkerboard PDF for ROS camera_calibration."""

import argparse
from pathlib import Path


POINTS_PER_MM = 72.0 / 25.4


def pdf_document(width_points, height_points, content):
    objects = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.6f %.6f] "
            "/Resources << >> /Contents 4 0 R >>"
            % (width_points, height_points)
        ).encode("ascii"),
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"endstream",
    ]
    output = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = [0]
    for index, obj in enumerate(objects, start=1):
        offsets.append(len(output))
        output.extend(("%d 0 obj\n" % index).encode("ascii"))
        output.extend(obj)
        output.extend(b"\nendobj\n")
    xref = len(output)
    output.extend(("xref\n0 %d\n" % (len(objects) + 1)).encode("ascii"))
    output.extend(b"0000000000 65535 f \n")
    for offset in offsets[1:]:
        output.extend(("%010d 00000 n \n" % offset).encode("ascii"))
    output.extend(
        (
            "trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
            % (len(objects) + 1, xref)
        ).encode("ascii")
    )
    return bytes(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inner-corners", default="8x6")
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        inner_x, inner_y = [int(value) for value in args.inner_corners.lower().split("x")]
    except (TypeError, ValueError):
        parser.error("--inner-corners must look like 8x6")
    if inner_x < 2 or inner_y < 2 or args.square_mm <= 0.0:
        parser.error("corner counts and square size must be positive")

    columns, rows = inner_x + 1, inner_y + 1
    page_width_mm, page_height_mm = 297.0, 210.0  # ISO A4 landscape
    board_width_mm = columns * args.square_mm
    board_height_mm = rows * args.square_mm
    if board_width_mm > page_width_mm or board_height_mm > page_height_mm:
        parser.error("checkerboard does not fit on A4 landscape")
    offset_x_mm = (page_width_mm - board_width_mm) / 2.0
    offset_y_mm = (page_height_mm - board_height_mm) / 2.0

    operations = ["q", "0 0 0 rg"]
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2:
                continue
            x = (offset_x_mm + column * args.square_mm) * POINTS_PER_MM
            # PDF y runs upward; the parity is immaterial, but this keeps the
            # board geometrically centered without a raster conversion.
            y = (offset_y_mm + (rows - 1 - row) * args.square_mm) * POINTS_PER_MM
            side = args.square_mm * POINTS_PER_MM
            operations.append("%.8f %.8f %.8f %.8f re f" % (x, y, side, side))
    operations.append("Q")
    content = ("\n".join(operations) + "\n").encode("ascii")
    destination = Path(args.output).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(
        pdf_document(page_width_mm * POINTS_PER_MM, page_height_mm * POINTS_PER_MM, content)
    )
    print(
        "%s: A4 landscape, %dx%d inner corners, %.3f mm squares, %.1fx%.1f mm board"
        % (
            destination,
            inner_x,
            inner_y,
            args.square_mm,
            board_width_mm,
            board_height_mm,
        )
    )


if __name__ == "__main__":
    main()

