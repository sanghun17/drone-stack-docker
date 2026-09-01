#!/usr/bin/env python3
"""Create a compact simulation replay bag with monotonic IMU timestamps.

The source bag is never modified. Only the raw topics consumed by the
FAST-LIVO simulation replay plus GT are copied. Exact duplicate or backward
IMU header stamps are dropped because the synchronized-initialization gate
correctly rejects a non-increasing input sequence.
"""

import argparse
import hashlib
from pathlib import Path

import rosbag


DEFAULT_TOPICS = (
    "/airsim_node/hmcl/imu/imu",
    "/camera/left/image_raw",
    "/voxel_grid/output",
    "/gt_odom",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def message_stamp(message, bag_time):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and stamp.to_nsec() > 0:
        return stamp.to_nsec()
    return bag_time.to_nsec()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--imu-topic", default="/airsim_node/hmcl/imu/imu",
    )
    parser.add_argument(
        "--topic", action="append", dest="topics",
        help="topic to retain; repeat to override the default topic set",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    source = args.input.resolve()
    output = args.output.resolve()
    if source == output:
        parser.error("input and output must be different files")
    if not source.is_file():
        parser.error(f"missing input bag: {source}")
    if output.exists() and not args.force:
        parser.error(f"refusing to overwrite {output}; pass --force")

    topics = tuple(args.topics or DEFAULT_TOPICS)
    if args.imu_topic not in topics:
        parser.error("the retained topics must include --imu-topic")

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".active")
    if temporary.exists():
        temporary.unlink()

    copied = {topic: 0 for topic in topics}
    duplicate_or_backward_imu = 0
    previous_imu_stamp = None
    try:
        with rosbag.Bag(str(source), "r") as reader, rosbag.Bag(
            str(temporary), "w", compression=rosbag.Compression.LZ4,
        ) as writer:
            for topic, message, bag_time in reader.read_messages(topics=topics):
                if topic == args.imu_topic:
                    stamp = message_stamp(message, bag_time)
                    if previous_imu_stamp is not None and stamp <= previous_imu_stamp:
                        duplicate_or_backward_imu += 1
                        continue
                    previous_imu_stamp = stamp
                writer.write(topic, message, bag_time)
                copied[topic] += 1
        temporary.replace(output)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise

    print(f"input={source}")
    print(f"output={output}")
    print(f"input_sha256={sha256(source)}")
    print(f"duplicate_or_backward_imu_dropped={duplicate_or_backward_imu}")
    for topic in topics:
        print(f"copied[{topic}]={copied[topic]}")


if __name__ == "__main__":
    main()
