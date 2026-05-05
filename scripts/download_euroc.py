#!/usr/bin/env python3
"""
Download EuRoC MH_01_easy dataset via kagglehub.

Usage:
    python scripts/download_euroc.py

Requires:
    pip install kagglehub

Kaggle API credentials may be required for first run:
    https://github.com/Kaggle/kaggle-api#api-credentials
"""

import sys


def main() -> int:
    try:
        import kagglehub
    except ImportError:
        print(
            "kagglehub not found. Install with: pip install kagglehub",
            file=sys.stderr,
        )
        return 1

    print("Downloading EuRoC MH_01_easy dataset (chunai/euroc-mh-01-easy-ros-bag-dataset)...")
    path = kagglehub.dataset_download("chunai/euroc-mh-01-easy-ros-bag-dataset")
    bag_path = path + "/MH_01_easy.bag"
    print(f"Path to dataset files: {path}")
    print()
    print("This is a ROS1 bag. Convert to ROS2 format first:")
    print("  pip install rosbags")
    print(f"  rosbags-convert --src {bag_path} --dst ./data/MH_01_easy_ros2")
    print()
    print("Then play the converted bag:")
    print("  ros2 bag play ./data/MH_01_easy_ros2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
