#!/usr/bin/env python3
"""Generate DICT_4X4_50 direction markers 1–8 as PNGs with white margins.

Usage: python scripts/generate_aruco_markers.py --output build/aruco-markers
Print without stretching; the configured physical size is the black square's
outer edge, excluding the white margin.
"""

import argparse
import logging
from pathlib import Path

_LOGGER = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/aruco-markers"))
    parser.add_argument("--size", type=int, default=600, help="black square width in pixels")
    parser.add_argument("--margin", type=int, default=100, help="white margin in pixels per side")
    args = parser.parse_args()
    if args.size < 6 or args.margin < 1:
        parser.error("size must be at least 6 and margin must be positive")
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        import cv2
        if not hasattr(cv2, "aruco"):
            raise ImportError("OpenCV build has no aruco module")
    except ImportError as exc:
        _LOGGER.error("Install opencv-contrib-python-headless: %s", exc)
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    args.output.mkdir(parents=True, exist_ok=True)
    for marker_id in range(1, 9):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, args.size)
        image = cv2.copyMakeBorder(
            marker, args.margin, args.margin, args.margin, args.margin,
            cv2.BORDER_CONSTANT, value=255,
        )
        path = args.output / f"aruco_DICT_4X4_50_{marker_id}.png"
        if not cv2.imwrite(str(path), image):
            raise OSError(f"Could not write {path}")
        _LOGGER.info("Saved %s", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
