#!/usr/bin/env python3
"""Offline import/model-inference check. Does not create ROS nodes or UDP sockets.

Only run with the team's trusted checkpoints: PyTorch .pt loading uses pickle.
Success checks execution compatibility, NOT sensor accuracy or permission to drive.
"""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/opt/AutoVehicle/morai_ws"))
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    args = parser.parse_args()

    import cv2
    import numpy as np
    import scipy
    import torch
    import torchvision
    import ultralytics
    from ultralytics import YOLO
    from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
    from morai_perception_msgs.msg import StopLineDetection
    from common.msg import ObjectInfoArray

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; check image flavor, driver and --gpus")
    print("VERSIONS", sys.version, cv2.__version__, np.__version__, scipy.__version__,
          torch.__version__, torchvision.__version__, ultralytics.__version__)
    print("MESSAGES", *(cls.__name__ for cls in
          (CtrlCmd, EgoVehicleStatus, GPSMessage, StopLineDetection, ObjectInfoArray)))
    camera = args.workspace / "src/detection/camera_perception"
    checkpoints = [camera / "models/yolov8n.pt", camera / "models/best0902.pt",
                   camera / "lane/lane_seg_best.pt"]
    for checkpoint in checkpoints:
        if not checkpoint.is_file() or checkpoint.stat().st_size < 1000000:
            raise RuntimeError("Missing or placeholder checkpoint: " + str(checkpoint))
    for checkpoint in checkpoints[:2]:
        model = YOLO(str(checkpoint))
        result = model.predict(np.zeros((416, 416, 3), dtype=np.uint8),
                               imgsz=416, device=args.device, verbose=False, save=False)
        assert len(result) == 1
        print("YOLO_OK", checkpoint.name, model.names)
        del model
    sys.path.insert(0, str(camera / "lane"))
    from seg_model import LaneSegNet
    checkpoint = torch.load(str(checkpoints[2]), map_location="cpu", weights_only=False)
    model = LaneSegNet(checkpoint["args"].get("backbone", "resnet34"), pretrained=False)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.to(args.device).eval()
    with torch.inference_mode():
        result = model(torch.zeros(1, 3, 256, 256, device=args.device))
    print("LANE_OK", tuple(result.shape))
    print("OFFLINE_SMOKE_PASS: not a driving/calibration approval")


if __name__ == "__main__":
    main()
