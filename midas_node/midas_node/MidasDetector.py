import torch
import numpy as np
import time
from enum import Enum
import cv2

# Available Models
class ModelType(Enum):
    DPT_LARGE = "DPT_Large" # Slow but powerful
    DPT_HYBRID = "DPT_Hybrid" # Balance between speed and power
    MIDAS_SMALL = "MiDaS_small" # Fast but not powerful

class MidasDetector:
    # Constructor
    def __init__(self, model_type=ModelType.MIDAS_SMALL): # Use small model for faster speed
        print("Loading MiDaS model...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.hub.load("isl-org/MiDaS", model_type.value).to(self.device).eval()

        # Transform Input Image to Fit Model
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = (
            midas_transforms.dpt_transform
            if model_type in [ModelType.DPT_LARGE, ModelType.DPT_HYBRID]
            else midas_transforms.small_transform
        )

    # Predict DepthMap for Each Frame
    def predict_depth(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = self.transform(img).to(self.device) # Transform Image and Send Tensor to Device

        with torch.no_grad(): # Non-Training Mode -> Don't Save Gradients
            prediction = self.model(input_tensor)
            prediction = torch.nn.functional.interpolate( # DepthMap Has to Fit Original Frame
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False
            ).squeeze()

        depth_raw = prediction.cpu().numpy() # Real Depth
        depth_norm = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)

        # Detect Danger in Rectangle
        h, w = depth_raw.shape
        x, y, roi_w, roi_h = w//2 - w//8, h//2 - h//8, w//4, h//4
        roi_depth = depth_raw[y:y+roi_h, x:x+roi_w]
        threshold = 0.8 * depth_raw.max()
        danger = np.any(roi_depth > threshold)

        return depth_colored, depth_raw , danger
