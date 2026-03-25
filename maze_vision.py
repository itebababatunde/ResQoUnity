"""
maze_vision.py — Vision pipeline for aerial-assisted maze navigation.

Subscribes to the drone's downward camera feed and converts it to a binary
occupancy grid that the A* planner can consume.

Camera parameters (from ros2_bridge.py):
    focal_length = 24.0 mm  (Isaac Sim units)
    horizontal_aperture = 20.955 mm
    resolution = 640 x 480

At altitude 20.0m:
    FOV_h ≈ 47.2°, FOV_v ≈ 36.2°
    Visible footprint ≈ 17.5m × 13.1m  (fully contains 12m × 12m maze with CELL_SIZE=2.0m)

Image processing pipeline:
    RGB -> grayscale -> adaptive threshold -> morphological open/close -> resize to 13×13

Coordinate frame:
    - Image (0,0) = top-left = world (-footprint_x/2, +footprint_y/2)  (North-West)
    - Image origin corresponds to drone position projected to ground plane

Usage:
    node = MazeVisionNode(drone_altitude=20.0, grid_shape=(13, 13))
    # Called from ROS2 camera callback:
    grid = node.process_image(rgb_array)
    # Coordinate helpers:
    wx, wy = node.pixel_to_world(px, py)
    px, py = node.world_to_pixel(wx, wy)
"""

import math
import numpy as np

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False
    print("[MazeVision] WARNING: OpenCV not available. process_image() will use fallback mode.")


# ---------------------------------------------------------------------------
# Camera parameters (must match ros2_bridge.py)
# ---------------------------------------------------------------------------
FOCAL_LENGTH_MM      = 24.0   # mm (Isaac Sim PinholeCameraCfg)
HORIZONTAL_APERTURE  = 20.955 # mm
IMG_WIDTH            = 640
IMG_HEIGHT           = 480

# Maze occupancy grid size
GRID_ROWS = 13
GRID_COLS = 13


class MazeVisionNode:
    """
    Converts drone camera RGB frames to a binary 13×13 occupancy grid.

    Designed to be instantiated once and reused for multiple frames.
    Call process_image() for each new frame; accumulate results externally.
    """

    def __init__(self, drone_altitude=10.5, grid_shape=(GRID_ROWS, GRID_COLS),
                 img_width=IMG_WIDTH, img_height=IMG_HEIGHT):
        self.altitude = drone_altitude
        self.grid_h, self.grid_w = grid_shape
        self.img_w = img_width
        self.img_h = img_height

        # Compute FOV and visible footprint
        fov_h = 2.0 * math.atan(HORIZONTAL_APERTURE / (2.0 * FOCAL_LENGTH_MM))
        aspect = img_height / img_width
        fov_v = 2.0 * math.atan(math.tan(fov_h / 2.0) * aspect)

        self.footprint_x = 2.0 * drone_altitude * math.tan(fov_h / 2.0)  # metres, X (width)
        self.footprint_y = 2.0 * drone_altitude * math.tan(fov_v / 2.0)  # metres, Y (height)

        # Pixel size in world units
        self.px_size_x = self.footprint_x / img_width
        self.px_size_y = self.footprint_y / img_height

        # Frame accumulation buffer
        self._frame_buffer = []

        print(f"[MazeVision] Altitude={drone_altitude:.1f}m  "
              f"FOV_h={math.degrees(fov_h):.1f}°  FOV_v={math.degrees(fov_v):.1f}°  "
              f"Footprint={self.footprint_x:.2f}×{self.footprint_y:.2f}m")

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def compute_pixel_to_world_transform(self, drone_x=0.0, drone_y=0.0):
        """
        Return an affine transform matrix (3×3 homogeneous) mapping image pixel
        coordinates (px, py) to world (x, y) coordinates.

        Assumes the drone is directly above (drone_x, drone_y) looking straight down.
        Image convention: px increases right (+X world), py increases down (-Y world).
        """
        sx = self.footprint_x / self.img_w
        sy = self.footprint_y / self.img_h
        # Origin offset: image (0,0) = top-left = world NW corner
        ox = drone_x - self.footprint_x / 2.0
        oy = drone_y + self.footprint_y / 2.0

        # [x]   [sx   0   ox] [px]
        # [y] = [ 0  -sy  oy] [py]
        # [1]   [ 0   0   1 ] [ 1]
        T = np.array([
            [sx,   0.0,  ox],
            [0.0, -sy,   oy],
            [0.0,  0.0,  1.0],
        ])
        return T

    def pixel_to_world(self, px, py, drone_x=0.0, drone_y=0.0):
        """Convert image pixel (px, py) to world (x, y)."""
        T = self.compute_pixel_to_world_transform(drone_x, drone_y)
        p = T @ np.array([px, py, 1.0])
        return (p[0], p[1])

    def world_to_pixel(self, wx, wy, drone_x=0.0, drone_y=0.0):
        """Convert world (wx, wy) to image pixel (px, py)."""
        T = self.compute_pixel_to_world_transform(drone_x, drone_y)
        T_inv = np.linalg.inv(T)
        p = T_inv @ np.array([wx, wy, 1.0])
        return (p[0], p[1])

    # ------------------------------------------------------------------
    # Image processing
    # ------------------------------------------------------------------

    def process_image(self, rgb):
        """
        Convert a single RGB frame to a 13×13 binary occupancy grid.

        Args:
            rgb: np.ndarray of shape (H, W, 3) or (H, W, 4), dtype uint8.

        Returns:
            np.ndarray of shape (13, 13), dtype uint8  (0=free, 1=wall).
        """
        if not _CV2_AVAILABLE:
            return self._fallback_process(rgb)

        # 1. Convert to grayscale
        if rgb.shape[2] == 4:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGBA2GRAY)
        else:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        # 2. Adaptive threshold — walls appear dark, floor appears bright
        #    (Isaac Sim default: white floor, gray/dark walls)
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            blockSize=31,
            C=5,
        )

        # 3. Morphological cleanup: remove noise, fill small gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 4. Resize to grid shape
        small = cv2.resize(cleaned, (self.grid_w, self.grid_h),
                           interpolation=cv2.INTER_AREA)

        # 5. Binarize: >127 = wall
        grid = (small > 127).astype(np.uint8)

        return grid

    def _fallback_process(self, rgb):
        """Fallback without OpenCV: simple luminance threshold + nearest-neighbour resize."""
        gray = np.mean(rgb[:, :, :3], axis=2).astype(np.uint8)
        thresh = (gray < 128).astype(np.uint8) * 255

        # Nearest-neighbour resize to grid shape
        h, w = thresh.shape
        row_idx = (np.arange(self.grid_h) * h / self.grid_h).astype(int)
        col_idx = (np.arange(self.grid_w) * w / self.grid_w).astype(int)
        small = thresh[np.ix_(row_idx, col_idx)]
        return (small > 127).astype(np.uint8)

    # ------------------------------------------------------------------
    # Frame accumulation
    # ------------------------------------------------------------------

    def add_frame(self, rgb):
        """Process and accumulate a frame into the internal buffer."""
        grid = self.process_image(rgb)
        self._frame_buffer.append(grid)

    def get_median_grid(self):
        """
        Return median occupancy grid from accumulated frames.
        Median over binary values = majority vote (0 if >50% free, 1 if >50% wall).

        Returns:
            np.ndarray (13, 13) uint8, or None if no frames accumulated.
        """
        if not self._frame_buffer:
            return None
        stack = np.stack(self._frame_buffer, axis=0).astype(np.float32)
        median = np.median(stack, axis=0)
        return (median >= 0.5).astype(np.uint8)

    def clear_buffer(self):
        """Reset frame accumulation buffer."""
        self._frame_buffer.clear()

    def frame_count(self):
        return len(self._frame_buffer)

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

    def compute_iou(self, pred_grid, gt_grid):
        """
        Compute Intersection-over-Union between predicted and ground-truth grids.

        Args:
            pred_grid: np.ndarray (H, W) uint8  (0=free, 1=wall)
            gt_grid:   np.ndarray (H, W) uint8

        Returns:
            float in [0, 1]
        """
        pred = pred_grid.astype(bool)
        gt = gt_grid.astype(bool)
        intersection = np.logical_and(pred, gt).sum()
        union = np.logical_or(pred, gt).sum()
        if union == 0:
            return 1.0
        return float(intersection) / float(union)


# ---------------------------------------------------------------------------
# Standalone test (synthetic image)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Testing MazeVisionNode ...")

    node = MazeVisionNode(drone_altitude=20.0)

    # Pixel -> world -> pixel round-trip
    px, py = 320.0, 240.0  # center pixel
    wx, wy = node.pixel_to_world(px, py, drone_x=0.0, drone_y=0.0)
    px2, py2 = node.world_to_pixel(wx, wy, drone_x=0.0, drone_y=0.0)
    print(f"Pixel ({px:.0f},{py:.0f}) -> world ({wx:.3f},{wy:.3f}) -> pixel ({px2:.1f},{py2:.1f})")
    assert abs(px - px2) < 0.5 and abs(py - py2) < 0.5, "Round-trip failed!"

    # Corner check: top-left pixel should be NW corner of footprint
    wx_tl, wy_tl = node.pixel_to_world(0, 0)
    print(f"Top-left pixel -> world ({wx_tl:.3f}, {wy_tl:.3f})  "
          f"(expected ~{-node.footprint_x/2:.2f}, {node.footprint_y/2:.2f})")

    # Synthetic image: white background with a dark grid pattern (mock maze)
    img = np.ones((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8) * 200
    # Draw horizontal and vertical lines to simulate walls
    for i in range(0, IMG_HEIGHT, IMG_HEIGHT // 13):
        img[i:i+3, :, :] = 30
    for j in range(0, IMG_WIDTH, IMG_WIDTH // 13):
        img[:, j:j+3, :] = 30

    grid = node.process_image(img)
    print(f"\nProcessed grid shape: {grid.shape}")
    print("Grid (0=free, 1=wall):")
    for row in grid:
        print(' '.join(str(v) for v in row))

    # IoU self-consistency
    iou = node.compute_iou(grid, grid)
    assert iou == 1.0, f"Self IoU should be 1.0, got {iou}"
    print(f"\nSelf-IoU: {iou:.3f}  PASS")

    print("\nAll MazeVisionNode tests passed.")
