#!/usr/bin/env python3
import rclpy
import math
import sys
import numpy as np
from copy import deepcopy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import cv2 as cv

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

from yolo_msgs.msg import DetectionArray
from std_srvs.srv import Trigger


# Constants
TARGET_FRAME_ID = "base_link"
PROCESS_MASK_OPEN_ITERATIONS = 2  # removes protrusions/lines
PROCESS_MASK_ERODE_ITERATIONS = 1  # shrinks mask
MASK_KERNEL_SIZE = (5, 5)          # removes noise



# Depth viewer window name
DEPTH_WINDOW_NAME = "Depth Image (click to measure)"
FILTERED_DEPTH_WINDOW = "Filtered Depth"

MASK_WINDOW = "rack Mask"

DISHWASHER_THICKNESS = 0.15


class DishwasherPerceptionROS(Node):
    """ROS interface for chairs around table detection"""

    def __init__(self):
        super().__init__('is_door_open')

        # ROS setup
        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.br = CvBridge()

        # TF2 setup for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data storage
        self.current_frame = None
        self.depth_image = None
        self.camera_info = None
        self.depth_frame_id = None

        # Image dimensions for coordinate scaling
        self.color_width = None
        self.color_height = None
        self.depth_width = None
        self.depth_height = None

        # ── Depth click viewer state ──────────────────────────────────────────
        # Stores the last clicked pixel and its measured depth so it can be
        # drawn as an overlay on every frame refresh.
        self._click_pixel = None        # (x, y) in depth-image space
        self._click_depth_mm = None     # raw depth value (mm for 16UC1)
        self._click_depth_m = None      # converted depth in metres
        self._click_3d_pose = None      # Pose in TARGET_FRAME_ID (if TF available)
        # ─────────────────────────────────────────────────────────────────────

        # ── Color-image click state ───────────────────────────────────────────
        # Tracks clicks on the main "Machine" color window (1280×720 display).
        # Coordinates are stored in *color image* space (before resize) so they
        # can be passed straight into estimate_pose / depth lookup.
        self._color_click_pixel_display = None  # (x,y) in 1280×720 display space
        self._color_click_pixel_img     = None  # (x,y) scaled back to color-image space
        self._color_click_depth_mm      = None
        self._color_click_depth_m       = None
        # ─────────────────────────────────────────────────────────────────────

        self.ran = False

        # QoS profile
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/femto_mega/color/image_raw',
            self.camera_callback,
            qos
        )

        self.create_subscription(
            Image,
            '/femto_mega/depth/image_raw',
            self.depth_callback,
            qos
        )
        self.create_subscription(
            CameraInfo,
            '/femto_mega/depth/camera_info',
            self.camera_info_callback,
            10
        )

        # Timer — comment out during deployment
        self.timer = self.create_timer(
            0.1,
            self.mainTimer,
            callback_group=main_timer_cb_group
        )

        # Set up the interactive depth viewer window
        self.setup_depth_click_viewer()

        # Set up click-to-depth on the main color window
        self.setup_color_click_viewer()

        
        cv.namedWindow(FILTERED_DEPTH_WINDOW, cv.WINDOW_NORMAL)
        cv.resizeWindow(FILTERED_DEPTH_WINDOW, 848, 480)

        self.get_logger().info('DishwasherPerceptionROS Node initialized')


    def setup_depth_click_viewer(self):
        """
        Create the OpenCV window used for interactive depth inspection and
        register the mouse-click callback.

        Call once during __init__.  The window will be updated every time
        update_depth_click_viewer() is called from mainTimer.
        """
        cv.namedWindow(DEPTH_WINDOW_NAME, cv.WINDOW_NORMAL)
        cv.resizeWindow(DEPTH_WINDOW_NAME, 848, 480)  # sensible default size
        cv.setMouseCallback(DEPTH_WINDOW_NAME, self._depth_click_callback)
        self.get_logger().info(
            f'Depth click viewer ready — left-click anywhere in '
            f'"{DEPTH_WINDOW_NAME}" to read depth at that pixel.'
        )

    def _depth_click_callback(self, event, x, y, flags, param):
        """
        OpenCV mouse callback attached to the depth viewer window.

        On a left-button click:
          1. Reads the raw depth value at (x, y).
          2. Samples a 15×15 neighbourhood for a robust median estimate.
          3. Converts to metres.
          4. Optionally back-projects to a 3-D pose in TARGET_FRAME_ID via TF2.
          5. Stores results so update_depth_click_viewer() can draw the overlay.

        Args:
            event : OpenCV mouse-event type
            x, y  : Pixel coordinates in the *displayed* depth image
            flags : OpenCV event flags (unused)
            param : Extra parameter passed to setMouseCallback (unused)
        """
        if event != cv.EVENT_LBUTTONDOWN:
            return

        if self.depth_image is None:
            self.get_logger().warn('Click received but no depth image available yet.')
            return

        h, w = self.depth_image.shape[:2]

        # Guard against out-of-bounds clicks (can happen with window resizing)
        if not (0 <= x < w and 0 <= y < h):
            self.get_logger().warn(f'Click ({x}, {y}) is outside depth image ({w}×{h}).')
            return

        # ── Sample a 15×15 neighbourhood (robust against sensor noise) ───────
        region_size = 15
        half = region_size // 2
        y0, y1 = max(0, y - half), min(h, y + half + 1)
        x0, x1 = max(0, x - half), min(w, x + half + 1)
        region = self.depth_image[y0:y1, x0:x1]
        valid  = region[region > 0]

        if len(valid) == 0:
            self.get_logger().warn(
                f'Click ({x}, {y}): no valid depth in {region_size}×{region_size} neighbourhood.'
            )
            self._click_pixel    = (x, y)
            self._click_depth_mm = 0
            self._click_depth_m  = 0.0
            self._click_3d_pose  = None
            return

        # Median is more robust than the raw single-pixel value
        depth_mm = float(np.median(valid))
        depth_m  = depth_mm / 1000.0   # 16UC1 → metres (adjust if 32FC1)

        # Store raw click info
        self._click_pixel    = (x, y)
        self._click_depth_mm = depth_mm
        self._click_depth_m  = depth_m

        self.get_logger().info(
            f'[Depth click] pixel=({x}, {y})  '
            f'depth={depth_mm:.1f} mm  ({depth_m:.3f} m)'
        )

        # ── Optional: back-project to 3-D pose in TARGET_FRAME_ID ────────────
        if self.camera_info is not None and depth_m > 0:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy_cam = self.camera_info.k[5]

            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = (x - cx) * depth_m / fx
            pose.position.y = (y - cy_cam) * depth_m / fy
            pose.position.z = depth_m

            if self.depth_frame_id is not None:
                pose = self.transform_pose(pose, self.depth_frame_id, TARGET_FRAME_ID)
                # Reset orientation after transform (keep as identity)
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0

            self._click_3d_pose = pose
            self.get_logger().info(
                f'[Depth click] 3-D pose in {TARGET_FRAME_ID}: '
                f'x={pose.position.x:.3f} m  '
                f'y={pose.position.y:.3f} m  '
                f'z={pose.position.z:.3f} m'
            )
        else:
            self._click_3d_pose = None

    def update_depth_click_viewer(self):
        """
        Render the latest depth image into the interactive viewer window and
        draw an overlay showing the last clicked pixel's depth value.

        Call this every cycle from mainTimer (after depth_image is populated).
        The depth image is false-coloured with COLORMAP_JET for readability.
        """
        if self.depth_image is None:
            return

        # ── Normalise 16-bit depth to 8-bit for display ───────────────────────
        display = cv.normalize(
            self.depth_image, None, 0, 255, cv.NORM_MINMAX
        ).astype(np.uint8)
        display = cv.applyColorMap(display, cv.COLORMAP_JET)

        # ── Draw crosshair + annotation at last clicked pixel ─────────────────
        if self._click_pixel is not None:
            px, py = self._click_pixel
            radius  = 8
            color_cross = (255, 255, 255)   # white crosshair
            color_dot   = (0, 0, 255)       # red dot centre

            # Crosshair lines
            cv.line(display, (px - radius, py), (px + radius, py), color_cross, 1, cv.LINE_AA)
            cv.line(display, (px, py - radius), (px, py + radius), color_cross, 1, cv.LINE_AA)
            cv.circle(display, (px, py), 3, color_dot, -1)

            # Build annotation text
            if self._click_depth_m is not None and self._click_depth_m > 0:
                depth_txt = f'{self._click_depth_mm:.0f} mm  ({self._click_depth_m:.3f} m)'
            else:
                depth_txt = 'No depth'

            lines = [
                f'Pixel : ({px}, {py})',
                f'Depth : {depth_txt}',
            ]
            if self._click_3d_pose is not None:
                p = self._click_3d_pose.position
                lines.append(
                    f'3-D({TARGET_FRAME_ID}): '
                    f'({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m'
                )

            # Draw dark background box then white text
            font       = cv.FONT_HERSHEY_SIMPLEX
            font_scale = 0.45
            thickness  = 1
            line_h     = 18
            pad        = 6

            text_widths = [
                cv.getTextSize(l, font, font_scale, thickness)[0][0] for l in lines
            ]
            box_w = max(text_widths) + pad * 2
            box_h = line_h * len(lines) + pad * 2

            # Position box to the right of the crosshair if possible
            h_img, w_img = display.shape[:2]
            bx = px + radius + 4
            if bx + box_w > w_img:
                bx = px - radius - box_w - 4
            by = py - box_h // 2
            by = max(0, min(by, h_img - box_h))

            overlay_bg = display.copy()
            cv.rectangle(overlay_bg, (bx, by), (bx + box_w, by + box_h), (20, 20, 20), -1)
            cv.addWeighted(overlay_bg, 0.75, display, 0.25, 0, display)

            for i, line in enumerate(lines):
                cv.putText(
                    display, line,
                    (bx + pad, by + pad + line_h * (i + 1) - 4),
                    font, font_scale, (255, 255, 255), thickness, cv.LINE_AA
                )

        cv.imshow(DEPTH_WINDOW_NAME, display)
        cv.waitKey(1)

    def setup_color_click_viewer(self):
        """
        Register a mouse callback on the main "Machine" color window so that
        left-clicking anywhere on it shows the depth at that pixel directly
        on the image.

        Must be called *after* the window is first created (done inside
        draw_detections on the first frame).  We call namedWindow here in
        advance so OpenCV knows about it even before the first frame arrives.
        """
        cv.namedWindow("Machine", cv.WINDOW_NORMAL)
        cv.setMouseCallback("Machine", self._color_click_callback)
        self.get_logger().info(
            'Color click viewer ready — left-click anywhere in "Machine" '
            'window to read depth at that pixel.'
        )

    def _color_click_callback(self, event, x, y, flags, param):
        """
        Mouse callback for the main "Machine" display window (1280×720).

        The display frame is a *resized* version of the original color image,
        so we scale the click coordinates back to color-image space before
        looking up depth.  Depth is then fetched from the depth image using
        the same scale logic as estimate_pose().

        On left-click:
          • Scales (x, y) from 1280×720 display → original color-image space.
          • Further scales to depth-image space.
          • Samples a 15×15 neighbourhood median depth.
          • Stores results for draw_detections() to render on the next frame.
        """
        if event != cv.EVENT_LBUTTONDOWN:
            return

        if self.depth_image is None:
            self.get_logger().warn('Color-window click: no depth image yet.')
            return

        if self.color_width is None or self.depth_width is None:
            self.get_logger().warn('Color-window click: image dimensions not ready yet.')
            return

        # ── Scale from display (1280×720) → color image space ────────────────
        display_w, display_h = 1280, 720
        scale_to_color_x = self.color_width  / display_w
        scale_to_color_y = self.color_height / display_h
        cx_color = int(x * scale_to_color_x)
        cy_color = int(y * scale_to_color_y)

        # ── Scale from color image space → depth image space ─────────────────
        scale_to_depth_x = self.depth_width  / self.color_width
        scale_to_depth_y = self.depth_height / self.color_height
        cx_depth = int(cx_color * scale_to_depth_x)
        cy_depth = int(cy_color * scale_to_depth_y)

        dh, dw = self.depth_image.shape[:2]
        if not (0 <= cx_depth < dw and 0 <= cy_depth < dh):
            self.get_logger().warn(
                f'Color click ({x},{y}) maps to depth ({cx_depth},{cy_depth}) '
                f'which is out of bounds ({dw}×{dh}).'
            )
            return

        # ── Median depth in 15×15 neighbourhood ──────────────────────────────
        half = 7
        y0, y1 = max(0, cy_depth - half), min(dh, cy_depth + half + 1)
        x0, x1 = max(0, cx_depth - half), min(dw, cx_depth + half + 1)
        region = self.depth_image[y0:y1, x0:x1]
        valid  = region[region > 0]

        if len(valid) == 0:
            self.get_logger().warn(f'Color click ({x},{y}): no valid depth nearby.')
            self._color_click_pixel_display = (x, y)
            self._color_click_pixel_img     = (cx_color, cy_color)
            self._color_click_depth_mm      = 0
            self._color_click_depth_m       = 0.0
            return

        depth_mm = float(np.median(valid))
        depth_m  = depth_mm / 1000.0

        self._color_click_pixel_display = (x, y)          # for drawing on 1280×720
        self._color_click_pixel_img     = (cx_color, cy_color)
        self._color_click_depth_mm      = depth_mm
        self._color_click_depth_m       = depth_m

        self.get_logger().info(
            f'[Color click] display=({x},{y})  '
            f'color-img=({cx_color},{cy_color})  '
            f'depth={depth_mm:.0f} mm  ({depth_m:.3f} m)'
        )

    def camera_callback(self, data):
        """Store latest camera frame"""
        self.current_frame = self.br.imgmsg_to_cv2(data)
        if self.color_height is None:
            self.color_height = data.height
            self.color_width = data.width
            self.get_logger().info(f'Color image: {self.color_width}x{self.color_height}')


    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_frame_id = msg.header.frame_id
            if self.depth_height is None:
                self.depth_height = msg.height
                self.depth_width = msg.width
                self.get_logger().info(f'Depth image: {self.depth_width}x{self.depth_height}')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        self.camera_info = msg

    def transform_pose(self, pose, source_frame, target_frame):
        """
        Transform a pose from source frame to target frame using TF2

        Args:
            pose: Pose object
            source_frame: Source frame ID (e.g., 'femto_mega_depth_optical_frame')
            target_frame: Target frame ID (e.g., 'base_link')

        Returns:
            Transformed Pose object
        """
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose

            if not self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            ):
                self.get_logger().warn(
                    f'Transform from {source_frame} to {target_frame} not available',
                    throttle_duration_sec=5.0
                )
                return pose

            transformed_pose_stamped = self.tf_buffer.transform(
                pose_stamped,
                target_frame,
            )
            return transformed_pose_stamped.pose

        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform from {source_frame} to {target_frame}: {ex}',
                throttle_duration_sec=5.0
            )
            return pose

    def estimate_pose(self, xc, yc):
        """
        Convert pixel coordinates + depth to 3D pose

        Args:
            xc: X pixel coordinate (in color image space)
            yc: Y pixel coordinate (in color image space)

        Returns:
            Pose object with 3D position
        """
        pose = Pose()
        pose.orientation.w = 1.0

        if self.depth_image is None or self.camera_info is None:
            self.get_logger().warn('Missing depth image or camera info', throttle_duration_sec=5.0)
            return pose

        if self.color_width is None or self.depth_width is None:
            self.get_logger().warn('Image dimensions not yet available', throttle_duration_sec=5.0)
            return pose

        scale_x = self.depth_width / self.color_width
        scale_y = self.depth_height / self.color_height
        xc_depth = int(xc * scale_x)
        yc_depth = int(yc * scale_y)

        height, width = self.depth_image.shape

        if xc_depth < 0 or xc_depth >= width or yc_depth < 0 or yc_depth >= height:
            self.get_logger().warn(
                f'Pixel coordinates ({xc_depth}, {yc_depth}) out of bounds '
                f'(depth image: {width}x{height})'
            )
            return pose

        region_size = 15
        half_size = region_size // 2
        y_start = max(0, yc_depth - half_size)
        y_end = min(height, yc_depth + half_size + 1)
        x_start = max(0, xc_depth - half_size)
        x_end = min(width, xc_depth + half_size + 1)

        depth_region = self.depth_image[y_start:y_end, x_start:x_end]
        valid_depths = depth_region[depth_region > 0]

        if len(valid_depths) == 0:
            self.get_logger().error(f'No valid depth at pixel ({xc_depth}, {yc_depth})')
            return pose

        depth_mm = float(np.median(valid_depths))
        z = depth_mm / 1000.0

        self.get_logger().debug(f'Pixel ({xc_depth}, {yc_depth}): depth = {depth_mm:.1f} mm')

        if np.isnan(z) or z <= 0:
            return pose

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        pose.position.x = (xc_depth - cx) * z / fx
        pose.position.y = (yc_depth - cy) * z / fy
        pose.position.z = z

        if self.depth_frame_id is not None:
            pose = self.transform_pose(pose, self.depth_frame_id, TARGET_FRAME_ID)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            self.get_logger().debug(
                f'Transformed to {TARGET_FRAME_ID}: '
                f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
            )

        return pose

    def mainTimer(self):
        """Main processing loop — runs at 10 Hz"""
        if self.current_frame is None :
            return


        # Refresh the interactive depth viewer each cycle 
        self.update_depth_click_viewer()

        mask, info = self.create_single_depth_slice_mask(
            near_m=0.85,
            far_m=0.9,
            min_z_m=0.07,
            max_z_m = 0.2,
            scale_to_color=True,
        )

        if mask is None or info['num_pixels'] == 0:
            self.get_logger().error ('door is closed!')
            return

        self.get_logger().info(
            f"Slice [{info['near_mm']:.0f}, {info['far_mm']:.0f}] mm "
            f"→ {info['num_pixels']} px"
        )


        #processed_mask = self.process_mask(mask)
        #processed_mask = self.remove_small_regions(processed_mask, min_pixels=3500)
        #processed_mask = self.remove_big_regions(processed_mask, max_pixels=25000)

        # if processed_mask is not None:
        #     edges = self.detect_edges(processed_mask)
        #     lines = self.find_horizontal_lines(edges)
        #     window_title = f"Slice {info['near_mm']:.0f}-{info['far_mm']:.0f}mm"
        #     self.display_horizontal_lines(edges, lines, window_title)

        if mask is not None:
            window_title = f"Slice {info['near_mm']:.0f}-{info['far_mm']:.0f}mm z={info['min_z_mm']:.0f}-{info['max_z_mm']:.0f}mm"
            cv.imshow(window_title, mask)

            left, right = self.find_leftright_pixels(mask)

            if left is not None:
                # self.get_logger().info(f"Left:  x={left[0]}, y={left[1]}")
                # self.get_logger().info(f"Right: x={right[0]}, y={right[1]}")

                left_pose  = self.estimate_pose(left[0],  left[1])
                right_pose = self.estimate_pose(right[0], right[1])

                # self.get_logger().info(
                #     f"Left  3D: ({left_pose.position.x:.3f}, {left_pose.position.y:.3f}, {left_pose.position.z:.3f})"
                # )
                # self.get_logger().info(
                #     f"Right 3D: ({right_pose.position.x:.3f}, {right_pose.position.y:.3f}, {right_pose.position.z:.3f})"
                # )

                filtered = self.create_filtered_depth_image(min_y_m = right_pose.position.y + DISHWASHER_THICKNESS, max_y_m = left_pose.position.y - DISHWASHER_THICKNESS)
                if filtered is not None:
                    display = cv.normalize(filtered, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
                    display = cv.applyColorMap(display, cv.COLORMAP_JET)
                    cv.imshow(FILTERED_DEPTH_WINDOW, display)

                    mask, num_pixels = self.create_depth_mask_from_min(filtered, max_depth_mm=50)
                    
                    #processed_mask = self.process_mask(mask)
                    processed_mask = mask
                    if mask is not None:
                        center_x = mask.shape[1] // 2
                        
                        
                        # Find first white pixel (starting from top)
                        first_white_y = None
                        for y in range(mask.shape[0]):
                            if mask[y, center_x] != 0:
                                first_white_y = y
                                break
                        
                        # Find first black pixel after white region
                        first_black_y = None
                        if first_white_y is not None:
                            for y in range(first_white_y, mask.shape[0]):
                                if mask[y, center_x] == 0:
                                    first_black_y = y
                                    break

                        cv.line(mask, (center_x, 0), (center_x, mask.shape[0]), (255, 0, 255), 2)
                        
                        # Calculate y_to_consider as midpoint between white and black
                        if first_white_y is not None and first_black_y is not None:
                            y_to_consider = (first_white_y + first_black_y) // 2
                            cv.circle(mask, (center_x, y_to_consider), 3, (0, 255, 0), -1)
                            print(f"Pixel to be considered : ({center_x}, {y_to_consider})")

                            final_pose = self.estimate_pose(center_x, y_to_consider)
                            print(f"Final pose - X: {final_pose.position.x}, Y: {final_pose.position.y}, Z: {final_pose.position.z}")
                            
                            # Display pose on image
                            text = f"Pose: ({final_pose.position.x:.2f}, {final_pose.position.y:.2f}, {final_pose.position.z:.2f})"
                            cv.putText(mask, text, (mask.shape[1] - 350, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)


                        
                        
                        cv.imshow(MASK_WINDOW, processed_mask)

        cv.waitKey(1)


    def create_depth_mask_from_min(self, filtered_depth_image, max_depth_mm, scale_to_color=True):
        """
        Create a binary mask from the MINIMUM DEPTH found in a pre-filtered depth image,
        extending up to max_depth_mm in base_link X coordinates.
        
        This function:
        1. Back-projects pixels from the filtered depth image to camera space, then base_link frame
        2. Finds the minimum (closest) valid base_link X coordinate
        3. Creates a mask of all pixels between min_x and (min_x + max_depth_mm)
        
        Includes pixel if:
        - Depth is valid (> 0) in the filtered_depth_image
        - base_link X is in [min_x_mm, min_x_mm + max_depth_mm]
        
        Args:
            filtered_depth_image: Pre-filtered depth image (uint16, in mm) from create_filtered_depth_image()
                                Only pixels with non-zero values are considered
            max_depth_mm: Depth range window in millimeters (width of slice around minimum)
            scale_to_color: If True, returns mask scaled to color image size
            
        Returns:
            mask: Binary mask (uint8): 255 = valid, 0 = outside range
                or None if filtered_depth_image/camera_info unavailable
            info: dict with keys:
                'min_x_mm'   – minimum base_link X found
                'max_x_mm'   – max base_link X in range (min_x_mm + max_depth_mm)
                'num_pixels' – non-zero pixels in the mask
            
        Example:
            # First create a filtered depth image
            filtered_depth = self.create_filtered_depth_image(min_y_m=-0.1, max_y_m=0.1, 
                                                            min_z_m=0.45, max_z_m=0.7)
            # Then create mask from minimum X in that filtered region
            mask, info = self.create_depth_mask_from_min(filtered_depth, max_depth_mm=1500)
        """
        if filtered_depth_image is None:
            self.get_logger().warn('create_depth_mask_from_min: No filtered depth image provided')
            return None, {}

        if self.camera_info is None:
            self.get_logger().warn('create_depth_mask_from_min: No camera info available')
            return None, {}

        # ── 1. Back-project valid depth pixels into camera space ─────────────────
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        depth   = filtered_depth_image.astype(np.float64)
        depth_m = depth / 1000.0

        # Filter by valid readings only (depth > 0) in the filtered image
        valid = depth > 0

        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        z      = depth_m[valid]
        x_cam  = (u_grid[valid] - cx) * z / fx
        y_cam  = (v_grid[valid] - cy) * z / fy
        pts_cam = np.stack([x_cam, y_cam, z], axis=1)  # (N, 3)

        # ── 2. Transform to base_link (TARGET_FRAME_ID) ───────────────────────────
        pts_working = pts_cam.copy()

        if self.depth_frame_id is not None:
            mat = self._get_transform_matrix(self.depth_frame_id, TARGET_FRAME_ID)
            if mat is not None:
                R, t = mat[:3, :3], mat[:3, 3]
                pts_working = (R @ pts_cam.T).T + t
            else:
                self.get_logger().warn(
                    'create_depth_mask_from_min: TF unavailable, falling back to camera frame',
                    throttle_duration_sec=5.0,
                )

        # ── 3. Find minimum base_link X and create mask ──────────────────────────
        # Find the minimum X coordinate in base_link frame
        x_coords_m = pts_working[:, 0]
        
        if x_coords_m.size == 0:
            self.get_logger().warn('create_depth_mask_from_min: No valid transformed points')
            return None, {}
        
        min_x_m = x_coords_m.min()
        min_x_mm = min_x_m * 1000.0
        max_x_mm = min_x_mm + max_depth_mm
        
        # Convert to mm for comparison
        x_coords_mm = x_coords_m * 1000.0
        
        # Find pixels in the range [min_x_mm, max_x_mm]
        in_range = (x_coords_mm >= min_x_mm) & (x_coords_mm <= max_x_mm)
        range_indices = np.argwhere(valid)[in_range]
        
        # Build mask
        mask = np.zeros((h, w), dtype=np.uint8)
        if range_indices.shape[0] > 0:
            mask[range_indices[:, 0], range_indices[:, 1]] = 255
        
        num_pixels = int(range_indices.shape[0])

        # ── 4. Scale to color image size if requested ────────────────────────────
        if scale_to_color and self.color_width is not None and self.color_height is not None:
            mask = cv.resize(
                mask,
                (self.color_width, self.color_height),
                interpolation=cv.INTER_NEAREST,
            )

        self.get_logger().info(
            f'create_depth_mask_from_min: base_link X range [{min_x_mm:.0f}, {max_x_mm:.0f}] mm '
            f'→ {num_pixels} px'
        )

        info = {
            'min_x_mm':   min_x_mm,
            'max_x_mm':   max_x_mm,
            'num_pixels': num_pixels,
        }
        
        return mask, info

    

    def create_filtered_depth_image(self, min_y_m=-0.1, max_y_m=0.1, min_z_m=0.45, max_z_m=0.7):
        if self.depth_image is None or self.camera_info is None:
            return None

        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        depth   = self.depth_image.astype(np.float64)
        depth_m = depth / 1000.0
        valid   = (depth_m >= 0.01) & (depth_m <= 20.0)

        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        z     = depth_m[valid]
        x_cam = (u_grid[valid] - cx) * z / fx
        y_cam = (v_grid[valid] - cy) * z / fy
        pts_cam = np.stack([x_cam, y_cam, z], axis=1)

        pts_working = pts_cam.copy()
        if self.depth_frame_id is not None:
            mat = self._get_transform_matrix(self.depth_frame_id, TARGET_FRAME_ID)
            if mat is not None:
                R, t = mat[:3, :3], mat[:3, 3]
                pts_working = (R @ pts_cam.T).T + t

        valid_filter = (
            (pts_working[:, 1] >= min_y_m) & (pts_working[:, 1] <= max_y_m) &
            (pts_working[:, 2] >= min_z_m) & (pts_working[:, 2] <= max_z_m)
        )
        valid_indices = np.argwhere(valid)

        filtered = np.zeros_like(self.depth_image)
        keep_indices = valid_indices[valid_filter]
        if keep_indices.shape[0] > 0:
            filtered[keep_indices[:, 0], keep_indices[:, 1]] = \
                self.depth_image[keep_indices[:, 0], keep_indices[:, 1]]

        return filtered

    def find_leftright_pixels(self, mask):
        """
        Find the leftmost and rightmost non-zero pixels in a binary mask.

        Args:
            mask: np.ndarray (uint8) binary mask

        Returns:
            left  : (x, y) of the leftmost non-zero pixel, or None if mask is empty
            right : (x, y) of the rightmost non-zero pixel, or None if mask is empty
        """
        points = np.argwhere(mask > 0)

        if points.shape[0] == 0:
            return None, None

        left  = (int(points[np.argmin(points[:, 1]), 1]), int(points[np.argmin(points[:, 1]), 0]))
        right = (int(points[np.argmax(points[:, 1]), 1]), int(points[np.argmax(points[:, 1]), 0]))

        return left, right


    def display_horizontal_lines(self, mask, lines, window_title):
        display = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv.line(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.imshow(window_title, display)


    def find_horizontal_lines(self,edges, min_line_length=20, max_line_gap=30, hough_threshold=20):
        """
        Find horizontal lines using Hough Line Transform.
        
        Args:
            edges: Binary edge image from Canny detection
            min_line_length: Minimum length of line to detect (pixels)
            max_line_gap: Maximum gap between line segments to connect (pixels) - higher = more agressive merging
            hough_threshold: Minimum votes for a line to be detected
        
        Returns:
            lines: List of detected lines as [x1, y1, x2, y2] or None
        """
        if edges is None:
            return None
        
        lines = cv.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )
        
        return lines


    def detect_edges(self,mask, canny_low=50, canny_high=150):
        """
        Apply Canny edge detection to the mask.
        
        Args:
            mask: Input binary mask (black and white image)
            canny_low: Lower threshold for Canny
            canny_high: Upper threshold for Canny
        
        Returns:
            edges: Binary image with detected edges
        """
        edges = cv.Canny(mask, canny_low, canny_high)
        return edges
           


    def process_mask(self, mask_img, kernel_size=MASK_KERNEL_SIZE,
                     open_iterations=PROCESS_MASK_OPEN_ITERATIONS,
                     erode_iterations=PROCESS_MASK_ERODE_ITERATIONS):
        kernel = np.ones(kernel_size, np.uint8)
        processed_mask = cv.morphologyEx(mask_img, cv.MORPH_OPEN, kernel, iterations=open_iterations)
        processed_mask = cv.erode(processed_mask, kernel, iterations=erode_iterations)
        if cv.countNonZero(processed_mask) == 0:
            return mask_img.copy()
        return processed_mask

  

    def _get_transform_matrix(self, source_frame: str, target_frame: str):
        """
        Return the TF2 transform from source_frame → target_frame as a (4, 4)
        float64 homogeneous matrix, or None if the transform is unavailable.

        The matrix can be used to transform batches of 3-D points efficiently:
            pts_target = (R @ pts_source.T).T + t
        where R = matrix[:3, :3] and t = matrix[:3, 3].
        """
        try:
            if not self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            ):
                self.get_logger().warn(
                    f'_get_transform_matrix: no transform {source_frame} → {target_frame}',
                    throttle_duration_sec=5.0,
                )
                return None

            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
            trans = t.transform.translation
            rot   = t.transform.rotation          # quaternion (x, y, z, w)

            # Quaternion → 3×3 rotation matrix
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            R = np.array([
                [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
                [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
                [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
            ], dtype=np.float64)

            mat = np.eye(4, dtype=np.float64)
            mat[:3, :3] = R
            mat[:3,  3] = [trans.x, trans.y, trans.z]
            return mat

        except TransformException as ex:
            self.get_logger().error(
                f'_get_transform_matrix: {ex}',
                throttle_duration_sec=5.0,
            )
            return None

    # ── modified mask function ────────────────────────────────────────────────

    

    def remove_big_regions(self, mask_img, max_pixels=500):
        """
        Remove all connected white regions bigger than max_pixels.
        """
        num_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask_img, connectivity=8)

        output = np.zeros_like(mask_img)
        for label in range(1, num_labels):  # skip 0 — that's the background
            area = stats[label, cv.CC_STAT_AREA]
            if area <= max_pixels:
                output[labels == label] = 255

        return output

    def remove_small_regions(self, mask_img, min_pixels=500):
        """
        Remove all connected white regions smaller than min_pixels.
        """
        num_labels, labels, stats, _ = cv.connectedComponentsWithStats(mask_img, connectivity=8)

        output = np.zeros_like(mask_img)
        for label in range(1, num_labels):  # skip 0 — that's the background
            area = stats[label, cv.CC_STAT_AREA]
            if area >= min_pixels:
                output[labels == label] = 255

        return output

    def create_single_depth_slice_mask(self, near_m, far_m, min_z_m=0.10, max_z_m = 0.2, scale_to_color=True):
        """
        Create a single binary mask for pixels whose back-projected base_link X
        coordinate falls within [near_m, far_m], ignoring pixels with camera-space
        Z (depth) below min_z_m.

        Args:
            near_m        : Near edge of the slice in base_link X, metres
            far_m         : Far  edge of the slice in base_link X, metres
            min_z_m       : Minimum camera-space depth to accept (default 0.10 m)
            scale_to_color: If True, resize the mask to the colour-image resolution

        Returns:
            mask       : np.ndarray (uint8) — 255 in-slice, 0 elsewhere
                        or None if depth/camera_info is unavailable
            info       : dict with keys
                        'near_mm'    – slice start in mm (base_link X)
                        'far_mm'     – slice end   in mm (base_link X)
                        'num_pixels' – non-zero pixels in the mask
        """
        if self.depth_image is None:
            self.get_logger().warn('create_depth_slice_mask: No depth image available')
            return None, {}

        if self.camera_info is None:
            self.get_logger().warn('create_depth_slice_mask: No camera info available')
            return None, {}

        # ── 1. Back-project valid depth pixels into camera space ─────────────────
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        depth   = self.depth_image.astype(np.float64)
        depth_m = depth / 1000.0

        # Reject pixels with no reading or below the minimum Z threshold
        #valid = (depth > 0) & (depth_m >= min_z_m)
        #valid = (depth_m >= min_z_m)
        valid = (depth_m >= min_z_m) & (depth_m <= 10.0)

        h, w       = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        z      = depth_m[valid]
        x_cam  = (u_grid[valid] - cx) * z / fx
        y_cam  = (v_grid[valid] - cy) * z / fy
        pts_cam = np.stack([x_cam, y_cam, z], axis=1)   # (N, 3)

        # ── 2. Transform to base_link (TARGET_FRAME_ID) ───────────────────────────
        pts_working = pts_cam.copy()

        if self.depth_frame_id is not None:
            mat = self._get_transform_matrix(self.depth_frame_id, TARGET_FRAME_ID)
            if mat is not None:
                R, t = mat[:3, :3], mat[:3, 3]
                pts_working = (R @ pts_cam.T).T + t
            else:
                self.get_logger().warn(
                    'create_depth_slice_mask: TF unavailable, falling back to camera frame',
                    throttle_duration_sec=5.0,
                )

        # ── 3. Build the mask ─────────────────────────────────────────────────────
        # near_mm = near_m * 1000.0
        # far_mm  = far_m  * 1000.0
        # x_coords_mm   = pts_working[:, 0] * 1000.0
        # valid_indices = np.argwhere(valid)              # (N, 2) — (row, col) pairs

        near_mm = near_m * 1000.0
        far_mm  = far_m  * 1000.0
        valid_z       = (pts_working[:, 2] >= min_z_m) & (pts_working[:, 2] <= max_z_m)
        x_coords_mm   = pts_working[valid_z, 0] * 1000.0
        valid_indices = np.argwhere(valid)[valid_z]

        in_slice      = (x_coords_mm >= near_mm) & (x_coords_mm <= far_mm)
        slice_indices = valid_indices[in_slice]

        mask = np.zeros((h, w), dtype=np.uint8)
        if slice_indices.shape[0] > 0:
            mask[slice_indices[:, 0], slice_indices[:, 1]] = 255

        num_pixels = int(slice_indices.shape[0])

        if scale_to_color and self.color_width is not None and self.color_height is not None:
            mask = cv.resize(
                mask,
                (self.color_width, self.color_height),
                interpolation=cv.INTER_NEAREST,
            )

        self.get_logger().debug(
            f'create_depth_slice_mask: [{near_mm:.0f}, {far_mm:.0f}] mm (base_link X), '
            f'min_z={min_z_m*1000:.0f} mm → {num_pixels} px'
        )

        info = {
            'near_mm':    near_mm,
            'far_mm':     far_mm,
            'num_pixels': num_pixels,
            'min_z_mm':   min_z_m * 1000.0,
            'max_z_mm':   max_z_m * 1000.0,
        }
        return mask, info

    def create_depth_slice_masks1(self, start_m, end_m, discrete_size_m, scale_to_color=True):
        """
        Create a list of binary masks, each covering a discrete slice along the
        X-axis of TARGET_FRAME_ID (base_link), rather than raw camera depth.

        The range [start_m, end_m] is divided into equal-width windows of
        discrete_size_m along base_link X.  Each window produces one mask where
        pixels whose back-projected X coordinate falls within that window are set
        to 255, everything else to 0.

        Args:
            start_m        : Near edge of the zone of interest in base_link X, metres
            end_m          : Far  edge of the zone of interest in base_link X, metres
            discrete_size_m: Width of each slice in metres
            scale_to_color : If True, resize every mask to the colour-image resolution

        Returns:
            masks      : list of np.ndarray (uint8), one per slice
            slice_info : list of dicts with keys
                        'index'      – 0-based slice index
                        'near_mm'    – slice start in mm (base_link X)
                        'far_mm'     – slice end   in mm (base_link X)
                        'num_pixels' – non-zero pixels in this mask
        """
        if self.depth_image is None:
            self.get_logger().warn('create_depth_slice_masks: No depth image available')
            return [], []

        if self.camera_info is None:
            self.get_logger().warn('create_depth_slice_masks: No camera info available')
            return [], []

        # ── 1. Back-project every valid depth pixel into camera space ────────────
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        depth = self.depth_image.astype(np.float64)
        valid = depth > 0
        depth_m = np.where(valid, depth / 1000.0, 0.0)

        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        z      = depth_m[valid]
        x_cam  = (u_grid[valid] - cx) * z / fx
        y_cam  = (v_grid[valid] - cy) * z / fy
        pts_cam = np.stack([x_cam, y_cam, z], axis=1)          # (N, 3)

        # ── 2. Transform to base_link (TARGET_FRAME_ID) ───────────────────────────
        pts_working = pts_cam.copy()

        if self.depth_frame_id is not None:
            mat = self._get_transform_matrix(self.depth_frame_id, TARGET_FRAME_ID)
            if mat is not None:
                R, t = mat[:3, :3], mat[:3, 3]
                pts_working = (R @ pts_cam.T).T + t
            else:
                self.get_logger().warn(
                    'create_depth_slice_masks: TF unavailable, falling back to camera frame',
                    throttle_duration_sec=5.0,
                )

        # x_coords_world holds the base_link X for every valid pixel
        x_coords_world = pts_working[:, 0]          # shape (N,)
        valid_indices  = np.argwhere(valid)          # shape (N, 2) — (row, col) pairs

        # ── 3. Build one mask per slice ───────────────────────────────────────────
        start_mm         = start_m         * 1000.0
        end_mm           = end_m           * 1000.0
        discrete_size_mm = discrete_size_m * 1000.0
        x_coords_mm      = x_coords_world  * 1000.0  # convert once for comparisons

        num_slices = int(math.ceil((end_mm - start_mm) / discrete_size_mm))

        self.get_logger().info(
            f'create_depth_slice_masks (base_link X): '
            f'{start_mm:.0f}–{end_mm:.0f} mm, '
            f'slice={discrete_size_mm:.0f} mm → {num_slices} masks'
        )

        masks      = []
        slice_info = []

        for i in range(num_slices):
            near_mm = start_mm + i * discrete_size_mm
            far_mm  = min(near_mm + discrete_size_mm, end_mm)

            # Select points whose base_link X falls in this window
            in_slice = (x_coords_mm >= near_mm) & (x_coords_mm <= far_mm)
            slice_indices = valid_indices[in_slice]   # (M, 2) — (row, col) in depth image

            # Paint selected pixels into a depth-resolution mask
            mask = np.zeros((h, w), dtype=np.uint8)
            if slice_indices.shape[0] > 0:
                mask[slice_indices[:, 0], slice_indices[:, 1]] = 255

            num_pixels = int(slice_indices.shape[0])

            if scale_to_color and self.color_width is not None and self.color_height is not None:
                mask = cv.resize(
                    mask,
                    (self.color_width, self.color_height),
                    interpolation=cv.INTER_NEAREST,
                )

            masks.append(mask)
            slice_info.append({
                'index':      i,
                'near_mm':    near_mm,
                'far_mm':     far_mm,
                'num_pixels': num_pixels,
            })

            self.get_logger().debug(
                f'  slice {i:3d}: [{near_mm:.0f}, {far_mm:.0f}] mm (base_link X) '
                f'→ {num_pixels} px'
            )

        return masks, slice_info

    def create_depth_slice_masks(self, start_m, end_m, discrete_size_m, scale_to_color=True):
        """
        Create a list of binary masks, each covering a discrete depth slice
        from start_m to end_m.

        The range [start_m, end_m] is divided into equal-width windows of
        discrete_size_m.  Each window produces one mask where pixels whose
        depth falls within that window are set to 255, everything else to 0.
        The last slice is clipped to end_m even if the window would overshoot.

        Args:
            start_m        : Near edge of the zone of interest, in metres (e.g. 0.85)
            end_m          : Far  edge of the zone of interest, in metres (e.g. 1.15)
            discrete_size_m: Width of each slice in metres           (e.g. 0.10)
            scale_to_color : If True, resize every mask to the colour-image resolution

        Returns:
            masks      : list of np.ndarray (uint8), one per slice
            slice_info : list of dicts, each with keys
                        'index'         – 0-based slice index
                        'near_mm'       – slice start in mm
                        'far_mm'        – slice end   in mm
                        'num_pixels'    – non-zero pixels in this mask

        Example:
            masks, info = self.create_depth_slice_masks(
                start_m=0.85, end_m=1.15, discrete_size_m=0.10
            )
            # Produces 3 masks covering [850,950), [950,1050), [1050,1150] mm
        """
        if self.depth_image is None:
            self.get_logger().warn('create_depth_slice_masks: No depth image available')
            return [], []

        # Convert to millimetres for direct comparison with the depth image
        start_mm        = start_m        * 1000.0
        end_mm          = end_m          * 1000.0
        discrete_size_mm = discrete_size_m * 1000.0

        num_slices = int(math.ceil((end_mm - start_mm) / discrete_size_mm))

        self.get_logger().info(
            f'create_depth_slice_masks: {start_mm:.0f}–{end_mm:.0f} mm, '
            f'slice={discrete_size_mm:.0f} mm → {num_slices} masks'
        )

        masks      = []
        slice_info = []

        for i in range(num_slices):
            near_mm = start_mm + i * discrete_size_mm
            far_mm  = min(near_mm + discrete_size_mm, end_mm)  # clip last slice

            valid_mask = (
                (self.depth_image > 0) &
                (self.depth_image >= near_mm) &
                (self.depth_image <= far_mm)
            )

            mask = np.zeros(self.depth_image.shape, dtype=np.uint8)
            mask[valid_mask] = 255

            num_pixels = int(np.count_nonzero(valid_mask))

            if scale_to_color and self.color_width is not None and self.color_height is not None:
                mask = cv.resize(
                    mask,
                    (self.color_width, self.color_height),
                    interpolation=cv.INTER_NEAREST
                )

            masks.append(mask)
            slice_info.append({
                'index':      i,
                'near_mm':    near_mm,
                'far_mm':     far_mm,
                'num_pixels': num_pixels,
            })

            self.get_logger().debug(
                f'  slice {i:3d}: [{near_mm:.0f}, {far_mm:.0f}] mm  '
                f'→ {num_pixels} px'
            )

        return masks, slice_info

    def depth_buckets(self, scale_to_color=True, bucket_offset=0):
        if self.depth_image is None or self.camera_info is None:
            self.get_logger().warn('depth_buckets: missing data')
            return

        if self.current_frame is None:
            self.get_logger().warn('depth_buckets: no color frame yet')
            return

        # -- 1. Back-projection --
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        depth = self.depth_image.astype(np.float64)
        valid = depth > 0
        depth_m = np.where(valid, depth / 1000.0, 0.0)

        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        z = depth_m[valid]
        x_cam = (u_grid[valid] - cx) * z / fx
        y_cam = (v_grid[valid] - cy) * z / fy
        pts_cam = np.stack([x_cam, y_cam, z], axis=1)

        # -- 2. Transform --
        pts_working = pts_cam

        if self.depth_frame_id is not None:
            mat = self._get_transform_matrix(self.depth_frame_id, TARGET_FRAME_ID)
            if mat is not None:
                R, t = mat[:3, :3], mat[:3, 3]
                pts_working = (R @ pts_cam.T).T + t

        if pts_working.shape[0] == 0:
            return

        # -- 3. Bucket logic --
        # x_coords = pts_working[:, 0]
        # valid_pixel_indices = np.argwhere(valid)  # shape (N, 2): each row is (row, col) in depth image
        # bucket_xs = np.unique(np.round(x_coords, decimals=3))

        # buckets = []
        # for bx in bucket_xs:
        #     mask = np.abs(x_coords - bx) < 0.0005
        #     pts_in_bucket = pts_working[mask]
        #     pixel_indices = valid_pixel_indices[mask]  # (row, col) pairs for these points
        #     buckets.append((bx, pts_in_bucket, pixel_indices))
        #     #print(f"Bucket x={bx:.4f}: {len(pts_in_bucket)} points")

        #     #for pt in pts_in_bucket:
        #         #print(f"  {pt}")


        # print(f"Total buckets: {len(buckets)}")

        # -- 3. Bucket logic --
        x_coords = pts_working[:, 0]

        # 1. Create a mask for the specific X range
        range_mask = (x_coords >= 0.8) & (x_coords <= 1.2)

        # 2. Apply that mask to all relevant arrays
        pts_filtered = pts_working[range_mask]
        valid_indices_filtered = np.argwhere(valid)[range_mask]
        x_coords_filtered = x_coords[range_mask]

        # 3. Now find unique buckets only within that filtered set
        bucket_xs = np.unique(np.round(x_coords_filtered, decimals=3))

        buckets = []
        for bx in bucket_xs:
            # Use the filtered arrays here for better performance
            mask = np.abs(x_coords_filtered - bx) < 0.0005
            pts_in_bucket = pts_filtered[mask]
            pixel_indices = valid_indices_filtered[mask]
            
            buckets.append((bx, pts_in_bucket, pixel_indices))

        print(f"Total buckets in range [0.8, 1.2]: {len(buckets)}")

        # # -- 4. Top 10 buckets by size --
        # buckets.sort(key=lambda b: len(b[1]), reverse=True)
        # top10 = buckets[bucket_offset:bucket_offset + 10]

        # # Distinct BGR colors for up to 10 buckets
        # colors = [
        #     (255,  64,  64),   # blue-ish
        #     ( 64, 255,  64),   # green
        #     ( 64,  64, 255),   # red
        #     (255, 255,  64),   # cyan
        #     ( 64, 255, 255),   # yellow
        #     (255,  64, 255),   # magenta
        #     (255, 165,   0),   # orange
        #     (128,   0, 255),   # purple
        #     (  0, 255, 165),   # mint
        #     (255, 192, 128),   # peach
        # ]

        # # -- 5. Paint pixels on color image --
        # display = self.current_frame.copy()

        # # Scale depth pixel coords to color image space if needed
        # scale_x = self.color_width  / w if self.color_width  else 1.0
        # scale_y = self.color_height / h if self.color_height else 1.0

        # for i, (bx, pts, pixel_indices) in enumerate(top10):
        #     color = colors[i]
        #     # pixel_indices are (row, col) in depth image space — scale to color image
        #     rows = (pixel_indices[:, 0] * scale_y).astype(int)
        #     cols = (pixel_indices[:, 1] * scale_x).astype(int)
        #     # Clamp to display bounds
        #     rows = np.clip(rows, 0, display.shape[0] - 1)
        #     cols = np.clip(cols, 0, display.shape[1] - 1)
        #     display[rows, cols] = color
        #     print(f"Top bucket #{i+1}: x={bx:.4f}, {len(pts)} points, color={color}")

        # cv.imshow("Depth Buckets", display)
        # cv.waitKey(1)

        # -- 4. Prepare for All Buckets --
        # We don't sort by size anymore; let's keep them in X-order or just use all
        num_buckets = len(buckets)
        if num_buckets == 0:
            cv.imshow("Depth Buckets", self.current_frame)
            cv.waitKey(1)
            return

        display = self.current_frame.copy()
        
        # Scale factors for depth-to-color mapping
        scale_x = self.color_width  / w if self.color_width  else 1.0
        scale_y = self.color_height / h if self.color_height else 1.0

        # -- 5. Paint Every Bucket --
        for i, (bx, pts, pixel_indices) in enumerate(buckets):
            # Generate a pseudo-random or sequential color based on the index
            # This ensures each bucket gets a distinct color even if there are 100+
            hue = int(180 * i / num_buckets) # OpenCV Hues go from 0-180
            color_hsv = np.uint8([[[hue, 255, 255]]])
            color_bgr = cv.cvtColor(color_hsv, cv.COLOR_HSV2BGR)[0][0]
            # Convert to standard python tuple for OpenCV
            color = tuple(int(c) for c in color_bgr)

            # Map depth pixels to color image pixels
            rows = np.clip((pixel_indices[:, 0] * scale_y).astype(int), 0, display.shape[0] - 1)
            cols = np.clip((pixel_indices[:, 1] * scale_x).astype(int), 0, display.shape[1] - 1)
            
            display[rows, cols] = color

        cv.imshow("Depth Buckets", display)
        cv.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    node = DishwasherPerceptionROS()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()