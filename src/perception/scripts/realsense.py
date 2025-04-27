import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)


# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for a new frame set
        frames = pipeline.wait_for_frames()

        # Get aligned color and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Apply colormap to depth image for visualization
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )

        # Resize depth image to match color (optional)
        depth_colormap_resized = depth_colormap

        # Stack both images side by side
        images = np.hstack((color_image, depth_colormap_resized))

        # Display
        cv2.imshow("RealSense (Color + Depth)", images)

        # Exit on key press
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
