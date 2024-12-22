import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
pipeline.start()

# Get the depth sensor intrinsics
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)  # Retrieve depth stream
intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

# Print the intrinsics
print("Width:", intrinsics.width)
print("Height:", intrinsics.height)
print("PPX (Principal Point X):", intrinsics.ppx)
print("PPY (Principal Point Y):", intrinsics.ppy)
print("Focal Length X:", intrinsics.fx)
print("Focal Length Y:", intrinsics.fy)
print("Distortion Model:", intrinsics.model)
print("Distortion Coefficients:", intrinsics.coeffs)

depth_video = depth_stream.as_video_stream_profile()

print("Supported Resolutions for Depth:")
for profile in depth_video.get_supported_formats():
    intr = profile.as_video_stream_profile().get_intrinsics()
    print(f"Resolution: {intr.width}x{intr.height}, FPS: {profile.fps()}")
pipeline.stop()