import cv2
import pyrealsense2 as rs

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert image to numpy array
            color_image = np.asanyarray(color_frame.get_data())

            # Show image
            cv2.imshow('RealSense Color Stream', color_image)

            # Break loop on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    import numpy as np
    main()
