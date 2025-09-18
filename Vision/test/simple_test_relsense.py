import pyrealsense2 as rs
import numpy as np, cv2

p = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

prof = p.start(cfg)
align = rs.align(rs.stream.color)

try:
    while True:
        frames = p.wait_for_frames()
        frames = align.process(frames)
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d or not c:
            continue
        depth = np.asanyarray(d.get_data())
        color = np.asanyarray(c.get_data())
        depth_viz = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow("color | depth", np.hstack([color, depth_viz]))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    p.stop()
    cv2.destroyAllWindows()
