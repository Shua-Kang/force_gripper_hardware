#!/usr/bin/env python3

import threading
import time
import numpy as np
import serial
import cv2
from scipy.ndimage import gaussian_filter

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

import force_gripper


THRESHOLD = 20
NOISE_SCALE = 60
BAUD = 2_000_000
ROWS = 16
COLS = 32

# Set to True if you want the local debug OpenCV window
ENABLE_LOCAL_PREVIEW = False


class TactileReader(object):
    """
    Per-sensor reader.
    - Handles serial streaming
    - Maintains baseline
    - Produces normalized contact map (ROWS x COLS, float32 in [0,1])
    - Supports runtime re-baseline via self.request_reinit()
    """

    def __init__(self, port, baud=BAUD, rows=ROWS, cols=COLS, name="left"):
        self.port = port
        self.baud = baud
        self.rows = rows
        self.cols = cols
        self.name = name

        self.lock = threading.Lock()

        # public output buffer
        self.current_contact_norm = np.zeros((self.rows, self.cols), dtype=np.float32)

        # internal state
        self._median_baseline = np.zeros((self.rows, self.cols), dtype=np.float32)
        self._stop_flag = False
        self._reinit_flag = True  # force warmup on startup
        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        self._ser.flush()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag = True
        if self._thread.is_alive():
            self._thread.join()
        self._ser.close()

    def request_reinit(self):
        """
        Ask the reader thread to redo baseline/warmup.
        """
        with self.lock:
            self._reinit_flag = True

    def _warmup_baseline(self):
        """
        Gather ~30 frames and compute median baseline.
        Returns True if success.
        """
        warmup_samples = []
        frame_candidate = []
        last_frame_time = time.time()

        while not self._stop_flag:
            if self._ser.in_waiting > 0:
                try:
                    line = self._ser.readline().decode("utf-8").strip()
                except Exception:
                    line = ""

                # packet boundary
                if len(line) < 10:
                    if frame_candidate and len(frame_candidate) == self.rows:
                        frame_arr = np.array(frame_candidate, dtype=np.float32)
                        warmup_samples.append(frame_arr)

                        # optional fps debug
                        now = time.time()
                        # rospy.logdebug("[%s] warmup fps: %.2f", self.name, 1.0 / (now - last_frame_time))
                        last_frame_time = now
                    frame_candidate = []

                    if len(warmup_samples) >= 30:
                        break
                    continue

                # row line
                vals = line.split()
                try:
                    row_int = [int(v) for v in vals]
                except ValueError:
                    row_int = []
                if row_int:
                    frame_candidate.append(row_int)

        if len(warmup_samples) == 0:
            # fallback
            baseline = np.zeros((self.rows, self.cols), dtype=np.float32)
        else:
            stack = np.stack(warmup_samples, axis=0)  # (N, rows, cols)
            baseline = np.median(stack, axis=0).astype(np.float32)

        with self.lock:
            self._median_baseline = baseline
            self._reinit_flag = False

        rospy.loginfo("[%s] Baseline initialized with %d samples", self.name, len(warmup_samples))
        return True

    def _stream_loop_once(self):
        """
        Read one "frame" (packet), update current_contact_norm.
        Returns True if a full frame was processed.
        """
        frame_candidate = []
        start_t = time.time()

        # collect rows for one frame
        while not self._stop_flag:
            if self._ser.in_waiting == 0:
                # Avoid tight spin
                time.sleep(0.001)
                # If we hang too long, break
                if (time.time() - start_t) > 0.1:
                    break
                continue

            try:
                line = self._ser.readline().decode("utf-8").strip()
            except Exception:
                line = ""

            # boundary condition
            if len(line) < 10:
                # got end of frame
                if frame_candidate and len(frame_candidate) == self.rows:
                    raw_frame = np.array(frame_candidate, dtype=np.float32)

                    with self.lock:
                        baseline = self._median_baseline.copy()

                    # subtract baseline and threshold
                    contact_data = raw_frame - baseline - THRESHOLD
                    contact_data = np.clip(contact_data, 0, 100)

                    max_val = float(np.max(contact_data))
                    if max_val < THRESHOLD:
                        contact_norm = contact_data / NOISE_SCALE
                    else:
                        if max_val == 0:
                            contact_norm = contact_data
                        else:
                            contact_norm = contact_data / max_val

                    # write current_contact_norm
                    with self.lock:
                        self.current_contact_norm = contact_norm.astype(np.float32)

                    return True

                # bad/incomplete frame
                return False

            # still inside frame (row data)
            vals = line.split()
            try:
                row_int = [int(v) for v in vals]
            except ValueError:
                row_int = []
            if row_int:
                frame_candidate.append(row_int)

        return False

    def _run(self):
        """
        Thread main:
        - if _reinit_flag: redo warmup baseline
        - else: read streaming frames into current_contact_norm
        """
        while not self._stop_flag:
            need_reinit = False
            with self.lock:
                need_reinit = self._reinit_flag

            if need_reinit:
                self._warmup_baseline()
                continue

            self._stream_loop_once()


def temporal_filter(new_frame, prev_frame, alpha=0.2):
    if prev_frame is None:
        return new_frame
    return alpha * new_frame + (1.0 - alpha) * prev_frame


def to_ros_image_msg(frame_float32):
    """
    Convert (H,W) float32 array to sensor_msgs/Image with encoding "32FC1".
    frame_float32 is assumed contiguous.
    """
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = frame_float32.shape[0]
    msg.width = frame_float32.shape[1]
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = frame_float32.shape[1] * 4  # float32 = 4 bytes
    msg.data = np.asarray(frame_float32, dtype=np.float32).tobytes()
    return msg


def main():
    rospy.init_node("dual_tactile_node")

    # detect ports
    left_port = force_gripper.utils.find_port_by_name("left")
    right_port = force_gripper.utils.find_port_by_name("right")
    rospy.loginfo("Left tactile port:  %s", left_port)
    rospy.loginfo("Right tactile port: %s", right_port)

    reader_left = TactileReader(port=left_port, baud=BAUD, rows=ROWS, cols=COLS, name="left")
    reader_right = TactileReader(port=right_port, baud=BAUD, rows=ROWS, cols=COLS, name="right")

    pub_left = rospy.Publisher("/tactile_left/image_raw", Image, queue_size=1)
    pub_right = rospy.Publisher("/tactile_right/image_raw", Image, queue_size=1)

    # optional OpenCV preview setup
    prev_left_vis = None
    prev_right_vis = None
    if ENABLE_LOCAL_PREVIEW:
        win_h = ROWS * 30
        win_w = COLS * 30 * 2
        cv2.namedWindow("Dual Tactile Preview", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Dual Tactile Preview", win_w, win_h)

    # service to trigger re-baseline
    def handle_reinit(req):
        rospy.loginfo("Received tactile reinit request.")
        reader_left.request_reinit()
        reader_right.request_reinit()
        return TriggerResponse(success=True, message="Baseline reinit triggered.")

    srv = rospy.Service("/tactile/reinit", Trigger, handle_reinit)

    rate_hz = rospy.get_param("~publish_rate_hz", 100.0)
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        # grab current frames atomically
        with reader_left.lock:
            left_frame = reader_left.current_contact_norm.copy()
        with reader_right.lock:
            right_frame = reader_right.current_contact_norm.copy()

        # temporal smoothing for preview only
        if ENABLE_LOCAL_PREVIEW:
            left_smooth = temporal_filter(left_frame, prev_left_vis, alpha=0.2)
            right_smooth = temporal_filter(right_frame, prev_right_vis, alpha=0.2)
            prev_left_vis = left_smooth
            prev_right_vis = right_smooth

            # scale to [0,255] uint8
            left_u8 = np.clip(left_smooth * 255.0, 0, 255).astype(np.uint8)
            right_u8 = np.clip(right_smooth * 255.0, 0, 255).astype(np.uint8)

            # color maps
            left_color = cv2.applyColorMap(left_u8, cv2.COLORMAP_VIRIDIS)
            right_color = cv2.applyColorMap(right_u8, cv2.COLORMAP_VIRIDIS)

            both = np.concatenate([left_color, right_color], axis=1)
            cv2.imshow("Dual Tactile Preview", both)
            cv2.waitKey(1)
        # import pdb;pdb.set_trace()
        # publish ROS images (raw float32 maps)
        # print(sum(left_frame[12:16,:]))
        # print(sum(right_frame[12:16,:]))

        pub_left.publish(to_ros_image_msg(left_frame[4:,:]))
        pub_right.publish(to_ros_image_msg(right_frame[4:,:]))

        r.sleep()

    # cleanup
    if ENABLE_LOCAL_PREVIEW:
        cv2.destroyAllWindows()
    reader_left.stop()
    reader_right.stop()


if __name__ == "__main__":
    main()
