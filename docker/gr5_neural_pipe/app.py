import os
import json
import time
import threading
import math
from contextlib import asynccontextmanager
import cv2
import numpy as np
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from ultralytics import YOLO
from dotenv import load_dotenv

import rclpy
from std_msgs.msg import String

dotenv_path = os.path.join(os.path.dirname(__file__), "..", ".env")
load_dotenv(dotenv_path=dotenv_path)

model = YOLO("yolov8n-pose.pt")

CAM_SRC = os.getenv("CAMERA_SOURCE", "0")
if isinstance(CAM_SRC, str) and CAM_SRC.isdigit():
    CAM_SRC = int(CAM_SRC)

FRAME_W = int(os.getenv("CAMERA_WIDTH", "1280"))
FRAME_H = int(os.getenv("CAMERA_HEIGHT", "720"))

WEB_HOST = os.getenv("WEB_HOST", "0.0.0.0")
WEB_PORT = int(os.getenv("WEB_PORT", "8085"))
LOG_LEVEL = os.getenv("LOG_LEVEL", "info").lower()

HOLD_DURATION = float(os.getenv("HOLD_SECONDS", "3.0"))
SAFE_DISTANCE = float(os.getenv("SAFE_DISTANCE", "1.2"))

BOX_CONF_THRESHOLD = float(os.getenv("BOX_CONF", "0.35"))
KPT_CONF_THRESHOLD = float(os.getenv("KPT_CONF", "0.30"))
HAND_ABOVE_HEAD_MARGIN = float(os.getenv("HAND_ABOVE_HEAD_MARGIN", "0.15"))
HAND_LOST_GRACE = float(os.getenv("HAND_LOST_GRACE", "0.4"))

state = {
    "status": "starting",
    "processing_fps": 0.0,
    "inference_ms": 0.0,
    "error": "Нет",
    "hand_detected": False,
    "hold_progress": 0.0,
    "target_locked": False,
    "mission_completed": False,
    "target_angle": 0.0,
    "target_distance_est": 0.0
}

latest_frame = None
frame_lock = threading.Lock()

hand_raise_start = None
last_hand_seen = 0.0
locked_center = None
last_seen_time = 0.0
LOST_TIMEOUT = 2.5

rclpy.init()
_ros_node = rclpy.create_node("vision_commander")
_ros_pub = _ros_node.create_publisher(String, "/robot_command", 10)

def send_robot_command(cmd: dict):
    msg = String()
    msg.data = json.dumps(cmd)
    _ros_pub.publish(msg)
    print(f"[ROBOT CMD] {cmd}")

def is_hand_raised_above_head(kpts, kconf, side="left"):
    if side == "left":
        sh_i, el_i, wr_i = 5, 7, 9
    else:
        sh_i, el_i, wr_i = 6, 8, 10

    if len(kpts) <= wr_i or len(kconf) <= wr_i:
        return False, None

    if kconf[sh_i] < KPT_CONF_THRESHOLD or kconf[wr_i] < KPT_CONF_THRESHOLD:
        return False, None

    l_sh = kpts[sh_i]
    l_wr = kpts[wr_i]

    if l_sh[0] <= 1 or l_sh[1] <= 1 or l_wr[0] <= 1 or l_wr[1] <= 1:
        return False, None

    nose_ok = len(kpts) > 0 and len(kconf) > 0 and kconf[0] >= KPT_CONF_THRESHOLD and kpts[0][0] > 1 and kpts[0][1] > 1
    if nose_ok:
        head_ref_y = kpts[0][1]
    else:
        head_ref_y = l_sh[1]

    body_height = max(abs(l_sh[1] - head_ref_y), 1.0)
    margin = body_height * HAND_ABOVE_HEAD_MARGIN

    is_above_head = l_wr[1] < (head_ref_y - margin)
    is_above_shoulder = l_wr[1] < l_sh[1]

    if not (is_above_head and is_above_shoulder):
        return False, None

    points = (
        (int(l_sh[0]), int(l_sh[1])),
        (int(l_wr[0]), int(l_wr[1])),
        (int(kpts[0][0]), int(kpts[0][1])) if nose_ok else None
    )

    return True, points

def estimate_distance(y1, y2, frame_h, kpts):
    box_h_ratio = max((y2 - y1) / float(frame_h), 0.05)

    lower_body_visible = False
    if len(kpts) >= 15:
        if kpts[13][1] > 0 or kpts[14][1] > 0:
            lower_body_visible = True

    scale_factor = 1.05 if lower_body_visible else 0.65
    dist = scale_factor / box_h_ratio
    return round(max(0.3, dist), 2)

def vision_worker():
    global latest_frame, hand_raise_start, last_hand_seen, locked_center, last_seen_time

    if isinstance(CAM_SRC, str) and CAM_SRC.startswith("rtsp://"):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"

    cap = cv2.VideoCapture(CAM_SRC)
    
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
    cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        state["status"] = "error"
        state["error"] = f"Не удалось открыть камеру ({CAM_SRC})"
        return

    state["status"] = "running"
    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            state["error"] = "Потерян кадр с камеры"
            time.sleep(0.1)
            continue

        h, w, _ = frame.shape
        center_x = w / 2.0
        fov_deg = 70.0
        now = time.time()

        t_inf_start = time.time()
        results = model(frame, verbose=False, imgsz=320)
        t_inf_end = time.time()

        people = []
        for r in results:
            if r.boxes is None or r.keypoints is None:
                continue

            for i in range(len(r.boxes)):
                if float(r.boxes.conf[i]) < BOX_CONF_THRESHOLD:
                    continue

                x1, y1, x2, y2 = map(int, r.boxes.xyxy[i])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                angle = ((cx - center_x) / center_x) * (fov_deg / 2.0)

                kpts = r.keypoints.xy[i].cpu().numpy()
                kconf = r.keypoints.conf[i].cpu().numpy()

                dist_est = estimate_distance(y1, y2, h, kpts)

                up_l, pts_l = is_hand_raised_above_head(kpts, kconf, "left")
                up_r, pts_r = is_hand_raised_above_head(kpts, kconf, "right")

                hand_up = up_l or up_r
                arm_pts = pts_l if up_l else pts_r

                people.append({
                    "box": (x1, y1, x2, y2),
                    "center": (cx, cy),
                    "angle": angle,
                    "dist": dist_est,
                    "hand_up": hand_up,
                    "arm_pts": arm_pts
                })

        if state["target_locked"]:
            matched_person = None
            min_pixel_dist = w * 0.35

            for p in people:
                px, py = p["center"]
                d_px = math.hypot(px - locked_center[0], py - locked_center[1])
                if d_px < min_pixel_dist:
                    min_pixel_dist = d_px
                    matched_person = p

            if matched_person is not None:
                locked_center = matched_person["center"]
                last_seen_time = now
                angle = matched_person["angle"]
                dist_est = matched_person["dist"]
                bx1, by1, bx2, by2 = matched_person["box"]

                state["target_angle"] = round(angle, 2)
                state["target_distance_est"] = dist_est
                state["hand_detected"] = matched_person["hand_up"]

                if dist_est <= SAFE_DISTANCE:
                    state["target_locked"] = False
                    state["mission_completed"] = True
                    locked_center = None
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 4)
                    cv2.putText(frame, f"ARRIVED", (bx1, max(by1 - 12, 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    send_robot_command({"cmd": "stop"})
                else:
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 0, 255), 3)
                    cv2.circle(frame, (int(locked_center[0]), int(locked_center[1])), 10, (0, 0, 255), 2)
                    cv2.putText(frame, f"LOCKED: {dist_est}m | angle: {angle:+.1f}", (bx1, max(by1 - 12, 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                    send_robot_command({
                        "cmd": "follow",
                        "angle_deg": round(angle, 2),
                        "distance_m": dist_est,
                        "stop_distance_m": SAFE_DISTANCE,
                    })
            else:
                if now - last_seen_time > LOST_TIMEOUT:
                    state["target_locked"] = False
                    locked_center = None
                    state["target_angle"] = 0.0
                    state["target_distance_est"] = 0.0
                    send_robot_command({"cmd": "stop"})
        else:
            hand_candidates = [p for p in people if p["hand_up"]]
            if hand_candidates:
                best = min(hand_candidates, key=lambda x: x["dist"])
                state["hand_detected"] = True
                state["mission_completed"] = False
                state["target_angle"] = round(best["angle"], 2)
                state["target_distance_est"] = best["dist"]

                last_hand_seen = now
                if hand_raise_start is None:
                    hand_raise_start = now

                elapsed = now - hand_raise_start
                state["hold_progress"] = min(round(elapsed, 2), HOLD_DURATION)

                bx1, by1, bx2, by2 = best["box"]

                if best["arm_pts"]:
                    p_sh, p_wr, p_head = best["arm_pts"]
                    cv2.line(frame, p_sh, p_wr, (0, 255, 255), 3)
                    cv2.circle(frame, p_sh, 6, (0, 0, 255), -1)
                    cv2.circle(frame, p_wr, 10, (0, 255, 0), -1)

                if elapsed >= HOLD_DURATION:
                    state["target_locked"] = True
                    locked_center = best["center"]
                    last_seen_time = now
                    hand_raise_start = None
                    state["hold_progress"] = 0.0

                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 0, 255), 4)
                    cv2.putText(frame, "TARGET LOCKED!", (bx1, max(by1 - 12, 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                    send_robot_command({
                        "cmd": "go_to_target",
                        "angle_deg": round(best["angle"], 2),
                        "distance_m": best["dist"],
                        "stop_distance_m": SAFE_DISTANCE,
                    })
                else:
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 255), 2)
                    cv2.putText(frame, f"Hold: {elapsed:.1f}/{HOLD_DURATION:.1f}s", (bx1, max(by1 - 12, 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            else:
                if hand_raise_start is not None and (now - last_hand_seen) > HAND_LOST_GRACE:
                    hand_raise_start = None
                    state["hold_progress"] = 0.0

                state["hand_detected"] = False
                if hand_raise_start is None:
                    state["target_angle"] = 0.0
                    state["target_distance_est"] = 0.0

        t_end = time.time()
        state["inference_ms"] = round((t_inf_end - t_inf_start) * 1000, 2)
        state["processing_fps"] = round(1.0 / max(t_end - prev_time, 1e-5), 2)
        state["error"] = "Нет"
        prev_time = t_end

        with frame_lock:
            latest_frame = frame.copy()

def generate_mjpeg():
    while True:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.02)
                continue
            _, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.03)

@asynccontextmanager
async def lifespan(app: FastAPI):
    threading.Thread(target=vision_worker, daemon=True).start()
    threading.Thread(target=lambda: rclpy.spin(_ros_node), daemon=True).start()
    yield

app = FastAPI(lifespan=lifespan)
templates = Jinja2Templates(directory="templates")

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(request=request, name="index.html")

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/api/telemetry")
async def telemetry():
    return JSONResponse(state)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host=WEB_HOST, port=WEB_PORT, log_level=LOG_LEVEL)