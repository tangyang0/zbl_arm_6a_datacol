"""ROS-independent decoding and conservative discovery of master EE PoseStamped."""
import json
import math
import re
import time


POSE_TYPES = {"geometry_msgs/PoseStamped", "geometry_msgs/msg/PoseStamped"}
INT64_MAX = 2**63 - 1


def select_master_pose_topic(topics, types, side):
    if (not isinstance(topics, list) or not isinstance(types, list)
            or len(topics) != len(types) or side not in ("left", "right")
            or not all(isinstance(value, str) for value in topics + types)):
        raise ValueError("invalid rosapi topic list or master side")
    candidates = []
    for topic, message_type in zip(topics, types):
        words = set(re.split(r"[/_\-]+", str(topic).lower()))
        opposite = "left" if side == "right" else "right"
        if (message_type in POSE_TYPES and side in words and opposite not in words
                and words & {"master", "leader"}
                and not words & {"command", "commands", "cmd", "target", "goal", "desired", "setpoint"}):
            candidates.append(topic)
    if len(candidates) != 1:
        available = [t for t, kind in zip(topics, types) if kind in POSE_TYPES]
        raise ValueError(f"cannot uniquely identify {side} master EE; PoseStamped topics={available}; "
                         "set --master-ee-topic explicitly")
    return candidates[0]


def discover_master_pose_topic(ws, side, requested="auto", timeout=2.0):
    if requested != "auto":
        if not requested.startswith("/") or any(c.isspace() for c in requested):
            raise ValueError("--master-ee-topic must be an absolute ROS topic name or auto")
        return requested
    request_id = "capture-master-ee-topics"
    ws.send(json.dumps({"op": "call_service", "service": "/rosapi/topics", "id": request_id, "args": {}}))
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("rosapi master EE topic discovery timed out")
        packet = json.loads(ws.recv(timeout=remaining))
        if not isinstance(packet, dict):
            raise ValueError("invalid rosapi response")
        if packet.get("id") != request_id:
            continue
        if packet.get("op") != "service_response" or packet.get("result") is False:
            raise ValueError("rosapi/topics discovery failed; set --master-ee-topic explicitly")
        values = packet.get("values", {})
        if not isinstance(values, dict):
            raise ValueError("invalid rosapi topic response values")
        return select_master_pose_topic(values.get("topics", []), values.get("types", []), side)


def validate_master_pose_packet(packet, side):
    if not isinstance(packet, dict):
        raise ValueError("master EE packet must be an object")
    if packet.get("side") != side or packet.get("message_type") not in POSE_TYPES:
        raise ValueError("master EE side/message type mismatch")
    pose = packet.get("pose", [])
    if (not isinstance(pose, list) or len(pose) != 7
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in pose)
            or not math.isfinite(math.hypot(*pose[3:])) or math.hypot(*pose[3:]) < 1e-8):
        raise ValueError("master EE must contain finite xyz + nonzero quaternion xyzw")
    for key, minimum in (("receive_mono_ns", 1), ("header_time_ns", 0)):
        value = packet.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= INT64_MAX:
            raise ValueError(f"invalid master EE {key}")
    if not isinstance(packet.get("frame_id"), str) or not isinstance(packet.get("topic"), str):
        raise ValueError("master EE frame_id/topic must be strings")
    if not packet["topic"].startswith("/"):
        raise ValueError("master EE topic must be absolute")
    return {**packet, "pose": [float(v) for v in pose]}


def parse_master_pose_message(message, topic, receive_ns, side):
    """Decode only PoseStamped; never guess Euler units/order or derive follower FK."""
    try:
        p, q = message["pose"]["position"], message["pose"]["orientation"]
        header = message["header"]
        if not isinstance(header, dict):
            raise ValueError("master EE requires a PoseStamped header object")
        stamp = header.get("stamp", {})
        if not isinstance(stamp, dict):
            raise ValueError("master EE requires a ROS stamp object")
        sec = stamp.get("sec", stamp.get("secs", 0))
        nsec = stamp.get("nanosec", stamp.get("nsecs", 0))
        if (isinstance(sec, bool) or isinstance(nsec, bool) or not isinstance(sec, int)
                or not isinstance(nsec, int) or sec < 0 or not 0 <= nsec < 10**9):
            raise ValueError("invalid master EE ROS header stamp")
        packet = {"side": side, "topic": topic, "message_type": "geometry_msgs/PoseStamped",
                  "frame_id": header.get("frame_id", ""), "header_time_ns": sec * 10**9 + nsec,
                  "receive_mono_ns": receive_ns,
                  "pose": [p["x"], p["y"], p["z"], q["x"], q["y"], q["z"], q["w"]]}
    except (KeyError, TypeError) as exc:
        raise ValueError("master EE payload is not geometry_msgs/PoseStamped") from exc
    return validate_master_pose_packet(packet, side)


def master_pose_source(packet):
    return {key: packet[key] for key in ("topic", "message_type", "side", "frame_id")}
