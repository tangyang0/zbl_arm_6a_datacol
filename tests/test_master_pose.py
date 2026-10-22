"""Master EE transport, source identity, timing and auxiliary-field persistence."""
import argparse
import ast
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acquisition import AlignmentConfig, AlignmentError, EpisodeBuffer, align_episode
from master_pose import (discover_master_pose_topic, master_pose_source, parse_master_pose_message,
                         select_master_pose_topic, validate_master_pose_packet)
from lerobot_writer import write_episode
from test_acquisition import make_buffer


def message():
    return {"header": {"stamp": {"sec": 1_700_000_000, "nanosec": 123_456_789}, "frame_id": "master_base"},
            "pose": {"position": {"x": .3, "y": .2, "z": .4},
                     "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}}


class DecoderTests(unittest.TestCase):
    def test_exact_ros1_ros2_stamps_and_quaternion_order(self):
        for stamp in ({"sec": 1_700_000_000, "nanosec": 123_456_789},
                      {"secs": 1_700_000_000, "nsecs": 123_456_789}):
            payload = message(); payload["header"]["stamp"] = stamp
            packet = parse_master_pose_message(payload, "/master_right/ee_pose", 10**12, "right")
            self.assertEqual(packet["header_time_ns"], 1_700_000_000_123_456_789)
            self.assertEqual(packet["pose"], [.3, .2, .4, 0., 0., 0., 1.])
            self.assertEqual(packet["frame_id"], "master_base")
            self.assertEqual(packet["receive_mono_ns"], 10**12)

    def test_invalid_pose_is_rejected_instead_of_guessing_an_euler_representation(self):
        for payload in ({"data": [0.] * 6}, {"pose": {}}, {"header": None, "pose": message()["pose"]}):
            with self.assertRaises(ValueError):
                parse_master_pose_message(payload, "/master_right/ee_pose", 123, "right")
        payload = message(); payload["pose"]["orientation"]["w"] = 0.
        with self.assertRaises(ValueError):
            parse_master_pose_message(payload, "/master_right/ee_pose", 123, "right")
        payload = message(); payload["pose"]["position"]["x"] = float("nan")
        with self.assertRaises(ValueError):
            parse_master_pose_message(payload, "/master_right/ee_pose", 123, "right")
        packet = parse_master_pose_message(message(), "/master_right/ee_pose", 123, "right")
        with self.assertRaises(ValueError):
            validate_master_pose_packet(packet, "left")

    def test_discovery_is_side_and_type_specific_and_refuses_ambiguity(self):
        topics = ["/master_left/ee_pose", "/master_right/ee_pose", "/arm/end_pose"]
        types = ["geometry_msgs/PoseStamped", "geometry_msgs/msg/PoseStamped", "geometry_msgs/PoseStamped"]
        self.assertEqual(select_master_pose_topic(topics, types, "right"), topics[1])
        for ts, kinds in ((["/arm/end_pose"], types[:1]),
                          ([topics[1], "/master_right/other_pose"], types[:2]),
                          (["/master_left_right/ee_pose"], types[:1]),
                          (["/master_right/target_pose"], types[:1]),
                          ([topics[1]], ["std_msgs/Float64MultiArray"])):
            with self.assertRaises(ValueError):
                select_master_pose_topic(ts, kinds, "right")

    def test_discovery_uses_only_rosapi_and_explicit_topic_needs_no_service(self):
        sent = []
        response = {"op": "service_response", "id": "capture-master-ee-topics", "result": True,
                    "values": {"topics": ["/master_right/ee_pose"], "types": ["geometry_msgs/PoseStamped"]}}
        ws = SimpleNamespace(send=lambda x: sent.append(json.loads(x)), recv=lambda **k: json.dumps(response))
        self.assertEqual(discover_master_pose_topic(ws, "right"), "/master_right/ee_pose")
        self.assertEqual(sent[0]["service"], "/rosapi/topics")
        sent.clear()
        self.assertEqual(discover_master_pose_topic(ws, "right", "/custom/ee_pose"), "/custom/ee_pose")
        self.assertFalse(sent)
        for values in (None, [], {"topics": ["/master_right/ee_pose"], "types": [{}]}):
            response["values"] = values
            with self.assertRaises(ValueError):
                discover_master_pose_topic(ws, "right")

    def test_receiver_main_forwards_master_pose_without_changing_joint_transport(self):
        source = Path(__file__).resolve().parents[1] / "master_rx.py"
        tree = ast.parse(source.read_text()); main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        received = iter([json.dumps({"op": "publish", "topic": "/master_right/ee_pose", "msg": message()}),
                         json.dumps({"op": "publish", "topic": "/joint_states", "msg": {
                             "name": [f"master_right_arm_joint{i}" for i in range(1, 7)], "position": [.2] * 6}})])
        sent, subscriptions = [], []
        class Websocket:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def send(self, data): subscriptions.append(json.loads(data))
            def recv(self, **kwargs):
                try: return next(received)
                except StopIteration: raise KeyboardInterrupt
        fake_socket = SimpleNamespace(AF_INET=socket.AF_INET, SOCK_DGRAM=socket.SOCK_DGRAM,
            socket=lambda *a: SimpleNamespace(sendto=lambda data, dest: sent.append(data)))
        namespace = {"argparse": argparse, "socket": fake_socket, "json": json, "math": math,
                     "time": SimpleNamespace(monotonic_ns=lambda: 10**12), "ws_connect": lambda *a, **k: Websocket(),
                     "discover_master_pose_topic": discover_master_pose_topic,
                     "parse_master_pose_message": parse_master_pose_message, "log_line": lambda *a: None}
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), "exec"), namespace)
        with mock.patch.object(sys, "argv", ["master_rx.py", "--master-ee-topic", "/master_right/ee_pose"]):
            namespace["main"]()
        self.assertEqual(json.loads(sent[0][3:])["pose"], [.3, .2, .4, 0., 0., 0., 1.])
        self.assertTrue(sent[1].startswith(b"MJ "))
        self.assertIn({"op": "subscribe", "topic": "/master_right/ee_pose", "queue_length": 1}, subscriptions)


class MasterBufferTests(unittest.TestCase):
    def test_missing_master_stream_is_rejected(self):
        with self.assertRaisesRegex(AlignmentError, "missing master EE"):
            align_episode(make_buffer(master_pose=False), trim=0)

    def test_pose_alignment_uses_receipt_clock_and_marks_stale_rows(self):
        buf = make_buffer(camera=False)
        # Keep only the first pose; it gets stale, rather than becoming an invented current pose.
        buf.master_pose = buf.master_pose[:1]; buf.master_pose_ts = buf.master_pose_ts[:1]
        buf.master_pose_carried = buf.master_pose_carried[:1]
        for key in buf.trace["master_pose"]: buf.trace["master_pose"][key] = buf.trace["master_pose"][key][:1]
        _, matches, diag, report = align_episode(buf, trim=0, config=AlignmentConfig(
            clock="header", header_clock_domain="follower_common", require_camera=False))
        np.testing.assert_array_equal(diag["acq.master_pose.valid"][:4], [1, 1, 1, 0])
        self.assertEqual(diag["acq.master_pose.reference_mono_ns"][0], 10**12)
        self.assertEqual(diag["acq.master_pose.header_time_ns"][0], 123_456_789)
        np.testing.assert_array_equal(matches["master_pose"]["values"][0], [.3, .2, .4, 0, 0, 0, 1])
        self.assertEqual(report["master_pose"]["frame_id"], "master_base")

    def test_frame_and_source_changes_are_rejected(self):
        for kind in ("frame", "topic"):
            buf = make_buffer()
            if kind == "frame": buf.trace["master_pose"]["frame_id"][4] = "another_base"
            else: buf.metadata["master_pose_source_changed"] = True
            with self.assertRaises(AlignmentError): align_episode(buf, trim=0)

    def test_bridge_buffers_pose_separately_and_carries_source_at_start(self):
        source = Path(__file__).resolve().parents[1] / "master_bridge.py"
        tree = ast.parse(source.read_text()); cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MasterBridge")
        funcs = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in (
            "drain_udp", "_accept_master_pose", "_begin_recording")]
        namespace = {"time": SimpleNamespace(monotonic_ns=lambda: 10**12 + 1, perf_counter=lambda: 1.),
                     "EpisodeBuffer": EpisodeBuffer, "json": json,
                     "validate_master_pose_packet": validate_master_pose_packet, "master_pose_source": master_pose_source}
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), namespace)
        owner = SimpleNamespace(master_ee_pose_packet=None, rec_buf=None, master_joints=[.9] * 6,
            _rec_stream_lock=threading.RLock(), last_joint_publish_ns=None, last_gripper_publish_ns=None,
            args=SimpleNamespace(side="right"), log_throttle=lambda *a: None)
        for func in funcs: setattr(owner, func.name, MethodType(namespace[func.name], owner))
        payload = message()
        payload["header"]["frame_id"] = "master_base_" + "x" * 3000
        packet = parse_master_pose_message(payload, "/master_right/ee_pose", 10**12, "right")
        datagrams = [b"MP " + json.dumps(packet).encode()]
        def recvfrom(size):
            if not datagrams: raise BlockingIOError
            return datagrams.pop(0)[:size], ("127.0.0.1", 9870)
        owner.udp = SimpleNamespace(recvfrom=recvfrom)
        owner.drain_udp(); owner._begin_recording({})
        self.assertEqual(owner.master_joints, [.9] * 6)
        self.assertEqual(owner.rec_buf.master_pose, [packet["pose"]])
        self.assertEqual(owner.rec_buf.master_pose_carried, [True])
        self.assertEqual(owner.rec_buf.metadata["master_pose_source"], master_pose_source(packet))
        owner._accept_master_pose({**packet, "receive_mono_ns": 10**12 + 2,
                                   "header_time_ns": packet["header_time_ns"] + 2})
        self.assertEqual(len(owner.rec_buf.master_pose), 2)

    def test_missing_or_stale_pose_blocks_recording_before_discarding_pending_data(self):
        source = Path(__file__).resolve().parents[1] / "master_bridge.py"
        tree = ast.parse(source.read_text()); cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MasterBridge")
        func = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "rec_toggle")
        namespace = {"time": SimpleNamespace(monotonic_ns=lambda: 10**12 + 1_000_000_000)}
        exec(compile(ast.Module(body=[func], type_ignores=[]), str(source), "exec"), namespace)
        pending = object()
        for packet in (None, {"receive_mono_ns": 10**12}):
            owner = SimpleNamespace(_recording=lambda: False, _home_busy=False, master_ee_pose_packet=packet,
                args=SimpleNamespace(sync_max_age_ms=100), rec_pending=pending, log=lambda *a: None)
            namespace["rec_toggle"](owner)
            self.assertIs(owner.rec_pending, pending)


@unittest.skipUnless(os.getenv("RUN_LEROBOT_INTEGRATION") == "1", "real LeRobot integration is opt-in")
class MasterWriterIntegrationTests(unittest.TestCase):
    def test_master_pose_and_raw_source_survive_save_append_and_readback(self):
        import pyarrow.parquet as pq
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        with tempfile.TemporaryDirectory() as tmp:
            for _ in range(2): self.assertEqual(write_episode(make_buffer(), tmp, "master", trim=0), 20)
            root = Path(tmp) / "master"
            ds = LeRobotDataset("master", root=root, video_backend="pyav")
            self.assertEqual(ds.meta.total_episodes, 2)
            rows = []
            for path in (root / "data").rglob("*.parquet"): rows.extend(pq.read_table(path).to_pylist())
            first = min(rows, key=lambda row: row["index"])
            np.testing.assert_array_equal(first["observation.master_ee_pose"], [.3, .2, .4, 0, 0, 0, 1])
            self.assertEqual(first["acq.master_pose.receive_mono_ns"], 10**12)
            self.assertEqual(first["acq.master_pose.header_time_ns"], 123_456_789)
            self.assertNotIn("observation.master_ee_pose", ds.meta.stats)
            for episode in range(2):
                meta = json.loads((root / f"meta/acquisition/episode_{episode:06d}.json").read_text())
                self.assertEqual(meta["contract"]["master_pose_source"]["topic"], "/master_right/ee_pose")
                raw = [json.loads(row) for row in (root / meta["raw_streams"]["path"]).read_text().splitlines()]
                poses = [row for row in raw if row["stream"] == "feedback.master_ee_pose"]
                self.assertEqual(len(poses), 21)
                self.assertEqual(poses[-1]["frame_id"], "master_base")
            self.assertFalse((root / "images").exists())
            ds.finalize()


if __name__ == "__main__":
    unittest.main()
