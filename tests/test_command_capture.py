"""Command/feedback distinction, causal alignment, and lossless episode evidence."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import MethodType, SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acquisition import (AlignmentConfig, EpisodeBuffer, align_episode, command_features,
                         master_pose_features, diagnostic_features, schema_errors, write_raw_streams)
from lerobot_writer import write_episode
from test_acquisition import make_buffer


def add_commands(buf):
    for i in range(105):
        buf.append_command("joints", [.8 + i * .0007] * 6,
                           10**12 - 20_000_000 + i * 10_000_000)
    # Holding an unchanged target does not require a fresh publication every frame.
    buf.append_command("gripper", [.85], 10**12 - 100_000_000, carried=True)
    buf.append_command("gripper", [.95], 10**12 + 55_000_000)
    return buf


class CommandAlignmentTests(unittest.TestCase):
    def test_causal_hold_uses_past_publication_not_nearest_future(self):
        buf = make_buffer(camera=False)
        buf.append_command("joints", [.8] * 6, 10**12 + 25_000_000)
        buf.append_command("joints", [.9] * 6, 10**12 + 75_000_000)
        buf.append_command("gripper", [.85], 10**12 - 100_000_000, carried=True)
        _, matches, diag, report = align_episode(
            buf, trim=0, config=AlignmentConfig(require_camera=False))
        np.testing.assert_array_equal(diag["acq.command_joints.valid"][:3], [0, 1, 1])
        np.testing.assert_array_equal(diag["acq.command_joints.source_index"][:3], [-1, 0, 1])
        np.testing.assert_allclose(matches["command_joints"]["values"][1], [.8] * 6)
        self.assertEqual(diag["acq.command_joints.publish_mono_ns"][0], 0)
        self.assertEqual(diag["acq.command_joints.age_ns"][0], -1)
        self.assertTrue(diag["acq.command_gripper.valid"].all())
        self.assertGreater(diag["acq.command_gripper.age_ns"][-1], 1_000_000_000)
        self.assertEqual(report["commands"]["command_gripper"]["carried_samples"], 1)

    def test_header_clock_still_aligns_commands_on_host_receive_clock(self):
        buf = add_commands(make_buffer(camera=False))
        _, matches, diag, _ = align_episode(buf, trim=0, config=AlignmentConfig(
            clock="header", header_clock_domain="ros_common", require_camera=False))
        self.assertGreater(diag["acq.grid_time_ns"][0], 10**18)
        self.assertEqual(diag["acq.command_joints.reference_mono_ns"][0], 10**12)
        self.assertEqual(diag["acq.command_joints.source_index"][0], 2)
        np.testing.assert_allclose(matches["command_joints"]["values"][0], [.8014] * 6)

    def test_missing_commands_are_explicitly_invalid(self):
        _, matches, diag, _ = align_episode(
            make_buffer(camera=False), trim=0, config=AlignmentConfig(require_camera=False))
        self.assertFalse(diag["acq.command_joints.valid"].any())
        self.assertFalse(diag["acq.command_gripper.valid"].any())
        np.testing.assert_array_equal(matches["command_joints"]["values"], np.zeros((20, 6)))
        self.assertEqual(set(diag), set(diagnostic_features(False)))

    def test_bad_command_shape_value_and_timestamp_are_rejected(self):
        for values, timestamp in (([0.] * 7, 1), ([float("nan")] * 6, 1),
                                  ([0.] * 6, 0), ([0.] * 6, True), ([0.] * 6, 1.5)):
            with self.subTest(values=values, timestamp=timestamp):
                with self.assertRaises(ValueError):
                    EpisodeBuffer().append_command("joints", values, timestamp)
        buf = EpisodeBuffer()
        buf.append_command("joints", [0.] * 6, 123)
        with self.assertRaises(ValueError):
            buf.append_command("joints", [0.] * 6, 123)

    def test_old_schema_requires_new_columns(self):
        features = {
            "observation.state": {"dtype": "float32", "shape": [10]},
            "observation.joints": {"dtype": "float32", "shape": [7]},
            "action": {"dtype": "float32", "shape": [10]},
            **diagnostic_features(False),
        }
        errors = schema_errors({"fps": 20, "features": features}, 20, require_camera=False)
        self.assertTrue(any("command.joints" in error for error in errors))
        features.update(command_features())
        features.update(master_pose_features())
        self.assertEqual(schema_errors({"fps": 20, "features": features}, 20,
                                       require_camera=False), [])


class PublisherTests(unittest.TestCase):
    def owner(self):
        source = Path(__file__).resolve().parents[1] / "master_bridge.py"
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MasterBridge")
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in (
            "publish_joints", "update_gripper", "_begin_recording", "rec_stop")]
        self.now = 1_000_000_000
        namespace = {"time": SimpleNamespace(monotonic_ns=lambda: self.now, time=lambda: 2.), "math": math,
                     "Float64MultiArray": SimpleNamespace, "EpisodeBuffer": EpisodeBuffer}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
        self.published = []
        pub = SimpleNamespace(publish=lambda msg: self.published.append(list(msg.data)))
        owner = SimpleNamespace(
            rec_buf=EpisodeBuffer(), _rec_stream_lock=threading.RLock(), joint_cmd_pub=pub,
            gripper_cmd_pub=pub, last_cmd=None, last_joint_publish_ns=None,
            grip_target=1.7, grip_last_cmd=.5, last_gripper_publish_ns=None, slave_gripper=.5,
            args=SimpleNamespace(gripper_vmax=12., rate=100., trim=0),
            rec_ep=1, rec_t0=0., rec_pending=None, log=lambda *a, **k: None)
        for node in methods:
            setattr(owner, node.name, MethodType(namespace[node.name], owner))
        return owner

    def test_records_published_values_and_carries_original_send_time(self):
        owner = self.owner()
        owner.publish_joints([.8] * 6)
        self.now += 1
        owner.update_gripper()
        self.assertEqual(owner.rec_buf.aj[0], self.published[0])
        self.assertEqual(owner.rec_buf.ag[0], self.published[1])
        self.assertAlmostEqual(owner.rec_buf.ag[0][0], .62)
        self.now += 1
        owner._begin_recording({})
        self.assertEqual(owner.rec_buf.command_trace["joints"]["publish_mono_ns"], [1_000_000_000])
        self.assertEqual(owner.rec_buf.command_trace["gripper"]["publish_mono_ns"], [1_000_000_001])
        self.assertEqual(owner.rec_buf.command_trace["gripper"]["carried_from_before_recording"], [True])
        self.assertEqual(len(self.published), 2)  # Carrying is not a new publication.

    def test_cached_feedback_is_not_fabricated_as_a_gripper_command(self):
        owner = self.owner()
        owner._begin_recording({})
        self.assertEqual(owner.rec_buf.ag, [])
        self.assertEqual(owner.rec_buf.aj, [])

    def test_failed_publication_does_not_record_a_command(self):
        owner = self.owner()
        def fail(msg):
            raise RuntimeError("publisher failed")
        owner.joint_cmd_pub.publish = fail
        with self.assertRaises(RuntimeError):
            owner.publish_joints([.8] * 6)
        self.assertEqual(owner.rec_buf.aj, [])
        self.assertIsNone(owner.last_joint_publish_ns)

    def test_stopped_episode_cannot_receive_later_commands(self):
        owner = self.owner()
        owner.publish_joints([.8] * 6)
        owner.rec_stop()
        saved = owner.rec_pending[0]
        self.now += 1
        owner.publish_joints([.9] * 6)
        self.assertEqual(saved.aj, [[.8] * 6])
        self.assertIsNone(owner.rec_buf)


class RawStreamTests(unittest.TestCase):
    def test_all_rate_samples_and_exact_timestamps_are_preserved(self):
        buf = add_commands(make_buffer(camera=False))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "streams.jsonl"
            summary = write_raw_streams(buf, path)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(summary["counts"], {
                "command.joints": 105, "command.gripper": 2, "feedback.joints": 21,
                "feedback.master_ee_pose": 21})
            self.assertEqual(summary["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
            commands = [r for r in rows if r["stream"] == "command.joints"]
            feedback = [r for r in rows if r["stream"] == "feedback.joints"]
            self.assertEqual(commands[-1]["values"], buf.aj[-1])
            self.assertEqual(feedback[0]["header_time_ns"], 1_700_000_000_000_000_123)
            self.assertEqual(feedback[-1]["values"], buf.state[-1])
            self.assertFalse(path.with_name(path.name + ".tmp").exists())
            with self.assertRaises(FileExistsError):
                write_raw_streams(buf, path)


@unittest.skipUnless(os.getenv("RUN_LEROBOT_INTEGRATION") == "1", "real LeRobot integration is opt-in")
class CommandWriterIntegrationTests(unittest.TestCase):
    def test_command_and_feedback_remain_distinct_after_write_append_and_readback(self):
        import pyarrow.parquet as pq
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        with tempfile.TemporaryDirectory() as tmp:
            for _ in range(2):
                buf = add_commands(make_buffer())
                buf.state[0][-1] = 2.2  # Preserve raw feedback even when fraction clips to 1.
                self.assertEqual(write_episode(buf, tmp, "commands", trim=0), 20)
            root = Path(tmp) / "commands"
            ds = LeRobotDataset("commands", root=root, video_backend="pyav")
            self.assertEqual(ds.meta.total_episodes, 2)
            rows = []
            for path in (root / "data").rglob("*.parquet"):
                rows.extend(pq.read_table(path).to_pylist())
            rows.sort(key=lambda row: row["index"])
            first = rows[0]
            np.testing.assert_allclose(first["command.joints"], [.8014] * 6)
            np.testing.assert_allclose(first["command.gripper"], [.85])
            np.testing.assert_allclose(first["observation.joints"][:6], [0.] * 6)
            np.testing.assert_allclose(first["observation.gripper_position"], [2.2])
            self.assertEqual(first["observation.joints"][6], 1.)
            np.testing.assert_array_equal(first["action"], rows[1]["observation.state"])
            self.assertFalse(any(k.startswith(("acq.", "command.")) for k in ds.meta.stats))
            self.assertEqual(ds.meta.features["command.joints"]["dtype"], "float64")
            for episode in range(2):
                meta = json.loads((root / f"meta/acquisition/episode_{episode:06d}.json").read_text())
                self.assertEqual(meta["contract"]["schema_version"], 3)
                raw = root / meta["raw_streams"]["path"]
                self.assertEqual(meta["raw_streams"]["counts"]["command.joints"], 105)
                self.assertEqual(meta["raw_streams"]["sha256"], hashlib.sha256(raw.read_bytes()).hexdigest())
            self.assertFalse((root / "images").exists())
            ds.finalize()


if __name__ == "__main__":
    unittest.main()
