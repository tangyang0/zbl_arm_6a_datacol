import io
import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acquisition import (
    AlignmentConfig, AlignmentError, EpisodeBuffer, align_episode, diagnostic_features,
    header_values, capture_provenance,
)
from lerobot_writer import (
    acquisition_contract, cleanup_empty_images, dataset_compatibility_errors,
    quat_to_6d, write_episode,
)


def make_buffer(camera=True, master_pose=True):
    buf = EpisodeBuffer({"fixture": True, "master_pose_source": {
        "topic": "/master_right/ee_pose", "message_type": "geometry_msgs/PoseStamped",
        "side": "right", "frame_id": "master_base"}})
    for i in range(21):
        receive = 10**12 + i * 50_000_000
        header = 1_700_000_000_000_000_123 + i * 50_000_000
        buf.append("joints", [0.01 * i] * 6 + [0.02 * i],
                   receive_ns=receive, header_ns=header, frame_id="base")
        buf.append("pose", [0.001 * i, 0.1, 0.2, 0, 0, 0, 1],
                   receive_ns=receive, header_ns=header, frame_id="base")
        if master_pose:
            buf.append("master_pose", [0.3 + 0.001 * i, 0.2, 0.4, 0, 0, 0, 1],
                       receive_ns=receive, header_ns=123_456_789 + i * 50_000_000,
                       frame_id="master_base")
        if camera:
            from PIL import Image
            image = np.arange(48 * 64 * 3, dtype=np.uint8).reshape(48, 64, 3)
            image = (image.astype(np.uint16) + i).astype(np.uint8)
            output = io.BytesIO()
            Image.fromarray(image).save(output, format="JPEG")
            buf.append("camera", output.getvalue(),
                       receive_ns=receive, header_ns=header, frame_id="wrist_optical")
    return buf


class AlignmentTests(unittest.TestCase):
    def test_provenance_preserves_actual_options_and_source_hash(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "collector.py").write_text("example source")
            result = capture_provenance(SimpleNamespace(gripper_max=1.7, calibration_id="fixture"), tmp)
            self.assertEqual(result["actual_arguments"]["gripper_max"], 1.7)
            self.assertIn("collector.py", result["source_sha256"])
            self.assertEqual(result["runtime"]["lerobot_adapter_version"], "0.4.4")
            self.assertIsNone(result["coordinate_transform_applied"])

    def test_callbacks_record_original_headers_and_never_reuse_stale_gripper(self):
        source = Path(__file__).resolve().parents[1] / "master_bridge.py"
        tree = ast.parse(source.read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MasterBridge")
        funcs = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in (
            "on_slave_joint_state", "on_end_pose", "on_image")]
        namespace = {"time": SimpleNamespace(monotonic_ns=lambda: 1234567890)}
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), namespace)
        def pick(msg, names):
            try:
                return [msg.position[msg.name.index(name)] for name in names]
            except ValueError:
                return None
        owner = SimpleNamespace(rec_buf=EpisodeBuffer(), slave_gripper=.7,
                                slave_joints=None, pick_joints=pick,
                                _rec_stream_lock=threading.RLock())
        header = SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=456), frame_id="base")
        msg = SimpleNamespace(name=[f"arm_joint{i}" for i in range(1, 7)],
                              position=[0.] * 6, header=header)
        namespace["on_slave_joint_state"](owner, msg)
        self.assertEqual(len(owner.rec_buf.state), 0)
        msg.name.insert(0, "arm_gripper")
        msg.position.insert(0, .85)
        namespace["on_slave_joint_state"](owner, msg)
        self.assertEqual(owner.rec_buf.state[0][-1], .85)
        self.assertEqual(owner.rec_buf.trace["joints"]["header_ns"], [123000000456])
        self.assertEqual(owner.rec_buf.trace["joints"]["receive_ns"], [1234567890])

    def test_stamp_preserves_integer_nanoseconds(self):
        msg = SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1_700_000_001, nanosec=123_456_789), frame_id="camera"))
        self.assertEqual(header_values(msg), (1_700_000_001_123_456_789, "camera"))

    def test_receive_alignment_and_next_action_trace(self):
        buf = make_buffer()
        axis, matches, diag, report = align_episode(buf, trim=0)
        self.assertEqual(len(axis), 21)
        np.testing.assert_array_equal(np.diff(axis), np.full(20, 50_000_000))
        np.testing.assert_array_equal(diag["acq.pose.source_index"], np.arange(20))
        np.testing.assert_array_equal(diag["acq.action_pose.source_index"], np.arange(1, 21))
        self.assertEqual(diag["acq.action_pose.header_time_ns"][-1], buf.trace["pose"]["header_ns"][-1])
        self.assertEqual(set(diag), set(diagnostic_features()))
        self.assertTrue(all(a.dtype == np.int64 for a in diag.values()))
        self.assertFalse(report["hardware_synchronization_established"])

    def test_coverage_is_intersection(self):
        buf = make_buffer()
        # Remove first pose sample; grid must start at pose coverage, not camera start.
        buf.pose.pop(0)
        buf.pose_ts.pop(0)
        for key in buf.trace["pose"]:
            buf.trace["pose"][key].pop(0)
        axis, _, _, _ = align_episode(buf, trim=0)
        self.assertEqual(axis[0], buf.trace["pose"]["receive_ns"][0])

    def test_missing_camera_rejected_by_default(self):
        with self.assertRaisesRegex(AlignmentError, "missing camera"):
            align_episode(make_buffer(camera=False), trim=0)
        _, _, diag, _ = align_episode(
            make_buffer(camera=False), trim=0, config=AlignmentConfig(require_camera=False))
        self.assertFalse(any(k.startswith("acq.camera") for k in diag))

    def test_gap_rejects_episode_instead_of_dropping_grid_rows(self):
        buf = make_buffer()
        for key in buf.trace["camera"]:
            del buf.trace["camera"][key][5:12]
        del buf.imgs[5:12]
        del buf.img_ts[5:12]
        with self.assertRaises(AlignmentError) as ctx:
            align_episode(buf, trim=0)
        self.assertGreater(len(ctx.exception.report["invalid_grid_indices"]), 0)

    def test_positive_repeated_header_rejected_even_on_receive_clock(self):
        buf = make_buffer()
        buf.trace["pose"]["header_ns"][5] = buf.trace["pose"]["header_ns"][4]
        with self.assertRaisesRegex(AlignmentError, "repeated"):
            align_episode(buf, trim=0)

    def test_missing_header_allowed_only_for_receive_mode(self):
        buf = make_buffer()
        buf.trace["pose"]["header_ns"] = [0] * len(buf.pose)
        align_episode(buf, trim=0)
        with self.assertRaisesRegex(AlignmentError, "valid increasing"):
            align_episode(buf, trim=0, config=AlignmentConfig(
                clock="header", header_clock_domain="verified_ros_clock"))

    def test_header_mode_requires_explicit_domain_and_coherent_offsets(self):
        with self.assertRaises(ValueError):
            AlignmentConfig(clock="header").validate()
        buf = make_buffer()
        axis, _, _, _ = align_episode(
            buf, trim=0, config=AlignmentConfig(clock="header", header_clock_domain="ros_common"))
        self.assertEqual(axis[0], buf.trace["pose"]["header_ns"][0])
        buf.trace["camera"]["header_ns"] = [x + 10**9 for x in buf.trace["camera"]["header_ns"]]
        with self.assertRaisesRegex(AlignmentError, "inconsistent"):
            align_episode(buf, trim=0, config=AlignmentConfig(
                clock="header", header_clock_domain="ros_common"))

    def test_changed_frame_id_and_invalid_parameters_rejected(self):
        buf = make_buffer()
        buf.trace["pose"]["frame_id"][5] = "different_base"
        with self.assertRaisesRegex(AlignmentError, "frame_id changed"):
            align_episode(buf, trim=0)
        for bad in (0, -1, float("nan"), 20.5):
            with self.assertRaises(AlignmentError):
                align_episode(make_buffer(), fps=bad, trim=0)

    def test_quaternion_checked_and_normalized(self):
        np.testing.assert_allclose(quat_to_6d(0, 0, 0, 2), [1, 0, 0, 0, 1, 0])
        with self.assertRaises(ValueError):
            quat_to_6d(0, 0, 0, 0)


class DirectoryTests(unittest.TestCase):
    def test_cleanup_preserves_symlinked_dataset_root(self):
        with tempfile.TemporaryDirectory() as tmp, tempfile.TemporaryDirectory() as external:
            camera = Path(external) / "images/observation.images.wrist"
            camera.mkdir(parents=True)
            alias = Path(tmp) / "linked_dataset"
            alias.symlink_to(external, target_is_directory=True)
            self.assertEqual(cleanup_empty_images(alias), [])
            self.assertTrue(alias.is_symlink())
            self.assertTrue(camera.is_dir())
            # A normal dataset under a possibly aliased /tmp ancestor is allowed.
            cleanup_empty_images(external)
            self.assertFalse((Path(external) / "images").exists())

    def test_cleanup_only_empty_directories(self):
        with tempfile.TemporaryDirectory() as tmp:
            camera = Path(tmp) / "images/observation.images.wrist"
            camera.mkdir(parents=True)
            cleanup_empty_images(tmp)
            self.assertFalse((Path(tmp) / "images").exists())
            camera.mkdir(parents=True)
            frame = camera / "frame.png"
            frame.write_bytes(b"unfinished encoding")
            cleanup_empty_images(tmp)
            self.assertTrue(frame.exists())

    def test_cleanup_preserves_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp, tempfile.TemporaryDirectory() as external:
            (Path(external) / "observation.images.wrist").mkdir()
            (Path(tmp) / "images").symlink_to(external, target_is_directory=True)
            cleanup_empty_images(tmp)
            self.assertTrue((Path(tmp) / "images").is_symlink())
            self.assertTrue(Path(external).is_dir())
            self.assertTrue((Path(external) / "observation.images.wrist").is_dir())

    def test_legacy_schema_rejected_before_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "meta/info.json"
            path.parent.mkdir()
            path.write_text(json.dumps({"fps": 20, "features": {
                "observation.state": {"dtype": "float32", "shape": [10]},
                "observation.joints": {"dtype": "float32", "shape": [7]},
                "action": {"dtype": "float32", "shape": [10]},
            }}))
            before = path.read_bytes()
            errors = dataset_compatibility_errors(tmp, 20, "ee", 1.7, AlignmentConfig())
            self.assertTrue(any("acq." in err for err in errors))
            self.assertEqual(path.read_bytes(), before)


class ReplayMetadataTests(unittest.TestCase):
    def test_replay_settings_exit_before_loading_data_or_importing_ros(self):
        import builtins
        import replay
        original_import = builtins.__import__
        def guarded_import(name, *args, **kwargs):
            if name == "rclpy" or name.startswith("rclpy."):
                raise AssertionError("ROS imported before replay preflight")
            return original_import(name, *args, **kwargs)
        for flag in ("--speed", "--vmax", "--align-vmax"):
            for value in ("0", "-1", "nan", "inf"):
                with self.subTest(flag=flag, value=value):
                    with mock.patch.object(sys, "argv", [
                        "replay.py", "unused_dataset", "--engage", flag, value
                    ]), mock.patch.object(replay, "load_episode") as load, \
                            mock.patch("builtins.__import__", side_effect=guarded_import):
                        with self.assertRaises(SystemExit):
                            replay.main()
                        load.assert_not_called()

    def test_replay_bad_data_exits_before_ros(self):
        import builtins
        import pandas as pd
        import replay
        good = [[-1., 1., -1., 0., 0., 0., .5]]
        bad_cases = [
            ([], 20), ([[0.] * 6], 20), ([[0.] * 8], 20),
            ([[-1., 1., -1., 0., float("nan"), 0., .5]], 20),
            ([[-1., 1., -1., 0., 0., float("inf"), .5]], 20),
            ([[-1., 1., -1., 0., 0., 0., -.01]], 20),
            ([[-1., 1., -1., 0., 0., 0., 1.01]], 20),
            ([[-1., -.01, -1., 0., 0., 0., .5]], 20),
            (good, 0), (good, float("nan")), (good, float("inf")),
        ]
        original_import = builtins.__import__
        def guarded_import(name, *args, **kwargs):
            if name == "rclpy" or name.startswith("rclpy."):
                raise AssertionError("ROS imported before validating replay data")
            return original_import(name, *args, **kwargs)
        for rows, fps in bad_cases:
            with self.subTest(rows=rows, fps=fps):
                frame = pd.DataFrame({"observation.joints": rows})
                with mock.patch.object(sys, "argv", [
                    "replay.py", "unused_dataset", "--engage", "--grip-max", "1.7"
                ]), mock.patch.object(replay, "load_episode", return_value=(0, frame, fps)), \
                        mock.patch.object(replay, "resolve_gripper_scale", return_value=1.7), \
                        mock.patch("builtins.__import__", side_effect=guarded_import):
                    with self.assertRaises(SystemExit):
                        replay.main()
        np.testing.assert_array_equal(
            replay.validate_replay_data(good, 20, 1, 3, .3), good)

    def test_replay_resolves_episode_from_metadata_not_filename(self):
        import pandas as pd
        from replay import load_episode
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "meta/episodes/chunk-000").mkdir(parents=True)
            (root / "data/chunk-003").mkdir(parents=True)
            (root / "meta/info.json").write_text(json.dumps({
                "total_episodes": 2, "fps": 20,
                "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"}))
            pd.DataFrame({
                "episode_index": [0, 1], "data/chunk_index": [3, 3], "data/file_index": [7, 7]
            }).to_parquet(root / "meta/episodes/chunk-000/file-000.parquet")
            pd.DataFrame({
                "episode_index": [0, 0, 1, 1], "frame_index": [0, 1, 0, 1],
                "observation.joints": [[0.] * 7, [1.] * 7, [2.] * 7, [3.] * 7],
            }).to_parquet(root / "data/chunk-003/file-007.parquet")
            ep, frame, fps = load_episode(root, 1)
            self.assertEqual(ep, 1)
            self.assertEqual(list(frame["episode_index"]), [1, 1])
            self.assertEqual(float(frame.iloc[0]["observation.joints"][0]), 2.)

    def test_replay_scale_requires_verified_legacy_parameter(self):
        from replay import resolve_gripper_scale
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(resolve_gripper_scale(tmp, 0))
            with self.assertRaises(SystemExit):
                resolve_gripper_scale(tmp, 0, require=True)
            self.assertEqual(resolve_gripper_scale(tmp, 0, explicit=1.7, require=True), 1.7)
            meta = Path(tmp) / "meta/acquisition/episode_000000.json"
            meta.parent.mkdir(parents=True)
            meta.write_text(json.dumps({"contract": {"gripper_max_rad": 1.7}}))
            self.assertEqual(resolve_gripper_scale(tmp, 0, require=True), 1.7)
            with self.assertRaises(SystemExit):
                resolve_gripper_scale(tmp, 0, explicit=1.0, require=True)


class WriterLifecycleTests(unittest.TestCase):
    @staticmethod
    def owner(writer, thread_module=threading):
        source = Path(__file__).resolve().parents[1] / "master_bridge.py"
        tree = ast.parse(source.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MasterBridge")
        funcs = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in (
            "_write_episode", "wait_rec_writes")]
        namespace = {
            "time": time, "threading": thread_module, "write_episode": writer,
            "AlignmentError": AlignmentError, "alignment_from_args": lambda args: None,
            "json": json,
        }
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), namespace)
        owner = SimpleNamespace(
            rec_dir="/unused", rec_failed=[], _rec_writes_pending=0,
            _rec_write_lock=threading.Lock(), _rec_write_condition=threading.Condition(),
            args=SimpleNamespace(task="fixture", action_mode="ee", fps=20, trim=.5,
                                 gripper_max=1.7, calibration_id="fixture"),
            log=lambda *a, **k: None, log_error=lambda *a, **k: None,
        )
        owner._write_episode = MethodType(namespace["_write_episode"], owner)
        owner.wait_rec_writes = MethodType(namespace["wait_rec_writes"], owner)
        return owner

    def test_wait_tracks_worker_before_it_enters_the_write_lock(self):
        deferred = []
        class DeferredThread:
            def __init__(self, target, daemon):
                self.target = target
            def start(self):
                deferred.append(self.target)  # OS has not scheduled worker code yet.
        owner = self.owner(lambda *a, **k: 20, SimpleNamespace(Thread=DeferredThread))
        owner._write_episode(SimpleNamespace(state_ts=[0., 1.]), 1)
        self.assertEqual(owner._rec_writes_pending, 1)
        self.assertFalse(owner.wait_rec_writes(timeout=.01))
        deferred.pop()()
        self.assertEqual(owner._rec_writes_pending, 0)
        self.assertTrue(owner.wait_rec_writes(timeout=.1))

    def test_wait_covers_all_queued_writers_with_one_total_timeout(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        calls, result = [], []
        def writer(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise RuntimeError("fixture was not released")
            calls.append(1)
            return 20
        owner = self.owner(writer)
        buf = SimpleNamespace(state_ts=[0., 1.])
        owner._write_episode(buf, 1)
        self.assertTrue(entered.wait(1))
        owner._write_episode(buf, 2)
        self.assertEqual(owner._rec_writes_pending, 2)
        started = time.monotonic()
        self.assertFalse(owner.wait_rec_writes(timeout=.03))
        self.assertLess(time.monotonic() - started, .5)
        def waiter():
            result.append(owner.wait_rec_writes(timeout=1))
            finished.set()
        wait_thread = threading.Thread(target=waiter)
        wait_thread.start()
        try:
            self.assertFalse(finished.wait(.02))
        finally:
            release.set()
        self.assertTrue(finished.wait(2))
        wait_thread.join()
        self.assertEqual(calls, [1, 1])
        self.assertEqual(result, [True])
        self.assertEqual(owner._rec_writes_pending, 0)

    def test_failed_write_finishes_pending_count_but_wait_returns_false(self):
        def writer(*args, **kwargs):
            raise ValueError("expected writer failure")
        owner = self.owner(writer)
        owner._write_episode(SimpleNamespace(state_ts=[0., 1.]), 1, sync=True)
        self.assertEqual(owner._rec_writes_pending, 0)
        self.assertEqual(len(owner.rec_failed), 1)
        self.assertFalse(owner.wait_rec_writes(timeout=.1))

    def test_thread_start_failure_does_not_leave_pending_count(self):
        class FailedThread:
            def __init__(self, **kwargs):
                pass
            def start(self):
                raise RuntimeError("cannot start worker")
        owner = self.owner(lambda *a, **k: 20, SimpleNamespace(Thread=FailedThread))
        with self.assertRaises(RuntimeError):
            owner._write_episode(SimpleNamespace(state_ts=[0., 1.]), 1)
        self.assertEqual(owner._rec_writes_pending, 0)
        self.assertEqual(len(owner.rec_failed), 1)
        self.assertFalse(owner.wait_rec_writes(timeout=.1))


@unittest.skipUnless(os.getenv("RUN_LEROBOT_INTEGRATION") == "1", "real LeRobot CPU integration is opt-in")
class LeRobotIntegrationTests(unittest.TestCase):
    def test_write_append_readback_stats_and_trace(self):
        import pyarrow.parquet as pq
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        import lerobot.datasets.lerobot_dataset as module
        original_stats = module.compute_episode_stats
        with tempfile.TemporaryDirectory() as tmp:
            n = write_episode(make_buffer(), tmp, "toy", trim=0)
            dsroot = Path(tmp) / "toy"
            self.assertEqual(n, 20)
            self.assertIs(module.compute_episode_stats, original_stats)
            self.assertFalse((dsroot / "images").exists())
            write_episode(make_buffer(), tmp, "toy", trim=0)
            ds = LeRobotDataset("toy", root=dsroot, video_backend="pyav")
            self.assertEqual(ds.meta.total_episodes, 2)
            self.assertEqual(len(ds), 40)
            self.assertEqual(tuple(ds[0]["observation.state"].shape), (10,))
            self.assertEqual(tuple(ds[0]["action"].shape), (10,))
            rows = []
            for path in (dsroot / "data").rglob("*.parquet"):
                rows.extend(pq.read_table(path).to_pylist())
            rows.sort(key=lambda r: r["index"])
            first = rows[0]
            self.assertEqual(first["acq.pose.header_time_ns"], 1_700_000_000_000_000_123)
            self.assertEqual(rows[19]["acq.action_pose.source_index"], 20)
            self.assertEqual(rows[19]["acq.action_pose.header_time_ns"],
                             1_700_000_000_000_000_123 + 20 * 50_000_000)
            np.testing.assert_array_equal(first["action"], rows[1]["observation.state"])
            self.assertTrue(np.asarray(ds.meta.stats["observation.images.wrist"]["std"]).min() > .05)
            self.assertFalse(any(key.startswith("acq.") for key in ds.meta.stats))
            meta = json.loads((dsroot / "meta/acquisition/episode_000000.json").read_text())
            self.assertEqual(meta["status"], "complete")
            self.assertEqual(meta["contract"]["gripper_max_rad"], 1.7)
            self.assertEqual(meta["alignment_report"]["streams"]["pose"]["frame_id"], "base")
            self.assertFalse(any((dsroot / "meta/acquisition").glob("*.pending.json")))
            errors = dataset_compatibility_errors(dsroot, 20, "ee", 2.0, AlignmentConfig())
            self.assertTrue(errors)
            ds.finalize()

    def test_failed_encoding_preserves_images_and_restores_stats_hook(self):
        import lerobot.datasets.lerobot_dataset as module
        original_stats = module.compute_episode_stats
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(module, "_encode_video_worker", side_effect=RuntimeError("intentional encoder error")):
                with self.assertRaisesRegex(RuntimeError, "intentional encoder"):
                    write_episode(make_buffer(), tmp, "failed", trim=0)
            dsroot = Path(tmp) / "failed"
            self.assertIs(module.compute_episode_stats, original_stats)
            self.assertTrue(list((dsroot / "images").rglob("*.png")))
            self.assertTrue(list((dsroot / "meta/acquisition").glob("*.pending.json")))
            self.assertTrue((dsroot / "meta/acquisition/episode_000000.streams.jsonl").is_file())
            self.assertFalse((dsroot / "meta/acquisition/episode_000000.json").exists())
            self.assertTrue(dataset_compatibility_errors(dsroot, 20, "ee", 1.7, AlignmentConfig()))


if __name__ == "__main__":
    unittest.main()
