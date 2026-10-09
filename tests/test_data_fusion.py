"""多源数据融合 (3.25 模块三) + 融合加载器测试"""
import json
import os

import pytest

from src.data_fusion.obstacle_fusion import ObstacleDataFusion
from src.io.input_loader import load_obstacle_set_fused, SOURCE_PRIORITY


def _static(oid, x, y, otype="building", **kw):
    d = {
        "id": oid, "name": oid, "type": otype,
        "x": x, "y": y, "z": 0.0,
        "width": 20.0, "depth": 20.0, "height": 50.0,
    }
    d.update(kw)
    return d


def _dynamic(oid, traj=None, safety=20.0):
    return {
        "id": oid, "name": oid, "type": "uav", "safety_radius": safety,
        "trajectory": traj or [
            {"x": 500.0, "y": 500.0, "z": 150.0, "t": 0.0},
            {"x": 1500.0, "y": 500.0, "z": 150.0, "t": 600.0},
        ],
    }


class TestObstacleDataFusion:
    def test_register_source(self):
        f = ObstacleDataFusion()
        f.register_source("city_gis", "static_buildings", priority=7)
        assert f.sources["city_gis"]["priority"] == 7

    def test_fuse_single_source(self):
        f = ObstacleDataFusion()
        f.register_source("city_gis", "static_buildings", 7)
        result = f.fuse({"city_gis": [_static("b1", 100, 100)]}, {})
        assert len(result.static_obstacles) == 1
        assert result.static_obstacles[0].obstacle_id == "b1"

    def test_fuse_records_source(self):
        f = ObstacleDataFusion()
        f.register_source("city_gis", "static_buildings", 7)
        result = f.fuse({"city_gis": [_static("b1", 100, 100)]}, {})
        assert result.static_obstacles[0].source == "city_gis"

    def test_fuse_dedups_static_by_position_and_type(self):
        """同一位置（10m 栅格）+ 同一类型的重复记录应去重"""
        f = ObstacleDataFusion()
        f.register_source("a", "static_buildings", 7)
        f.register_source("b", "static_buildings", 5)
        result = f.fuse(
            {"a": [_static("b1", 100, 100)], "b": [_static("b2", 103, 97)]},
            {},
        )
        assert len(result.static_obstacles) == 1
        assert any("去重" in w for w in f.warnings)

    def test_fuse_keeps_different_types_at_same_position(self):
        f = ObstacleDataFusion()
        f.register_source("a", "static_buildings", 7)
        result = f.fuse(
            {"a": [_static("b1", 100, 100), _static("t1", 100, 100, otype="tower")]},
            {},
        )
        assert len(result.static_obstacles) == 2

    def test_fuse_dedups_dynamic_by_id(self):
        f = ObstacleDataFusion()
        f.register_source("a", "dynamic_uav", 8)
        f.register_source("b", "dynamic_uav", 6)
        result = f.fuse({}, {"a": [_dynamic("d1")], "b": [_dynamic("d1")]})
        assert len(result.dynamic_obstacles) == 1

    def test_fuse_higher_priority_wins_dedup(self):
        """优先级高的源应先占位，被去重掉的是低优先级源"""
        f = ObstacleDataFusion()
        f.register_source("low", "static_buildings", 2)
        f.register_source("high", "static_buildings", 9)
        result = f.fuse(
            {"low": [_static("low_b", 100, 100)], "high": [_static("high_b", 100, 100)]},
            {},
        )
        assert result.static_obstacles[0].obstacle_id == "high_b"

    def test_fuse_tolerates_missing_fields(self):
        """缺 x/y 的记录应记 warning 并跳过，不抛异常"""
        f = ObstacleDataFusion()
        f.register_source("a", "static_buildings", 7)
        bad = {"id": "bad", "name": "bad"}
        result = f.fuse({"a": [bad, _static("ok", 100, 100)]}, {})
        assert len(result.static_obstacles) == 1
        assert any("解析失败" in w for w in f.warnings)

    def test_fuse_tolerates_bad_dynamic(self):
        f = ObstacleDataFusion()
        f.register_source("a", "dynamic_uav", 8)
        bad = {"id": "d", "trajectory": [{"x": 1.0}]}  # 缺 y/z/t
        result = f.fuse({}, {"a": [bad, _dynamic("good")]})
        assert len(result.dynamic_obstacles) == 1
        assert any("解析失败" in w for w in f.warnings)

    def test_fuse_empty_inputs(self):
        f = ObstacleDataFusion()
        result = f.fuse({}, {})
        assert result.static_obstacles == []
        assert result.dynamic_obstacles == []

    def test_quality_report(self):
        f = ObstacleDataFusion()
        f.register_source("a", "static_buildings", 7)
        f.fuse({"a": [_static("b1", 100, 100)]}, {})
        rep = f.quality_report()
        assert rep["registered_sources"] == 1
        assert rep["warning_count"] == 0
        assert "a" in rep["source_details"]

    def test_source_counts_tracked(self):
        f = ObstacleDataFusion()
        f.register_source("a", "static_buildings", 7)
        f.fuse({"a": [_static("b1", 100, 100), _static("b2", 500, 500)]}, {})
        assert f.sources["a"]["static_count"] == 2


class TestFusedLoader:
    def test_loads_real_sample_data(self, input_dir):
        obs, rep = load_obstacle_set_fused(
            [os.path.join(input_dir, "static_obstacles.json")],
            [os.path.join(input_dir, "dynamic_obstacles.json")],
        )
        assert len(obs.static_obstacles) == 12
        assert len(obs.dynamic_obstacles) == 4
        assert rep["warning_count"] == 0
        assert rep["registered_sources"] >= 2

    def test_preserves_obstacle_ids(self, input_dir):
        obs, _ = load_obstacle_set_fused(
            [os.path.join(input_dir, "static_obstacles.json")], []
        )
        ids = {o.obstacle_id for o in obs.static_obstacles}
        with open(os.path.join(input_dir, "static_obstacles.json"), encoding="utf-8") as f:
            expected = {o["id"] for o in json.load(f)["obstacles"]}
        assert ids == expected

    def test_preserves_dynamic_trajectories(self, input_dir):
        obs, _ = load_obstacle_set_fused(
            [], [os.path.join(input_dir, "dynamic_obstacles.json")]
        )
        assert all(len(o.trajectory) >= 2 for o in obs.dynamic_obstacles)

    def test_missing_file_is_skipped(self, input_dir):
        obs, rep = load_obstacle_set_fused(
            [os.path.join(input_dir, "static_obstacles.json")],
            [os.path.join(input_dir, "does_not_exist.json")],
        )
        assert len(obs.static_obstacles) == 12
        assert rep["warning_count"] == 0

    def test_multiple_files_merged(self, tmp_path):
        a = tmp_path / "a.json"
        a.write_text(json.dumps({"obstacles": [_static("s1", 100, 100)]}), encoding="utf-8")
        b = tmp_path / "b.json"
        b.write_text(json.dumps({"obstacles": [_static("s2", 900, 900)]}), encoding="utf-8")
        obs, _ = load_obstacle_set_fused([str(a), str(b)], [])
        assert {o.obstacle_id for o in obs.static_obstacles} == {"s1", "s2"}

    def test_falls_back_to_filename_when_no_source_field(self, tmp_path):
        f = tmp_path / "city_model.json"
        rec = _static("s1", 100, 100)
        rec.pop("source", None)
        f.write_text(json.dumps({"obstacles": [rec]}), encoding="utf-8")
        obs, rep = load_obstacle_set_fused([str(f)], [])
        assert len(obs.static_obstacles) == 1
        assert "city_model" in rep["source_details"]

    def test_bare_list_format_supported(self, tmp_path):
        f = tmp_path / "bare.json"
        f.write_text(json.dumps([_static("s1", 100, 100)]), encoding="utf-8")
        obs, _ = load_obstacle_set_fused([str(f)], [])
        assert len(obs.static_obstacles) == 1

    def test_matches_legacy_loader_counts(self, input_dir):
        """融合加载与旧加载器应产出同样数量的障碍物（无重复可去时）"""
        from src.io.input_loader import load_obstacle_set
        legacy = load_obstacle_set(
            os.path.join(input_dir, "static_obstacles.json"),
            os.path.join(input_dir, "dynamic_obstacles.json"),
        )
        fused, _ = load_obstacle_set_fused(
            [os.path.join(input_dir, "static_obstacles.json")],
            [os.path.join(input_dir, "dynamic_obstacles.json")],
        )
        assert len(fused.static_obstacles) == len(legacy.static_obstacles)
        assert len(fused.dynamic_obstacles) == len(legacy.dynamic_obstacles)

    def test_source_priority_covers_known_sources(self):
        for src in ("power_grid", "telecom", "city_gis", "dem"):
            assert src in SOURCE_PRIORITY
        assert SOURCE_PRIORITY["power_grid"] > SOURCE_PRIORITY["city_gis"]