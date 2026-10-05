import hashlib
import json
import os
import tempfile
import unittest

from purepursuit_mgeo.mission_regions import load_mission_regions
from purepursuit_mgeo.route_geometry import RoutePolyline


class MissionRegionsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.route_file = os.path.join(self.temp.name, "route.txt")
        with open(self.route_file, "w", encoding="utf-8") as stream:
            stream.write("0 0 0\n100 0 0\n")
        self.route_length = RoutePolyline.from_path_file(self.route_file).length
        self.config_file = os.path.join(self.temp.name, "regions.json")
        self.route_digest = hashlib.sha256(self._read_route()).hexdigest()
        self.config = {
            "route_sha256": self.route_digest,
            "highway": {
                "start_s_m": 10.0, "end_s_m": 30.0,
                "handoff_enabled": False, "handoff_s_m": None,
                "max_route_offset_m": 11.0,
            },
            "roundabout": {
                "enabled": False, "request_start_s_m": None,
                "yield_s_m": None, "entry_s_m": None,
                "conflict_s_m": None, "request_end_s_m": None,
                "conflict_xy_map": None, "conflict_radius_m": 3.0,
                "circulating_link_ids": [], "max_route_offset_m": 5.0,
            },
        }

    def tearDown(self):
        self.temp.cleanup()

    def _read_route(self):
        with open(self.route_file, "rb") as stream:
            return stream.read()

    def write_config(self):
        with open(self.config_file, "w", encoding="utf-8") as stream:
            json.dump(self.config, stream)

    def test_unconfigured_regions_load_and_handoff_can_be_enabled(self):
        self.write_config()
        config = load_mission_regions(self.config_file, self.route_file, self.route_length)
        self.assertFalse(config["roundabout"]["enabled"])
        self.config["highway"].update(handoff_enabled=True, handoff_s_m=35.0)
        self.write_config()
        config = load_mission_regions(self.config_file, self.route_file, self.route_length)
        self.assertEqual(config["highway"]["handoff_s_m"], 35.0)

    def test_enabled_roundabout_requires_ordered_entry_and_complete_map_data(self):
        self.config["roundabout"].update({
            "enabled": True, "request_start_s_m": 40.0, "yield_s_m": 48.0,
            "entry_s_m": 50.0, "conflict_s_m": 58.0, "request_end_s_m": 80.0,
            "conflict_xy_map": [58.0, 0.0], "circulating_link_ids": ["ring_1"],
        })
        self.write_config()
        loaded = load_mission_regions(self.config_file, self.route_file, self.route_length)
        self.assertEqual(loaded["roundabout"]["entry_s_m"], 50.0)

    def test_rejects_missing_or_misordered_entry(self):
        region = self.config["roundabout"]
        region.update({
            "enabled": True, "request_start_s_m": 20.0, "yield_s_m": 25.0,
            "entry_s_m": 24.0, "conflict_s_m": 40.0, "request_end_s_m": 80.0,
            "conflict_xy_map": [40.0, 0.0], "circulating_link_ids": ["ring_1"],
        })
        self.write_config()
        with self.assertRaises(ValueError):
            load_mission_regions(self.config_file, self.route_file, self.route_length)
        region["entry_s_m"] = None
        self.write_config()
        with self.assertRaises(ValueError):
            load_mission_regions(self.config_file, self.route_file, self.route_length)

    def test_rejects_roundabout_overlap_with_highway(self):
        self.config["roundabout"].update({
            "enabled": True, "request_start_s_m": 25.0, "yield_s_m": 27.0,
            "entry_s_m": 28.0, "conflict_s_m": 32.0, "request_end_s_m": 40.0,
            "conflict_xy_map": [32.0, 0.0], "circulating_link_ids": ["ring_1"],
        })
        self.write_config()
        with self.assertRaises(ValueError):
            load_mission_regions(self.config_file, self.route_file, self.route_length)

    def test_rejects_route_hash_mismatch_and_boolean_as_distance(self):
        self.config["route_sha256"] = "0" * 64
        self.write_config()
        with self.assertRaises(ValueError):
            load_mission_regions(self.config_file, self.route_file, self.route_length)
        self.config["route_sha256"] = self.route_digest
        self.config["highway"]["start_s_m"] = True
        self.write_config()
        with self.assertRaises(ValueError):
            load_mission_regions(self.config_file, self.route_file, self.route_length)


if __name__ == "__main__":
    unittest.main()
