"""The composed sensor/planner static transforms must form one tree."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


class SensorTFCompositionTests(unittest.TestCase):
    def test_la_reference_keeps_both_map_and_world_without_two_odom_parents(self):
        sensor = ET.parse(ROOT/'stacks/sim-x86/config/launch/airsim_sensor_pipeline.launch')
        la = ET.parse(ROOT/'ws/risk-aware-comparison/src/risk_aware_planning/la_planner/la_planner_bridge/launch/la_planner_airsim.launch')
        reference = sensor.find("node[@name='map_to_reference']").get('args')
        own = la.find("node[@name='world_to_odom_la']").get('args')
        for frame, extra in [('odom', []), ('world', [own])]:
            args = reference.replace('$(arg map_reference_frame)', frame)
            parents = {}
            for transform in [args, *extra]:
                parent, child = transform.split()[-2:]
                self.assertNotIn(child, parents, 'Multiple static parents for '+child)
                parents[child] = parent
            node = 'odom'
            seen = set()
            while node in parents:
                self.assertNotIn(node, seen, 'TF cycle')
                seen.add(node)
                node = parents[node]
            self.assertEqual(node, 'map')


if __name__ == '__main__':
    unittest.main()
