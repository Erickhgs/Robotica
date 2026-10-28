import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class PackageTests(unittest.TestCase):
    def test_failed_ekf_requests_shutdown_instead_of_continuing_without_pose(self):
        path = ROOT/'launch/pratica04.launch.py'
        spec = importlib.util.spec_from_file_location('pratica04_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        reason = '/odometry/filtered indisponivel'
        callback = module.after_exit([], reason)
        self.assertEqual(callback(SimpleNamespace(returncode=0), None), [])
        with patch.object(module, 'LogInfo'):
            actions = callback(SimpleNamespace(returncode=127), None)
        self.assertIsInstance(actions[-1], module.EmitEvent)
        self.assertIsInstance(actions[-1].event, module.Shutdown)
        self.assertEqual(actions[-1].event.reason, reason)

    def test_python_310_syntax(self):
        for path in ROOT.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path), feature_version=(3, 10))

    def test_manifest_and_resource(self):
        root = ET.parse(ROOT/'package.xml').getroot()
        self.assertEqual(root.find('name').text, 'robot_control')
        self.assertEqual(root.find('export/build_type').text, 'ament_python')
        self.assertTrue((ROOT/'resource/robot_control').is_file())

    def test_yaml_structure_and_exercise_values(self):
        for path in (ROOT/'config').glob('*.yaml'):
            data = yaml.safe_load(path.read_text(encoding='utf-8'))
            for node, content in data.items():
                self.assertIsInstance(content['ros__parameters'], dict, (path, node))
        mission = yaml.safe_load((ROOT/'config/mission.yaml').read_text())['mission_executor']['ros__parameters']
        points = mission['waypoints']
        self.assertGreaterEqual(len(points['x']), 3)
        self.assertEqual(len(points['x']), len(points['y']))
        self.assertEqual(len(points['x']), len(points['yaw']))
        trajectory = yaml.safe_load((ROOT/'config/trajectory.yaml').read_text())['figure_eight']['ros__parameters']
        self.assertEqual(trajectory['trajectory']['A'], 1.)
        self.assertEqual(trajectory['trajectory']['B'], 2.)
        self.assertEqual(trajectory['trajectory']['omega'], .5)


if __name__ == '__main__':
    unittest.main()
