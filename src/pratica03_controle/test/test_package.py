import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class PackageTests(unittest.TestCase):
    def test_python_310_syntax(self):
        for path in ROOT.rglob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path), feature_version=(3, 10))

    def test_manifest_and_resource(self):
        root = ET.parse(ROOT/'package.xml').getroot()
        self.assertEqual(root.find('name').text, 'pratica03_controle')
        self.assertEqual(root.find('export/build_type').text, 'ament_python')
        self.assertTrue((ROOT/'resource/pratica03_controle').is_file())

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
