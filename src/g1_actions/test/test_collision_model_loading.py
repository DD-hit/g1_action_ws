"""Model resource resolution and C++ fail-closed startup; no motion commands."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]


class ModelResourceTests(unittest.TestCase):
    def test_all_visual_and_collision_meshes_resolve_without_legacy_package(self):
        for path in sorted((ROOT / 'robot').glob('*.urdf')):
            with self.subTest(model=path.name):
                model = ET.parse(path).getroot()
                meshes = model.findall('.//mesh')
                self.assertEqual(len(meshes), 61)
                for mesh in meshes:
                    uri = mesh.attrib['filename']
                    self.assertTrue(uri.startswith('package://g1_actions/'), uri)
                    package, relative = uri[len('package://'):].split('/', 1)
                    self.assertTrue((Path(get_package_share_directory(package)) / relative).is_file(), uri)



@unittest.skipUnless(os.environ.get('ELE_TEST_DRIVER_INTEGRATION') == '1',
                     'Opt in to the isolated localhost fake driver')
class DriverModelLoadingTests(unittest.TestCase):
    def run_model(self, corrupt):
        with tempfile.TemporaryDirectory(prefix='g1_model_loading_') as directory:
            folder = Path(directory)
            cfg = yaml.safe_load((ROOT / 'config/elevator.yaml').read_text())
            pipeline = cfg['pipelines'][0]
            pipeline.update(fake_mode=True, fake_has_upper_body_authority=False)
            pipeline['unitree']['connect_on_init'] = False
            if corrupt:
                model = ET.parse(ROOT / 'robot/g1_29dof_mode_15.urdf')
                mesh = model.getroot().find('./link/collision/geometry/mesh')
                mesh.set('filename', 'package://g1_actions/robot/meshes/missing_regression_mesh.STL')
                model.write(folder / 'broken.urdf')
                pipeline['moveit']['robot_description_file'] = str(folder / 'broken.urdf')
            config = folder / 'driver.yaml'
            config.write_text(yaml.safe_dump(cfg))
            env = dict(os.environ, ROS_DOMAIN_ID='210', ROS_LOCALHOST_ONLY='1',
                       RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CONFIG_FILE=str(config),
                       UNITREE_SDK2_ROOT='/opt/unitree_sdk2', UNITREE_DDS_ROOT='/opt/unitree_sdk2')
            env['LD_LIBRARY_PATH'] = '/opt/unitree_sdk2/lib:' + env.get('LD_LIBRARY_PATH', '')
            with (folder / 'driver.log').open('w') as output:
                driver = subprocess.Popen(['bash', '/workspace/src/brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh', 'unitree_g1', 'fake'],
                                          env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 15
                    while time.monotonic() < deadline:
                        text = (folder / 'driver.log').read_text()
                        if corrupt and 'Collision geometry incomplete:' in text:
                            break
                        if not corrupt and '[CollisionGeometryLoaded]' in text:
                            break
                        if driver.poll() is not None:
                            break
                        time.sleep(.05)
                    text = (folder / 'driver.log').read_text()
                    if corrupt:
                        self.assertIn('Collision geometry incomplete:', text)
                        self.assertNotIn('[CollisionGeometryLoaded]', text)
                    else:
                        self.assertIn('[CollisionGeometryLoaded]', text)
                        self.assertNotIn('Error retrieving file', text)
                        self.assertIsNone(driver.poll())
                finally:
                    if driver.poll() is None:
                        os.killpg(driver.pid, signal.SIGINT)
                        try:
                            driver.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(driver.pid, signal.SIGKILL)
                            driver.wait(timeout=5)

    def test_valid_collision_geometry_loads(self):
        self.run_model(False)

    def test_missing_collision_mesh_prevents_successful_initialization(self):
        self.run_model(True)


if __name__ == '__main__':
    unittest.main()
