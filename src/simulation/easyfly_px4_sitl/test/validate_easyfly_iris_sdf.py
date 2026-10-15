#!/usr/bin/env python3
"""Static regression checks for the EasyFly Iris SDF's rigid bodies and D435i contract."""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


SDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "easyfly_iris"
    / "easyfly_iris.sdf"
)
URDF_PATH = SDF_PATH.with_name("easyfly_iris.urdf")
SENSORS_YAML_PATH = (
    Path(__file__).resolve().parents[3]
    / "perception"
    / "p1_preprocess"
    / "easyfly_camera_calib"
    / "config"
    / "sensors.yaml"
)


def _number(parent, path):
    element = parent.find(path)
    if element is None or element.text is None:
        raise AssertionError(f"missing numeric SDF element: {path}")
    return float(element.text)


def _symmetric_eigenvalues(matrix):
    """Return eigenvalues of a 3x3 symmetric matrix using Jacobi rotations."""
    values = [row[:] for row in matrix]
    for _ in range(64):
        p, q = max(
            ((0, 1), (0, 2), (1, 2)),
            key=lambda pair: abs(values[pair[0]][pair[1]]),
        )
        off_diagonal = values[p][q]
        scale = max(abs(values[index][index]) for index in range(3))
        if abs(off_diagonal) <= max(scale * 1e-14, 1e-30):
            break

        tau = (values[q][q] - values[p][p]) / (2.0 * off_diagonal)
        if tau >= 0:
            tangent = 1.0 / (tau + math.sqrt(1.0 + tau * tau))
        else:
            tangent = -1.0 / (-tau + math.sqrt(1.0 + tau * tau))
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine

        pp = values[p][p]
        qq = values[q][q]
        values[p][p] = pp - tangent * off_diagonal
        values[q][q] = qq + tangent * off_diagonal
        values[p][q] = 0.0
        values[q][p] = 0.0

        for index in range(3):
            if index in (p, q):
                continue
            ip = values[index][p]
            iq = values[index][q]
            values[index][p] = cosine * ip - sine * iq
            values[p][index] = values[index][p]
            values[index][q] = sine * ip + cosine * iq
            values[q][index] = values[index][q]

    return [values[index][index] for index in range(3)]


def _inertia_matrix(inertia):
    ixx = _number(inertia, "ixx")
    iyy = _number(inertia, "iyy")
    izz = _number(inertia, "izz")
    ixy = _number(inertia, "ixy")
    ixz = _number(inertia, "ixz")
    iyz = _number(inertia, "iyz")
    return [
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ]


def _rpy_matrix(rpy):
    """Convert fixed-axis roll/pitch/yaw to a 3x3 rotation matrix (Rz Ry Rx)."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_product(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _rotate(rotation, vector):
    return [sum(rotation[row][k] * vector[k] for k in range(3)) for row in range(3)]


def _compose_translation(mount_translation, mount_rotation, optical_translation):
    rotated_offset = _rotate(mount_rotation, optical_translation)
    return [mount_translation[index] + rotated_offset[index] for index in range(3)]


def _assert_vector_close(test_case, actual, expected, *, delta=1e-9):
    test_case.assertEqual(len(actual), len(expected))
    for actual_value, expected_value in zip(actual, expected):
        test_case.assertAlmostEqual(actual_value, expected_value, delta=delta)


def _assert_rotation_close(test_case, actual, expected, *, delta=1e-9):
    for actual_row, expected_row in zip(actual, expected):
        _assert_vector_close(test_case, actual_row, expected_row, delta=delta)


class EasyflyIrisSdfTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(SDF_PATH).getroot()
        cls.model = cls.root.find("model")
        if cls.model is None:
            raise AssertionError("SDF must contain a model element")
        cls.urdf = ET.parse(URDF_PATH).getroot()
        cls.calibration = yaml.safe_load(SENSORS_YAML_PATH.read_text())
        if not isinstance(cls.calibration, dict):
            raise AssertionError("camera calibration YAML must contain a mapping")
        cls.camera_calibration = cls.calibration["sensors"]["camera"]

    def test_imu_angular_noise_uses_density_not_bandwidth_rms(self):
        imu = self.model.find("plugin[@name='rotors_gazebo_imu_plugin']")
        self.assertIsNotNone(imu)
        self.assertEqual(imu.get("filename"), "libgazebo_imu_plugin.so")
        self.assertEqual(imu.findtext("linkName"), "/imu_link")
        self.assertEqual(imu.findtext("imuTopic"), "/imu")
        densities = imu.findall("gyroscopeNoiseDensity")
        self.assertEqual(len(densities), 1, "explicit unique gyro density required")
        density = _number(imu, "gyroscopeNoiseDensity")
        self.assertTrue(math.isfinite(density))
        self.assertGreater(density, 0.0, "a noiseless IMU must not bypass this contract")
        # DS-000529: 0.005 deg/s/sqrt(Hz); 0.05 deg/s is RMS at 100 Hz, not density.
        self.assertAlmostEqual(density, math.radians(0.005), delta=1e-14)
        self.assertGreater(abs(density - math.radians(0.05)), 1e-4)
        # This repair changes no accelerometer/bias/random-walk defaults.
        self.assertEqual(
            {child.tag for child in imu},
            {"robotNamespace", "linkName", "imuTopic", "gyroscopeNoiseDensity"},
        )

    def test_all_explicit_link_inertias_are_physically_realizable(self):
        links = self.model.findall("link")
        self.assertTrue(links, "model must declare links")

        for link in links:
            inertial = link.find("inertial")
            if inertial is None:
                continue
            with self.subTest(link=link.get("name")):
                mass = _number(inertial, "mass")
                self.assertTrue(math.isfinite(mass))
                self.assertGreater(mass, 0.0)

                matrix = _inertia_matrix(inertial.find("inertia"))
                self.assertTrue(
                    all(math.isfinite(value) for row in matrix for value in row)
                )
                moments = _symmetric_eigenvalues(matrix)
                scale = max(abs(moment) for moment in moments)
                tolerance = max(scale * 1e-9, 1e-30)
                self.assertGreater(min(moments), tolerance)
                self.assertLessEqual(
                    max(moments), sum(moments) - max(moments) + tolerance,
                    f"principal moments violate the rigid-body triangle inequality: {moments}",
                )

    def test_camera_visual_is_pose_only_and_adds_no_rigid_body(self):
        base_link = self.model.find("link[@name='base_link']")
        self.assertIsNotNone(base_link)
        camera_link_names = {
            "camera_link",
            "camera_color_frame",
            "camera_depth_frame",
        }
        declared_links = {link.get("name") for link in self.model.findall("link")}
        self.assertTrue(camera_link_names.isdisjoint(declared_links))
        for joint in self.model.findall("joint"):
            self.assertNotIn(joint.findtext("child"), camera_link_names)
            self.assertNotIn(joint.findtext("parent"), camera_link_names)

        camera_visual = base_link.find("visual[@name='camera_visual']")
        self.assertIsNotNone(camera_visual)
        self.assertIsNone(camera_visual.find("inertial"))
        size_text = camera_visual.findtext("geometry/box/size")
        self.assertIsNotNone(size_text)
        self.assertEqual(tuple(float(value) for value in size_text.split()), (0.025, 0.09, 0.025))
        pose_values = [float(value) for value in camera_visual.findtext("pose").split()]
        self.assertEqual(len(pose_values), 6)
        mount = self.camera_calibration["mount"]
        _assert_vector_close(
            self,
            pose_values,
            [*mount["translation"], *mount["rotation_rpy"]],
        )

    def test_camera_calibration_composes_consistently_with_urdf(self):
        mount = self.camera_calibration["mount"]
        optical = self.camera_calibration["optical"]
        self.assertEqual(mount["parent"], "base_link")
        self.assertEqual(mount["child"], "camera_link")

        mount_rotation = _rpy_matrix(mount["rotation_rpy"])
        optical_rotation = _rpy_matrix(optical["rotation_rpy"])
        expected_optical_translation = _compose_translation(
            mount["translation"], mount_rotation, optical["translation"]
        )
        expected_optical_rotation = _matrix_product(mount_rotation, optical_rotation)

        mount_joint = self.urdf.find("joint[@name='camera_joint']")
        self.assertIsNotNone(mount_joint)
        self.assertEqual(mount_joint.find("parent").get("link"), mount["parent"])
        self.assertEqual(mount_joint.find("child").get("link"), mount["child"])
        mount_origin = mount_joint.find("origin")
        _assert_vector_close(
            self,
            [float(value) for value in mount_origin.get("xyz").split()],
            mount["translation"],
        )
        _assert_rotation_close(
            self,
            _rpy_matrix([float(value) for value in mount_origin.get("rpy").split()]),
            mount_rotation,
        )

        actual_optical_translations = []
        actual_optical_rotations = []
        for frame, joint_name in (
            (optical["color_frame"], "camera_color_optical_joint"),
            (optical["depth_frame"], "camera_depth_optical_joint"),
        ):
            joint = self.urdf.find(f"joint[@name='{joint_name}']")
            self.assertIsNotNone(joint)
            self.assertEqual(joint.find("parent").get("link"), mount["child"])
            self.assertEqual(joint.find("child").get("link"), frame)
            origin = joint.find("origin")
            optical_translation = [float(value) for value in origin.get("xyz").split()]
            optical_rpy = [float(value) for value in origin.get("rpy").split()]
            _assert_vector_close(self, optical_translation, optical["translation"])
            _assert_rotation_close(
                self, _rpy_matrix(optical_rpy), optical_rotation
            )
            actual_optical_translations.append(
                _compose_translation(
                    mount["translation"], mount_rotation, optical_translation
                )
            )
            actual_optical_rotations.append(
                _matrix_product(mount_rotation, _rpy_matrix(optical_rpy))
            )

        for translation, rotation in zip(
            actual_optical_translations, actual_optical_rotations
        ):
            _assert_vector_close(self, translation, expected_optical_translation)
            _assert_rotation_close(self, rotation, expected_optical_rotation)

    def test_d435i_sensor_topics_and_optical_frames_are_preserved(self):
        base_link = self.model.find("link[@name='base_link']")
        self.assertIsNotNone(base_link)
        direct_sensors = base_link.findall("sensor")
        sensors = {sensor.get("name"): sensor for sensor in direct_sensors}
        self.assertEqual(set(sensors), {"color", "depth"})
        self.assertEqual(
            len(self.model.findall(".//sensor")), len(direct_sensors),
            "both sensors must be direct children of base_link",
        )

        optical = self.camera_calibration["optical"]
        mount = self.camera_calibration["mount"]
        mount_rotation = _rpy_matrix(mount["rotation_rpy"])
        expected_sensor_translation = _compose_translation(
            mount["translation"],
            mount_rotation,
            optical["translation"],
        )
        expected_sensor_rotation = mount_rotation
        contract = {
            "color": (
                "color_camera_plugin",
                "/camera/color/image_raw",
                optical["color_frame"],
            ),
            "depth": (
                "depth_camera_plugin",
                "/camera/aligned_depth_to_color/image_raw",
                optical["depth_frame"],
            ),
        }
        for sensor_name, (plugin_name, topic, frame) in contract.items():
            with self.subTest(sensor=sensor_name):
                sensor = sensors[sensor_name]
                self.assertEqual(sensor.get("type"), "camera" if sensor_name == "color" else "depth")
                self.assertEqual(_number(sensor, "update_rate"), 30.0)
                pose_element = sensor.find("pose")
                # SDF 1.6 in the deployed Gazebo parser ignores pose@frame.
                # The direct-child check above makes the owning frame base_link.
                self.assertIsNone(pose_element.get("frame"))
                pose = [float(value) for value in pose_element.text.split()]
                self.assertEqual(len(pose), 6)
                _assert_vector_close(self, pose[:3], expected_sensor_translation)
                # Gazebo's SDF camera sensor is expressed in the physical FLU frame;
                # the optical-frame rotation is represented by the calibration TF.
                _assert_rotation_close(
                    self,
                    _rpy_matrix(pose[3:]),
                    expected_sensor_rotation,
                )
                plugins = sensor.findall("plugin")
                self.assertEqual(len(plugins), 1)
                plugin = plugins[0]
                self.assertEqual(plugin.get("name"), plugin_name)
                self.assertEqual(plugin.get("filename"), "libeasyfly_gazebo_ros_camera.so")
                self.assertEqual(plugin.findtext("ros/namespace"), "/camera")
                camera_name = plugin.findtext("camera_name")
                self.assertIsNotNone(camera_name)
                self.assertEqual(f"/camera/{camera_name}/image_raw", topic)
                self.assertEqual(plugin.findtext("frame_name"), frame)


if __name__ == "__main__":
    unittest.main(verbosity=2)
