"""Robot odometry pose normalization for Unity root driving."""

import math


def _normalize_quaternion_wxyz(quaternion):
    values = [float(v) for v in quaternion[:4]]
    norm = math.sqrt(sum(v * v for v in values))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid root quaternion")
    return [v / norm for v in values]


def _multiply_quaternion_wxyz(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _yaw_from_quaternion_wxyz(quaternion):
    """Return Unitree Z-up yaw from a normalized WXYZ quaternion."""
    w, x, y, z = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _unitree_quaternion_to_unity_transport(quaternion):
    """Pre-compensate Unity's existing quaternion coordinate conversion."""
    w, x, y, z = quaternion
    return [w, -y, -x, -z]


class RootPoseTransformer:
    """Convert absolute odom pose into a Unity-compatible anchored pose.

    ``unity_relative`` treats the first valid odom sample as the session origin.
    Horizontal and vertical odom changes remain observable, while the initial
    output height is anchored to the Unity model's standing pelvis height.
    Quaternion input and output both use ``[w, x, y, z]`` storage order. With
    ``unitree_to_unity`` enabled, output components pre-compensate Unity's
    existing ``LeftRightQuatCoordinateConvert`` implementation.
    """

    MODES = ("unity_relative", "absolute")

    AXIS_MAPPINGS = ("unitree_to_unity", "raw")
    HEADING_REFERENCES = ("initial", "odom_world")
    VERTICAL_MODES = ("filtered", "fixed", "relative")

    def __init__(
        self,
        mode="unity_relative",
        pelvis_height=0.793,
        axis_mapping="unitree_to_unity",
        heading_reference="initial",
        vertical_mode="filtered",
        vertical_deadband=0.01,
        vertical_filter_alpha=0.2,
    ):
        if mode not in self.MODES:
            raise ValueError(f"unsupported root pose mode: {mode}")
        if axis_mapping not in self.AXIS_MAPPINGS:
            raise ValueError(f"unsupported root axis mapping: {axis_mapping}")
        if heading_reference not in self.HEADING_REFERENCES:
            raise ValueError(f"unsupported root heading reference: {heading_reference}")
        if vertical_mode not in self.VERTICAL_MODES:
            raise ValueError(f"unsupported root vertical mode: {vertical_mode}")
        pelvis_height = float(pelvis_height)
        vertical_deadband = float(vertical_deadband)
        vertical_filter_alpha = float(vertical_filter_alpha)
        if not math.isfinite(pelvis_height):
            raise ValueError("pelvis_height must be finite")
        if vertical_deadband < 0.0 or not math.isfinite(vertical_deadband):
            raise ValueError("vertical_deadband must be finite and non-negative")
        if not 0.0 < vertical_filter_alpha <= 1.0:
            raise ValueError("vertical_filter_alpha must be in (0, 1]")

        self.mode = mode
        self.pelvis_height = pelvis_height
        self.axis_mapping = axis_mapping
        self.heading_reference = heading_reference
        self.vertical_mode = vertical_mode
        self.vertical_deadband = vertical_deadband
        self.vertical_filter_alpha = vertical_filter_alpha
        self._origin_position = None
        self._origin_rotation = None
        self._origin_yaw = None
        self._last_output_rotation = None
        self._filtered_delta_z = 0.0

    def reset(self):
        self._origin_position = None
        self._origin_rotation = None
        self._origin_yaw = None
        self._last_output_rotation = None
        self._filtered_delta_z = 0.0

    def transform(self, position, rotation_wxyz):
        position = [float(v) for v in position[:3]]
        rotation = _normalize_quaternion_wxyz(rotation_wxyz)
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError("invalid root position")

        if self.mode == "absolute":
            output_position = position
            output_rotation = rotation
        else:
            if self._origin_position is None:
                self._origin_position = list(position)
                self._origin_rotation = list(rotation)
                self._origin_yaw = _yaw_from_quaternion_wxyz(rotation)

            delta_x = position[0] - self._origin_position[0]
            delta_y = position[1] - self._origin_position[1]
            delta_z = position[2] - self._origin_position[2]

            if self.heading_reference == "initial":
                cos_yaw = math.cos(self._origin_yaw)
                sin_yaw = math.sin(self._origin_yaw)
                heading_x = cos_yaw * delta_x + sin_yaw * delta_y
                heading_y = -sin_yaw * delta_x + cos_yaw * delta_y
            else:
                heading_x = delta_x
                heading_y = delta_y

            if self.axis_mapping == "unitree_to_unity":
                # Unitree: +X forward, +Y left, +Z up. The existing Unity
                # DataTools position conversion consumes transport values as
                # right/forward/up, so send right=-Y and forward=+X.
                output_x = -heading_y
                output_y = heading_x
            else:
                output_x = heading_x
                output_y = heading_y

            output_z = self.pelvis_height
            if self.vertical_mode == "filtered":
                if abs(delta_z) <= self.vertical_deadband:
                    target_delta_z = 0.0
                else:
                    target_delta_z = delta_z
                self._filtered_delta_z += self.vertical_filter_alpha * (
                    target_delta_z - self._filtered_delta_z
                )
                output_z += self._filtered_delta_z
            elif self.vertical_mode == "relative":
                output_z += delta_z

            output_position = [output_x, output_y, output_z]

            origin_inverse = [
                self._origin_rotation[0],
                -self._origin_rotation[1],
                -self._origin_rotation[2],
                -self._origin_rotation[3],
            ]
            # Left delta: q_delta * q_initial = q_current. Unity applies the
            # received delta in the same order: targetQuat * pelvisInitQuat.
            output_rotation = _normalize_quaternion_wxyz(
                _multiply_quaternion_wxyz(rotation, origin_inverse)
            )
            if self.axis_mapping == "unitree_to_unity":
                output_rotation = _unitree_quaternion_to_unity_transport(
                    output_rotation
                )

        if (
            self._last_output_rotation is not None
            and sum(a * b for a, b in zip(output_rotation, self._last_output_rotation)) < 0.0
        ):
            output_rotation = [-v for v in output_rotation]
        self._last_output_rotation = list(output_rotation)

        return output_position, output_rotation
