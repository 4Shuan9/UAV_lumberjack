import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker


class BranchPcaNode(Node):
    """
    Step13.4.1 + Step13.4.2 + Step13.4.3

    Step13.4.1:
        PCA -> branch principal direction d

    Step13.4.2:
        Project target points onto d
        -> axis coordinates s_i
        -> branch endpoints
        -> geometric center p0
        -> estimated length L

    Step13.4.3:
        Project target cloud to the plane normal to d
        -> fit a transverse circle
        -> corrected cylinder axis center
        -> estimated radius r

    Input:
        /perception/target_branch_cloud
        frame_id = base_link

    Outputs:
        /perception/branch_axis_marker
        /perception/branch_center_marker
        /perception/branch_length_marker

    cv_bridge is not used.
    """

    def __init__(self):
        super().__init__('branch_pca_node')

        self.input_topic = '/perception/target_branch_cloud'

        self.axis_marker_topic = '/perception/branch_axis_marker'
        self.center_marker_topic = '/perception/branch_center_marker'
        self.length_marker_topic = '/perception/branch_length_marker'
        self.cylinder_marker_topic = '/perception/branch_cylinder_marker'

        self.min_points = 10

        # Step13.4.1 visualization only.
        # This fixed arrow length is NOT the estimated branch length.
        self.axis_display_length = 0.40

        # Light temporal smoothing of PCA direction for RViz stability.
        self.direction_smoothing = 0.25
        self.previous_direction = None

        # Simulation ground truth used only for evaluation/logging.
        # It is NOT used by the estimation algorithm.
        self.simulation_gt_length = 0.30
        self.simulation_gt_radius = 0.038

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.axis_marker_pub = self.create_publisher(
            Marker,
            self.axis_marker_topic,
            10
        )

        self.center_marker_pub = self.create_publisher(
            Marker,
            self.center_marker_topic,
            10
        )

        self.length_marker_pub = self.create_publisher(
            Marker,
            self.length_marker_topic,
            10
        )

        self.cylinder_marker_pub = self.create_publisher(
            Marker,
            self.cylinder_marker_topic,
            10
        )

        self.frame_count = 0

        self.get_logger().info('====================================================')
        self.get_logger().info(' Step13.4.1 + Step13.4.2 Branch Geometry')
        self.get_logger().info(f' Input : {self.input_topic}')
        self.get_logger().info(
            f' Axis  : {self.axis_marker_topic}'
        )
        self.get_logger().info(
            f' Center: {self.center_marker_topic}'
        )
        self.get_logger().info(
            f' Length: {self.length_marker_topic}'
        )
        self.get_logger().info(
            f' Cylinder: {self.cylinder_marker_topic}'
        )
        self.get_logger().info(
            f' Simulation GT length = '
            f'{self.simulation_gt_length:.3f} m, '
            f'GT radius = {self.simulation_gt_radius:.3f} m '
            '(evaluation only)'
        )
        self.get_logger().info('====================================================')

    # ============================================================
    # PointCloud2 -> XYZ
    # ============================================================

    @staticmethod
    def pointcloud_xyz(msg):
        fields = {field.name: field for field in msg.fields}

        for name in ('x', 'y', 'z'):
            if name not in fields:
                raise ValueError(
                    f'PointCloud2 has no "{name}" field'
                )

            if fields[name].datatype != PointField.FLOAT32:
                raise ValueError(
                    f'PointCloud2 field "{name}" is not FLOAT32'
                )

        if msg.width == 0 or len(msg.data) == 0:
            return np.empty(
                (0, 3),
                dtype=np.float32
            )

        endian = '>' if msg.is_bigendian else '<'

        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [
                endian + 'f4',
                endian + 'f4',
                endian + 'f4'
            ],
            'offsets': [
                fields['x'].offset,
                fields['y'].offset,
                fields['z'].offset
            ],
            'itemsize': msg.point_step
        })

        cloud = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=memoryview(msg.data),
            strides=(msg.row_step, msg.point_step)
        )

        x = cloud['x'].reshape(-1)
        y = cloud['y'].reshape(-1)
        z = cloud['z'].reshape(-1)

        finite = (
            np.isfinite(x)
            & np.isfinite(y)
            & np.isfinite(z)
        )

        return np.column_stack(
            (x[finite], y[finite], z[finite])
        ).astype(
            np.float32,
            copy=False
        )

    # ============================================================
    # PCA
    # ============================================================

    def compute_pca(self, points):
        """
        Returns:
            mean_center : arithmetic mean of target points
            direction   : PCA principal direction d
            eigenvalues : lambda1 >= lambda2 >= lambda3
        """

        mean_center = np.mean(
            points,
            axis=0
        )

        centered = points - mean_center

        covariance = (
            centered.T @ centered
        ) / float(points.shape[0])

        eigenvalues, eigenvectors = np.linalg.eigh(
            covariance
        )

        order = np.argsort(
            eigenvalues
        )[::-1]

        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]

        direction = eigenvectors[:, 0].astype(
            np.float32,
            copy=False
        )

        norm = float(
            np.linalg.norm(direction)
        )

        if norm < 1.0e-9:
            raise ValueError(
                'PCA principal direction norm is too small'
            )

        direction = direction / norm

        # PCA eigenvectors have +/- sign ambiguity.
        if self.previous_direction is None:
            major_component = int(
                np.argmax(
                    np.abs(direction)
                )
            )

            if direction[major_component] < 0.0:
                direction = -direction

        else:
            if float(
                np.dot(
                    direction,
                    self.previous_direction
                )
            ) < 0.0:
                direction = -direction

            alpha = self.direction_smoothing

            smoothed = (
                (1.0 - alpha)
                * self.previous_direction
                + alpha
                * direction
            )

            smoothed_norm = float(
                np.linalg.norm(smoothed)
            )

            if smoothed_norm > 1.0e-9:
                direction = (
                    smoothed
                    / smoothed_norm
                ).astype(
                    np.float32
                )

        self.previous_direction = direction.copy()

        return (
            mean_center,
            direction,
            eigenvalues
        )

    # ============================================================
    # Step13.4.2 p0 + L
    # ============================================================

    @staticmethod
    def compute_axis_geometry(
        points,
        mean_center,
        direction
    ):
        """
        Project all target points onto the PCA axis.

        s_i = (p_i - mean_center)^T d

        Returns:
            p0       : midpoint of the estimated axial extent
            length   : s_max - s_min
            endpoint_min / endpoint_max
            s_min / s_max
        """

        centered = points - mean_center

        s = centered @ direction

        s_min = float(
            np.min(s)
        )

        s_max = float(
            np.max(s)
        )

        length = (
            s_max - s_min
        )

        axial_mid = 0.5 * (
            s_min + s_max
        )

        p0 = (
            mean_center
            + axial_mid * direction
        ).astype(
            np.float32
        )

        endpoint_min = (
            mean_center
            + s_min * direction
        ).astype(
            np.float32
        )

        endpoint_max = (
            mean_center
            + s_max * direction
        ).astype(
            np.float32
        )

        return (
            p0,
            float(length),
            endpoint_min,
            endpoint_max,
            s_min,
            s_max
        )

    # ============================================================
    # Step13.4.3 radius r by transverse-plane circle fitting
    # ============================================================

    @staticmethod
    def compute_radius_and_axis_center(
        points,
        p0,
        direction
    ):
        """
        Fit a circle to the target cloud in the plane perpendicular
        to the PCA axis.

        1) Build two orthonormal basis vectors e1/e2 perpendicular to d.
        2) Project all 3D points onto that transverse plane.
        3) Fit:
               x^2 + y^2 + A*x + B*y + C = 0
           by linear least squares.
        4) Recover the circle center and radius.

        Returns:
            corrected_p0 : p0 shifted onto the fitted cylinder axis
            radius       : fitted branch radius
            center_2d    : transverse circle center [cx, cy]
            residual_rms : radial fitting RMS error
        """

        d = direction.astype(np.float64)

        # Choose a helper vector that is not parallel to d.
        if abs(d[2]) < 0.90:
            helper = np.array(
                [0.0, 0.0, 1.0],
                dtype=np.float64
            )
        else:
            helper = np.array(
                [1.0, 0.0, 0.0],
                dtype=np.float64
            )

        e1 = np.cross(
            d,
            helper
        )

        e1_norm = float(
            np.linalg.norm(e1)
        )

        if e1_norm < 1.0e-9:
            raise ValueError(
                'Failed to build transverse basis e1'
            )

        e1 = e1 / e1_norm

        e2 = np.cross(
            d,
            e1
        )

        e2_norm = float(
            np.linalg.norm(e2)
        )

        if e2_norm < 1.0e-9:
            raise ValueError(
                'Failed to build transverse basis e2'
            )

        e2 = e2 / e2_norm

        rel = (
            points.astype(np.float64)
            - p0.astype(np.float64)
        )

        x = rel @ e1
        y = rel @ e2

        # Algebraic circle fit:
        # x^2 + y^2 + A*x + B*y + C = 0
        A_mat = np.column_stack(
            (x, y, np.ones_like(x))
        )

        b_vec = -(
            x*x + y*y
        )

        params, _, _, _ = np.linalg.lstsq(
            A_mat,
            b_vec,
            rcond=None
        )

        A_coef = float(params[0])
        B_coef = float(params[1])
        C_coef = float(params[2])

        cx = -0.5 * A_coef
        cy = -0.5 * B_coef

        radius_sq = (
            cx*cx
            + cy*cy
            - C_coef
        )

        if radius_sq <= 0.0:
            raise ValueError(
                f'Invalid fitted radius^2={radius_sq:.6e}'
            )

        radius = float(
            np.sqrt(radius_sq)
        )

        corrected_p0 = (
            p0.astype(np.float64)
            + cx * e1
            + cy * e2
        ).astype(
            np.float32
        )

        radial_dist = np.sqrt(
            (x - cx)**2
            + (y - cy)**2
        )

        residual_rms = float(
            np.sqrt(
                np.mean(
                    (radial_dist - radius)**2
                )
            )
        )

        center_2d = np.array(
            [cx, cy],
            dtype=np.float32
        )

        return (
            corrected_p0,
            radius,
            center_2d,
            residual_rms
        )

    # ============================================================
    # Quaternion helper: local +Z -> branch direction d
    # ============================================================

    @staticmethod
    def quaternion_from_z_axis(direction):
        d = direction.astype(np.float64)

        d_norm = float(
            np.linalg.norm(d)
        )

        if d_norm < 1.0e-9:
            raise ValueError(
                'Direction norm is too small'
            )

        d = d / d_norm

        z_axis = np.array(
            [0.0, 0.0, 1.0],
            dtype=np.float64
        )

        dot = float(
            np.clip(
                np.dot(z_axis, d),
                -1.0,
                1.0
            )
        )

        # Same direction.
        if dot > 1.0 - 1.0e-9:
            return (
                0.0,
                0.0,
                0.0,
                1.0
            )

        # Opposite direction: 180 deg around X.
        if dot < -1.0 + 1.0e-9:
            return (
                1.0,
                0.0,
                0.0,
                0.0
            )

        axis = np.cross(
            z_axis,
            d
        )

        axis_norm = float(
            np.linalg.norm(axis)
        )

        axis = axis / axis_norm

        angle = float(
            np.arccos(dot)
        )

        half = 0.5 * angle
        s = float(
            np.sin(half)
        )

        qx = float(axis[0] * s)
        qy = float(axis[1] * s)
        qz = float(axis[2] * s)
        qw = float(np.cos(half))

        return (
            qx,
            qy,
            qz,
            qw
        )

    # ============================================================
    # Marker helpers
    # ============================================================

    @staticmethod
    def xyz_to_point(xyz):
        point = Point()

        point.x = float(xyz[0])
        point.y = float(xyz[1])
        point.z = float(xyz[2])

        return point

    def publish_axis_marker(
        self,
        stamp,
        frame_id,
        center,
        direction
    ):
        marker = Marker()

        marker.header.stamp = stamp
        marker.header.frame_id = frame_id

        marker.ns = 'branch_pca'
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        half_length = (
            0.5
            * self.axis_display_length
        )

        start_xyz = (
            center
            - half_length * direction
        )

        end_xyz = (
            center
            + half_length * direction
        )

        marker.points = [
            self.xyz_to_point(start_xyz),
            self.xyz_to_point(end_xyz)
        ]

        marker.scale.x = 0.010
        marker.scale.y = 0.026
        marker.scale.z = 0.045

        # Green = PCA principal direction d
        marker.color.r = 0.10
        marker.color.g = 1.00
        marker.color.b = 0.10
        marker.color.a = 1.00

        self.axis_marker_pub.publish(
            marker
        )

    def publish_center_marker(
        self,
        stamp,
        frame_id,
        p0
    ):
        marker = Marker()

        marker.header.stamp = stamp
        marker.header.frame_id = frame_id

        marker.ns = 'branch_geometry'
        marker.id = 1

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(p0[0])
        marker.pose.position.y = float(p0[1])
        marker.pose.position.z = float(p0[2])

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.045

        # Blue = estimated center p0
        marker.color.r = 0.10
        marker.color.g = 0.35
        marker.color.b = 1.00
        marker.color.a = 1.00

        self.center_marker_pub.publish(
            marker
        )

    def publish_length_marker(
        self,
        stamp,
        frame_id,
        endpoint_min,
        endpoint_max
    ):
        marker = Marker()

        marker.header.stamp = stamp
        marker.header.frame_id = frame_id

        marker.ns = 'branch_geometry'
        marker.id = 2

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.points = [
            self.xyz_to_point(endpoint_min),
            self.xyz_to_point(endpoint_max)
        ]

        marker.scale.x = 0.018

        # Yellow = currently estimated axial length L
        marker.color.r = 1.00
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 1.00

        self.length_marker_pub.publish(
            marker
        )

    def publish_cylinder_marker(
        self,
        stamp,
        frame_id,
        p0,
        direction,
        length,
        radius
    ):
        marker = Marker()

        marker.header.stamp = stamp
        marker.header.frame_id = frame_id

        marker.ns = 'branch_geometry'
        marker.id = 3

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(p0[0])
        marker.pose.position.y = float(p0[1])
        marker.pose.position.z = float(p0[2])

        qx, qy, qz, qw = self.quaternion_from_z_axis(
            direction
        )

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        diameter = 2.0 * float(radius)

        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = float(length)

        # Semi-transparent cyan fitted cylinder.
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.00
        marker.color.a = 0.28

        self.cylinder_marker_pub.publish(
            marker
        )

    def delete_markers(
        self,
        stamp,
        frame_id
    ):
        for publisher, namespace, marker_id in (
            (
                self.axis_marker_pub,
                'branch_pca',
                0
            ),
            (
                self.center_marker_pub,
                'branch_geometry',
                1
            ),
            (
                self.length_marker_pub,
                'branch_geometry',
                2
            ),
            (
                self.cylinder_marker_pub,
                'branch_geometry',
                3
            ),
        ):
            marker = Marker()

            marker.header.stamp = stamp
            marker.header.frame_id = frame_id

            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE

            publisher.publish(
                marker
            )

    # ============================================================
    # Callback
    # ============================================================

    def cloud_callback(self, msg):
        try:
            points = self.pointcloud_xyz(
                msg
            )

        except Exception as exc:
            self.get_logger().error(
                f'PointCloud2 parse failed: {exc}'
            )
            return

        # LOST / REACQUIRING / TRACKING grace publish empty cloud.
        # PCA and geometry must not update from an invalid target.
        if points.shape[0] < self.min_points:
            self.delete_markers(
                msg.header.stamp,
                msg.header.frame_id
            )

            self.previous_direction = None

            self.frame_count += 1

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    '[GEOMETRY WAIT] '
                    f'points={points.shape[0]} < '
                    f'min_points={self.min_points}; '
                    'markers removed'
                )

            return

        try:
            (
                mean_center,
                direction,
                eigenvalues
            ) = self.compute_pca(
                points
            )

            (
                p0,
                length,
                endpoint_min,
                endpoint_max,
                s_min,
                s_max
            ) = self.compute_axis_geometry(
                points,
                mean_center,
                direction
            )

            (
                corrected_p0,
                radius,
                circle_center_2d,
                radius_rms
            ) = self.compute_radius_and_axis_center(
                points,
                p0,
                direction
            )

            # Shift endpoints together with the transverse axis correction.
            axis_shift = (
                corrected_p0 - p0
            )

            endpoint_min = (
                endpoint_min + axis_shift
            ).astype(
                np.float32
            )

            endpoint_max = (
                endpoint_max + axis_shift
            ).astype(
                np.float32
            )

            p0 = corrected_p0

        except Exception as exc:
            self.get_logger().error(
                f'Branch geometry failed: {exc}'
            )
            return

        # Step13.4.1 fixed-length green PCA direction marker.
        self.publish_axis_marker(
            msg.header.stamp,
            msg.header.frame_id,
            p0,
            direction
        )

        # Step13.4.2 blue center p0.
        self.publish_center_marker(
            msg.header.stamp,
            msg.header.frame_id,
            p0
        )

        # Step13.4.2 yellow segment whose length equals estimated L.
        self.publish_length_marker(
            msg.header.stamp,
            msg.header.frame_id,
            endpoint_min,
            endpoint_max
        )

        # Step13.4.3 fitted cylinder.
        self.publish_cylinder_marker(
            msg.header.stamp,
            msg.header.frame_id,
            p0,
            direction,
            length,
            radius
        )

        self.frame_count += 1

        if self.frame_count % 10 == 0:
            lambda1 = float(
                eigenvalues[0]
            )

            lambda2 = float(
                eigenvalues[1]
            )

            lambda3 = float(
                eigenvalues[2]
            )

            ratio12 = (
                lambda1 / lambda2
                if lambda2 > 1.0e-12
                else float('inf')
            )

            length_error = abs(
                length
                - self.simulation_gt_length
            )

            length_error_percent = (
                100.0
                * length_error
                / self.simulation_gt_length
            )

            self.get_logger().info(
                '[PCA] '
                f'n={points.shape[0]}, '
                f'd=['
                f'{direction[0]:.4f}, '
                f'{direction[1]:.4f}, '
                f'{direction[2]:.4f}], '
                f'lambda1/lambda2={ratio12:.2f}'
            )

            self.get_logger().info(
                '[GEOMETRY] '
                f'p0=['
                f'{p0[0]:.3f}, '
                f'{p0[1]:.3f}, '
                f'{p0[2]:.3f}], '
                f'L={length:.4f} m, '
                f's=[{s_min:.4f}, {s_max:.4f}] m, '
                f'GT={self.simulation_gt_length:.3f} m, '
                f'|e_L|={length_error:.4f} m '
                f'({length_error_percent:.2f}%)'
            )

            radius_error = abs(
                radius
                - self.simulation_gt_radius
            )

            radius_error_percent = (
                100.0
                * radius_error
                / self.simulation_gt_radius
            )

            self.get_logger().info(
                '[RADIUS] '
                f'r={radius:.4f} m, '
                f'GT={self.simulation_gt_radius:.3f} m, '
                f'|e_r|={radius_error:.4f} m '
                f'({radius_error_percent:.2f}%), '
                f'fit_rms={radius_rms:.4f} m, '
                f'circle_center_2d=['
                f'{circle_center_2d[0]:.4f}, '
                f'{circle_center_2d[1]:.4f}]'
            )


def main(args=None):
    rclpy.init(args=args)

    node = BranchPcaNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
