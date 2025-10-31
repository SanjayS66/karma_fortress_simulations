import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFixer(Node):
    def __init__(self):
        super().__init__('imu_fixer')
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 50)
        self.pub = self.create_publisher(Imu, '/imu_fixed', 50)
        # example starting covariances (tune these)
        self.orient_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.angvel_cov = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        self.linacc_cov = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        # set covariances if zeros (or always overwrite)
        if not any(msg.orientation_covariance):
            out.orientation_covariance = self.orient_cov
        else:
            out.orientation_covariance = list(msg.orientation_covariance)

        if not any(msg.angular_velocity_covariance):
            out.angular_velocity_covariance = self.angvel_cov
        else:
            out.angular_velocity_covariance = list(msg.angular_velocity_covariance)

        if not any(msg.linear_acceleration_covariance):
            out.linear_acceleration_covariance = self.linacc_cov
        else:
            out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()