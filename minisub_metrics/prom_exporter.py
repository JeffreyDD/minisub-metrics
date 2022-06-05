import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from prometheus_client import start_http_server, Gauge

from sensor_msgs.msg import BatteryState, Imu


class BatteryExporter():
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic

        self.subscription_battery = node.create_subscription(
            BatteryState,
            self.topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription_battery

    def listener_callback(self, msg):
        self.node.gauge_battery_voltage.labels(topic=self.topic).set(msg.voltage)
        self.node.gauge_battery_current.labels(topic=self.topic).set(msg.current)


class ImuExporter():
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic

        self.subscription_imu = node.create_subscription(
            Imu,
            self.topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription_imu

    def listener_callback(self, msg):
        # Orientation
        self.node.gauge_imu_orient.labels(topic=self.topic, axis='x').set(msg.orientation.x)
        self.node.gauge_imu_orient.labels(topic=self.topic, axis='y').set(msg.orientation.y)
        self.node.gauge_imu_orient.labels(topic=self.topic, axis='z').set(msg.orientation.z)
        self.node.gauge_imu_orient.labels(topic=self.topic, axis='w').set(msg.orientation.w)
        
        # Angular Velocity
        self.node.gauge_imu_ang_vel.labels(topic=self.topic, axis='x').set(msg.angular_velocity.x)
        self.node.gauge_imu_ang_vel.labels(topic=self.topic, axis='y').set(msg.angular_velocity.y)
        self.node.gauge_imu_ang_vel.labels(topic=self.topic, axis='z').set(msg.angular_velocity.z)

        # Linear Acceleration
        self.node.gauge_imu_lin_acc.labels(topic=self.topic, axis='x').set(msg.angular_velocity.x)
        self.node.gauge_imu_lin_acc.labels(topic=self.topic, axis='y').set(msg.angular_velocity.y)
        self.node.gauge_imu_lin_acc.labels(topic=self.topic, axis='z').set(msg.angular_velocity.z)


class MetricExporter(Node):
    def __init__(self):
        super().__init__('metric_exporter')

        # Common labels
        labels = ['topic']

        # Battery metrics
        self.gauge_battery_voltage = Gauge('battery_voltage', 'Battery Voltage', labels)
        self.gauge_battery_current = Gauge('battery_current', 'Battery Current', labels)

        battery_exporter_0 = BatteryExporter(self, "power_thruster_0")
        battery_exporter_1 = BatteryExporter(self, "power_thruster_1")

        # IMU metrics
        imu_labels = labels + ['axis']

        self.gauge_imu_orient =  Gauge('imu_orientation', 'IMU Orientation', imu_labels)
        self.gauge_imu_ang_vel =  Gauge('imu_angular_velocity', 'IMU Angular Velocity', imu_labels)
        self.gauge_imu_lin_acc =  Gauge('imu_linear_acceleration', 'IMU Linear Acceleration', imu_labels)

        imu_exporter = ImuExporter(self, "imu")

        # Run HTTP server for prom to scrape
        start_http_server(8000)


def main(args=None):
    rclpy.init(args=args)

    metric_exporter = MetricExporter()

    rclpy.spin(metric_exporter)
    
    metric_exporter.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
