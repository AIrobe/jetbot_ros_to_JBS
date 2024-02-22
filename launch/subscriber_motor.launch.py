import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from azure.iot.device import IoTHubDeviceClient

class IoTHubNode(Node):
    def __init__(self):
        super().__init__('iothub_node')
        self.subscription = self.create_subscription(
            String,
            'motor_data',
            self.callback,
            10
        )
        self.IOTHUB_DEVICE_CONNECTION_STRING = "HostName=imv-aih-001.azure-devices.net;DeviceId=Jetbot_ROS;SharedAccessKey=j8etKYL5Jc5Ki83zD6t7QmuaQ9QyD0nlKAIoTHCOroQ="
        self.client = IoTHubDeviceClient.create_from_connection_string(self.IOTHUB_DEVICE_CONNECTION_STRING)

    def callback(self, msg):
        self.get_logger().info("Received message from motor_node: " + msg.data)
        self.get_logger().info("Sending message to IoT Hub...")
        self.client.send_message(msg.data)
        self.get_logger().info("Message successfully sent to IoT Hub")

def main(args=None):
    rclpy.init(args=args)
    node = IoTHubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()