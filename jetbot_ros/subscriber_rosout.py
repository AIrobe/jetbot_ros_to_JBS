from azure.iot.device import IoTHubDeviceClient, Message
from rclpy.node import Node
from rcl_interfaces.msg import Log

class IoTHubNode(Node):
    def __init__(self):
        super().__init__('iothub_node')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.callback,
            10
        )
        self.IOTHUB_DEVICE_CONNECTION_STRING = "HostName=imv-aih-001.azure-devices.net;DeviceId=Jetbot_ROS;SharedAccessKey=j8etKYL5Jc5Ki83zD6t7QmuaQ9QyD0nlKAIoTHCOroQ="
        self.client = IoTHubDeviceClient.create_from_connection_string(self.IOTHUB_DEVICE_CONNECTION_STRING)

    def callback(self, msg):
        self.get_logger().info("Received message from /rosout: " + msg.msg)
        self.get_logger().info("Sending message to IoT Hub...")
        message = Message(msg.msg)
        try:
            self.client.send_message(message)
            self.get_logger().info("Message successfully sent to IoT Hub")
        except Exception as e:
            self.get_logger().error("Failed to send message to IoT Hub: " + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = IoTHubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()