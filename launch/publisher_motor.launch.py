import json
import rclpy
import atexit
from rclpy.node import Node
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT
import traitlets
from traitlets.config.configurable import Configurable

class Motor(Configurable):

    value = traitlets.Float()
    
    # config
    alpha = traitlets.Float(default_value=1.0).tag(config=True)
    beta = traitlets.Float(default_value=0.0).tag(config=True)

    def __init__(self, driver, channel, *args, **kwargs):
        super(Motor, self).__init__(*args, **kwargs)  # initializes traitlets

        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        if(channel == 1):
            self._ina = 1
            self._inb = 0
        else:
            self._ina = 2
            self._inb = 3
        atexit.register(self._release)
        
    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        """Sets motor value between [-1, 1]"""
        mapped_value = int(255.0 * (self.alpha * value + self.beta))
        speed = min(max(abs(mapped_value), 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina,0,0)
            self._driver._pwm.setPWM(self._inb,0,speed*16)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina,0,speed*16)
            self._driver._pwm.setPWM(self._inb,0,0)

    def _release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)
        # The two lines below are required for the Waveshare JetBot Board only
        self._driver._pwm.setPWM(self._ina,0,0)
        self._driver._pwm.setPWM(self._inb,0,0)

class MotorNode(Node):
    def __init__(self, motor):
        super().__init__('motor_node')
        self.publisher = self.create_publisher(String, 'motor_data', 10)
        self.motor = motor
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        data = {
            'value': self.motor.value,
            'alpha': self.motor.alpha,
            'beta': self.motor.beta,
        }
        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # モータードライバーを初期化します
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)
    atexit.register(motor_driver._pwm._device.__del__)

    # モーターを初期化します
    motor = Motor(motor_driver, channel=1)

    node = MotorNode(motor)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()