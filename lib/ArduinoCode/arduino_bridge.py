#   rosserial went away with ros2, and couldn't get microRos to work,
#   so doing this with simple serial communication and a ROS2 bridge

import rclpy, rclpy.node
import serial, time
from topic_def import sensor_names, sensor_types, actuator_names, actuator_types
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray


class ArduinoBridge(rclpy.node.Node):

    def __init__(self):
        super().__init__('arduino_bridge')
        self.init_ros()
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        self.timer = self.create_timer(0.1, self.process_serial)

    def serial_write(self, msg):
        print("serial_write:", msg)
        self.ser.write(msg.encode())

    def led_cb(self, data):
        self.serial_write('led|%s\n' %data.data)

    def fan_cb(self, data):
        self.serial_write('fan|%s\n' %(1 if data.data else 0))

    def wpump_cb(self, data):
        self.serial_write('wpump|%s\n' %(1 if data.data else 0))

    def freq_cb(self, data):
        print("freq_cb:", "NYI")

    def init_ros (self):
        self.cbs = {"led": self.led_cb, "fan": self.fan_cb,
                    "wpump": self.wpump_cb, "freq": self.freq_cb}
        self.pubs = {}
        for name in actuator_names:
            if name == "camera": continue
            msg_name = name if name == "freq" else name+"_raw"
            self.pubs[name] = self.create_subscription(actuator_types[name],
                                                       msg_name,
                                                       self.cbs[name], 1)
        for name in sensor_names:
            msg_name = name+"_raw"
            self.pubs[name] = self.create_publisher(sensor_types[name],
                                                    msg_name, 1)

    def process_serial(self):
        if self.ser.in_waiting > 0:
            try:
                try:
                    line = self.ser.readline().decode(errors='ignore').strip()
                except Exception as inst:
                    print("ERROR: readline failed")
                    self.ser.reset_input_buffer()
                    return
                if line[0] == '#': return
                name, data = line.split('|')
                stype = sensor_types[name]
                conversion = (float if stype in [Float32, Float32MultiArray] else
                              int if stype in [Int32, Int32MultiArray, Bool] else
                              lambda x: x)
                data = [conversion(datum) for datum in data.split(' ')]
                if len(data) == 1: data = data[0]
                msg = stype(data=data)
                self.pubs[name].publish(msg)
                #self.get_logger().info(f'Published: {msg.data}')
            except Exception as inst:
                print("ERROR: bad input: %s (%s)" %(line, str(inst.args)))
                self.ser.reset_input_buffer()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
