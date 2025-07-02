import rclpy
from rclpy.node import Node
import serial
import threading
import time
from std_msgs.msg import Header
from moa_msgs.msg import CANStamped
import re

# fsae must be granted to the dialout group permission for this node to run:
# if [Errno 13] Permission denied is printed use the following command to add
# the current user to the dialout group: 
# sudo usermod -a -G dialout $USERsudo usermod -a -G dialout $USER


def clean_can_frame(data):
    frames = re.sub(r'[^a-zA-Z0-9]', '', data)
    frames = frames.split('t')
    frames.remove('')
    return frames


def parse_can_frame(frame):
    """Parses a CAN frame and returns a tuple of ID, length, data.

    Args:
      frame: A CAN frame in hex format.

    Returns:
      A tuple of ID, length, data, where data is a list of bytes.
    """
    id = int(frame[:3], 16)
    length = int(frame[3:4], 16)
    data = [int(frame[4 + i:4 + i + 2], 16) if i + 4 < len(frame) else 0 for i in range(8)]

    return id, length, data


def get_baud_rate_command(rate: int):
    command = b'S5'

    if rate == 10:
        command = b'S0'
    elif rate == 20:
        command = b'S1'
    elif rate == 50:
        command = b'S2'
    elif rate == 100:
        command = b'S3'
    elif rate == 125:
        command = b'S4'
    elif rate == 250:
        command = b'S5'
    elif rate == 500:
        command = b'S6'
    elif rate == 800:
        command = b'S7'
    elif rate == 1000:
        command = b'S8'

    return command


class CanDapter(Node):
    def __init__(self, *vargs, **kwargs):
        super().__init__('candapter_node', *vargs, **kwargs)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baud_rate', 250),
            ]
        )

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.serial_port = serial.Serial(
            self.port,
        )

        self.can_subscription = self.create_subscription(
            CANStamped,
            'pub_raw_can',
            self.can_send_callback,
            10
        )

        self.can_publisher = self.create_publisher(
            CANStamped,
            'sub_raw_can',
            10
        )

        self.send_command_with_response(b'C')  # restart can dapter

        while not self.send_command_with_response(
                get_baud_rate_command(self.baud_rate)
        ):
            pass

        while not self.send_command_with_response(b'O'):  # start can dapter
            pass

        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.on_start()
    
    def on_start(self):
        self.get_logger().info("CanDapter Initialized")
        # encode_message
        
    def shutdown(self):
        # context is invalid when this function is called
        self.get_logger().info("CanDapter shutdown")
    
    def encode_message(self, can_msg):
        return (f'T{can_msg.id:03X}'
            f'{len(can_msg.data):01X}'
            f'{"".join(["%02X" % byte for byte in can_msg.data])}'
            .encode())

    def can_send_callback(self, can_stamped_msg):
        if can_stamped_msg.header.frame_id == 'candapter_node':
            return # Question: what is this doing/stopping the node from doing?
        
        command = self.encode_message(can_stamped_msg.can)
   
        self.send_command(command)
        
        self.get_logger().info(f'Sent cmd')

    def read_serial(self):
        while True:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read_until(b'\r') \
                    .decode() \
                    .strip()

                try:
                    index_of_t = data.index('t')
                    data = data[index_of_t:]
                except ValueError:
                    pass

                if data[0] == 't':
                    cleaned_data_list = clean_can_frame(data)
                    for cleaned_data in cleaned_data_list:
                        try:
                            frame = parse_can_frame(cleaned_data)
                            msg = self.can_frame_to_can_stamped(frame)
                            self.can_publisher.publish(msg)

                        except:
                            self.get_logger().error(f'Message failed to be received: {cleaned_data}')

    def send_command(self, command: bytes):
        if not self.serial_port.is_open:
            self.get_logger().error(f'Error Serial Port not open')
            return False

        msg = b'\015' + command + b'\015'
        self.serial_port.write(msg)
        self.get_logger().info(f'CAN command: {command}')
        return True

    def can_frame_to_can_stamped(self, frame):
        """Converts a CAN frame data to a ROS 2 Humble CANStamped message.

      Args:
        frame: A CAN frame data tuple in the format (id, length, data).

      Returns:
        A ROS 2 Humble CANStamped message.
      """

        can_stamped = CANStamped()
        can_stamped.header.frame_id = 'candapter_node'
        can_stamped.header.stamp = self.get_clock().now().to_msg()
        can_stamped.can.id = frame[0]
        can_stamped.can.data = frame[2]

        return can_stamped

    def send_command_with_response(self, command: bytes):
        # NOTE DO NOT RUN THIS IF YOU HAVE READING THREAD OPEN
        if not self.serial_port.is_open:
            return False

        msg = b'\015' + command + b'\015'
        self.serial_port.write(msg)
        response = self.serial_port.read()

        self.get_logger().info(f'Sent cmd: {command} | Response: {response}')
        return response == b'\x06'


from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
import signal
import sys

def main(args=None):
    
    shutdown_delay_seconds = 2.0
    context = Context()
    
    # Emergency shutdown handler - runs BEFORE ROS shutdown
    def emergency_shutdown(signum, frame):
        node.get_logger().info("Caught Ctrl+C — EMERGENCY SHUTDOWN")
        # Call hardware-level disable immediately
        if node.serial_port.is_open:
            # node.send_zero_throttle()
            node.get_logger().info("handle safety critical communications")
        else:
            node.get_logger().info("CAN bus not initialized — nothing can be sent.")
        
         # Now start delayed ROS shutdown in a thread
        def delayed_shutdown():
            node.get_logger().info(f"Waiting {shutdown_delay_seconds} seconds for clean shutdown...")
            time.sleep(shutdown_delay_seconds)
            executor.shutdown()

        shutdown_thread = threading.Thread(target=delayed_shutdown)
        shutdown_thread.start()

    # Install our emergency signal handler BEFORE ROS 2 starts
    signal.signal(signal.SIGINT, emergency_shutdown)

    # Disable ROS 2 signal handler
    rclpy.init(args=args, context=context, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    executor = SingleThreadedExecutor(context=context)
    node = CanDapter(context=context)
    executor.add_node(node)
    
    # try:
    #     # executor.spin()
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     print("\nkeyboard interrupt signal intercepted")
    #     # continue processing messages for 4 more seconds
    #     # allows for communication to and from the MoTec prior to shutdown
    #     rclpy.spin_until_future_complete(node, )
        
    # finally:
    #     print("\nshutting down")
    
    # spin until the executor is shutdown
    executor.spin()
    node.get_logger().info("executor done spinning")
    # Full shutdown after ROS executor exits
    node.shutdown()
    node.send_command(b'C')
    node.destroy_node()
    rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
