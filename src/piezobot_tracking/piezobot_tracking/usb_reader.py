import rclpy
from rclpy.node import Node
import usb.core

class USBReaderNode(Node):
    def __init__(self):
        super().__init__('usb_reader_node')
        self.device = None  # USB device object
        self.is_device_connected = False

    def detect_usb_device(self):
        # Detect the USB device if not already connected
        if not self.is_device_connected:
            self.device = usb.core.find(idVendor=YOUR_VENDOR_ID, idProduct=YOUR_PRODUCT_ID)
            if self.device is not None:
                self.is_device_connected = True
                self.get_logger().info('USB device detected')

    def read_usb_data(self):
        # Read data from the USB device if connected
        if self.is_device_connected:
            # Implement your data reading logic here
            # Example: Read data from an endpoint or interface
            endpoint = self.device[0][(0, 0)][0]
            data = self.device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
            # Process the data or publish it as ROS2 messages

    def run(self):
        while rclpy.ok():
            # Detect USB device if not already connected
            self.detect_usb_device()

            # Read data from USB if device is connected
            self.read_usb_data()

            # Sleep for a short duration before checking again
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    usb_reader_node = USBReaderNode()
    usb_reader_node.run()
    usb_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
