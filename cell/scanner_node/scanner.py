import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import random

# Returns a random barcode digit as a string 
def make_barcode():
    return str(random.randint(10000, 99999))

# Create scanner node     
class Scan(Node): 
    def __init__(self):
        super().__init__('scan')
        self.last_scan = '00000'

        # Create topic 
        self.publisher = self.create_publisher(String,'barcode',10)

        # Call publisher barcode number
        self.create_timer(2,self.publish_barcode)

        # create service to calling last barcode 
        self.create_service(Trigger, 'last_barcode',self.handle_service)

        # Start up msg
        self.get_logger().info('Barcode started')

    # make barcode number and publish
    def publish_barcode(self):
        self.last_barcode = make_barcode()
        msgs = String()
        msgs.data = self.last_barcode
        self.publisher.publish(msgs)
        print('Barcode scan:',msgs.data)

    # Create service for last scan from barcode
    def handle_service(self,request,response):
        response.success = True
        response.message = self.last_barcode
        return response
#    
def main():
        rclpy.init()
        node = Scan()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()