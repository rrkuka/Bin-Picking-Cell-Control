import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import random

# Create door node     
class Ebutton(Node): 
    def __init__(self):
        super().__init__('ebutton')

        # Intial state
        self.pressed = None

        # Create topic 
        self.publisher = self.create_publisher(Bool,'e_button_status',10)

        # publish current door status
        self.create_timer(0.5,self.publish_ebutton_status)

         # create service to trigger ebutton press
        self.create_service(Trigger, 'press',self.press_service)

         # create service to trigger ebutton releases
        self.create_service(Trigger, 'release',self.release_service)

        # Start up msg
        # self.get_logger().info(f'E-button state: {})


    # Publish Ebutton status
    def publish_ebutton_status(self):
        msgs = Bool()
        msgs.data = self.pressed
        self.publisher.publish(msgs)
        print('E-button Status:',msgs.data)
    
    def press_service(self,request,response):
        self.pressed = True
        response.success = True
        response.message = 'Emergency button pressed!.'
        return response
    def release_service(self,request,response):
        self.pressed = False
        response.success = True
        response.message = 'Emergency button released.'
        return response

#    
def main():
        rclpy.init()
        node = Ebutton()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()