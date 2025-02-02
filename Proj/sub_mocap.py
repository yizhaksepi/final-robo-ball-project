import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mocap_interfaces.msg import RigidBodies

class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            RigidBodies,
            '/mocap/rigid_bodies',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # print(msg,"msg")
        print(msg.rigid_bodies[0].id,"id")
        print(msg.rigid_bodies[0].pose_stamped.pose.position.x,"x pose")
        print(msg.rigid_bodies[0].pose_stamped.pose.position.y,"y pose")
        print(msg.rigid_bodies[0].pose_stamped.pose.position.z,"z pose")
        print(msg.rigid_bodies[0].pose_stamped.pose.orientation.x,"x rot")
        print(msg.rigid_bodies[0].pose_stamped.pose.orientation.y,"y rot")
        print(msg.rigid_bodies[0].pose_stamped.pose.orientation.z,"z rot")
        print(msg.rigid_bodies[0].pose_stamped.pose.orientation.w,"w rot")
        
        #print(msg.rigid_bodies[1].id,"id")
        #print(msg.rigid_bodies[1].pose_stamped.pose.position.x,"x pose")
        #print(msg.rigid_bodies[1].pose_stamped.pose.position.y,"y pose")
        #print(msg.rigid_bodies[1].pose_stamped.pose.position.z,"z pose")

        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PositionSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
