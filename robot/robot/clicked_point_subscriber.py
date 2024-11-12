import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# /clicked_point topic değerini dinle ve veri geldikce kaydet
# cünkü bu veriler rviz den gelen istasyon kaydetme verisi olarak kullanılacak
class ClickedPointSubscriber(Node):
    def __init__(self):
        super().__init__("clicked_point_subscriber")
        self.subscription = self.create_subscription(
            PointStamped,
            "clicked_point",
            self.listener_callback,
            10
        )

    
    def listener_callback(self,msg : PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        station_file = open("stations.txt","a")
        station_file.write(f"x:{x},y:{y},z:{z}\n")
        station_file.close()
        self.get_logger().info("Hey station info is saved in stations.txt..\n")
    

def main(args=None):
    rclpy.init(args=args)

    clicked_point_subscriber = ClickedPointSubscriber()

    rclpy.spin(clicked_point_subscriber)

    clicked_point_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()