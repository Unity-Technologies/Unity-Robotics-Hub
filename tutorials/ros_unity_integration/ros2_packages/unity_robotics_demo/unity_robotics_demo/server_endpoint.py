import rclpy

from ros2_tcp_endpoint.server import TcpServer
from ros2_tcp_endpoint.publisher import RosPublisher
from ros2_tcp_endpoint.subscriber import RosSubscriber
from ros2_tcp_endpoint.service import RosService
from ros2_tcp_endpoint.unity_service import UnityService

#from unity_interfaces.msg import UnityColor
#from unity_interfaces.msg import PosRot, UnityColor
#from unity_interfaces.srv import PositionService, ObjectPoseService

def main(args=None):
    rclpy.init(args=args)

    ros_node_name = 'TCPServer'
    buffer_size = 1024
    connections = 10
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)

    tcp_server.start({
        #'pos_srv': RosService('pos_srv', PositionService),
        #'pos_rot': RosPublisher('pos_rot', PosRot, queue_size=10),
        #'color': RosSubscriber('color', UnityColor(), tcp_server),
        #'obj_pose_srv': UnityService('obj_pose_srv', ObjectPoseService, tcp_server),
    })

    # Setup executors for nodes defined in source_destination_dict
    tcp_server.setup_executor()

    # Clean up nodes defined in source_destination_dict
    tcp_server.destroy_nodes()
    rclpy.shutdown()

if __name__ == '__main__':
    main()