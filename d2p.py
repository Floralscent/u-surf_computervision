
             
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
import std_msgs.msg as std_msgs

class Dept2Point(Node):
    f_x = 204.2553253173828; f_y = 204.2553253173828  # focal length
    c_x = 320; c_y = 240
    k_matrix = [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]
    height = 480; width = 640

    def __init__(self):
        super().__init__('dept2point')
        self.get_logger().info('Camera info received with K matrix: {}'.format(self.k_matrix))
        self.get_logger().info('height: {}, width: {}'.format(self.height, self.width))
        
        self.subscription_depth = self.create_subscription(
            Image,
            '/jetbot/depth',
            self.depth_callback,
            10)

        self.subscription_odometry = self.create_subscription(
            Odometry,
            '/jetbot/odometry',
            self.odometry_callback,
            10)

        self.publisher = self.create_publisher(PointCloud2, 'pointcloud_msg', 10)

        # 초기 로봇 위치 및 방향
        self.robot_position = [0.0, 0.0, 0.0]
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion

    def odometry_callback(self, msg):
        # 오도메트리 콜백에서 로봇의 위치와 방향을 업데이트
        self.robot_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        self.robot_orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

    def depth_callback(self, msg):
        data_size = len(msg.data)
        self.get_logger().info(f"Received depth data size: {data_size}")

        try:
            depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(self.height, self.width)
            self.get_logger().info(f"Reshaped depth image with dtype=np.float32: {depth_image.shape}")
        except ValueError as e:
            self.get_logger().error(f"Error reshaping data: {e}")
            return

        pointcloud = self.depth2pointcloud(depth_image)

        # 로봇 위치와 방향으로 변환된 포인트 클라우드 생성
        transformed_pointcloud = self.transform_pointcloud(pointcloud)
        pointcloud_msg = self.create_point_cloud_msg(transformed_pointcloud, msg.header.stamp)
        
        self.publisher.publish(pointcloud_msg)

    def depth2pointcloud(self, depth_image):
        point_cloud = []
        for v in range(self.height):
            for u in range(self.width):
                Z = depth_image[v, u]
                if Z <= 0:  # Filter
                    continue
                X = (u - self.c_x) * Z / self.f_x
                Y = (v - self.c_y) * Z / self.f_y
                point_cloud.append([X, Y, Z])

        return np.array(point_cloud, dtype=np.float32)

    def transform_pointcloud(self, points):
        # 로테이션 행렬 R과 그 역행렬
        R = np.array([
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0]
        ], dtype=np.float32)
        R_inv = np.linalg.inv(R)

        # 변환 행렬 생성 (4x4)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R_inv
        transformation_matrix[:3, 3] = np.dot(R_inv, self.robot_position)

        # 포인트 클라우드 변환
        transformed_points = []
        for point in points:
            point_h = np.append(point, 1.0)  # homogeneous coordinates
            transformed_point_h = np.dot(transformation_matrix, point_h)
            transformed_points.append(transformed_point_h[:3])

        return np.array(transformed_points, dtype=np.float32)

    def create_point_cloud_msg(self, points, stamp):
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        header = std_msgs.Header()
        header.stamp = stamp
        header.frame_id = 'world' 

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

def main(args=None):
    rclpy.init(args=args)
    node = Dept2Point()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            
      
