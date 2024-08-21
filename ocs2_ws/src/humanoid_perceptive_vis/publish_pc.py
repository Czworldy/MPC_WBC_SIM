import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import std_msgs.msg

height_bias_map = 0
height_bias_obj = 6

def read_ply_file(file_path):
    # 使用 open3d 读取 .ply 文件
    pcd = o3d.io.read_point_cloud(file_path)
    points = pcd.points
    colors = pcd.colors
    return points, colors

def point_cloud_to_ros(points, height_bias, colors=None):
    # 创建 PointCloud2 消息
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    if colors is not None:
        fields += [
            PointField('r', 12, PointField.FLOAT32, 1),
            PointField('g', 16, PointField.FLOAT32, 1),
            PointField('b', 20, PointField.FLOAT32, 1),
        ]
        points_and_colors = zip(points, colors)
        
        cloud_data = [[point[0], point[1], point[2] + height_bias, color[0], color[1], color[2]]for point, color in points_and_colors]

    else:
        cloud_data = points

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pc2 = point_cloud2.create_cloud(header, fields, cloud_data)
    return pc2

    

if __name__ == '__main__':
    map_file_path = './map0709-filter.ply'
    obj_file_path = '/home/cyx/vis_bag/map0709-filter.ply'

    rospy.init_node('point_cloud_publisher')
    map_pub = rospy.Publisher('map_point_cloud', PointCloud2, queue_size=10)
    # obj_pub = rospy.Publisher('obj_point_cloud', PointCloud2, queue_size=10)

    map_points, map_colors = read_ply_file(map_file_path)
    map_cloud = point_cloud_to_ros(map_points, height_bias_map, map_colors)

    # obj_points, obj_colors = read_ply_file(obj_file_path)
    # obj_cloud = point_cloud_to_ros(obj_points, height_bias_obj, obj_colors)

    rate = rospy.Rate(1) # 每秒发布一次
    while not rospy.is_shutdown():
        map_pub.publish(map_cloud)
        # obj_pub.publish(obj_cloud)
        rate.sleep()
