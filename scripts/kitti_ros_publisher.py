import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
# write a ros publish function for me
def publish():
    # read the kitti point cloud
    data_path = "/home/your/dataset_path/sequences/"
    
    pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
    rospy.init_node('kitti_ros_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    frame_id = 0
    seq_num = 0
    while not rospy.is_shutdown():
        try:
            # read the kitti point cloud
            data_filename = data_path + str(seq_num).zfill(2) + "/velodyne/" + str(frame_id).zfill(6) + ".bin"
            point_cloud = np.fromfile(data_filename, dtype=np.float32).reshape(-1, 4)
            if(point_cloud.shape[0] == 0):
                rospy.logerr("seq: " + str(seq_num).zfill(2) + " frame: " + str(frame_id).zfill(6) + " not found.")
                exit()
            # transform the point_cloud to ros pointcloud2
            pcd_msg = PointCloud2()
            pcd_msg.header.stamp = rospy.Time.now()
            pcd_msg.header.frame_id = "laser_link"
            pcd_msg.height = 1
            pcd_msg.width = point_cloud.shape[0]
            pcd_msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1),
            ]
            pcd_msg.is_bigendian = False
            pcd_msg.point_step = 16
            pcd_msg.row_step = pcd_msg.point_step * point_cloud.shape[0]
            pcd_msg.is_dense = False
            pcd_msg.data = point_cloud.tostring()
            # publish the point cloud
            pub.publish(pcd_msg)
            rate.sleep()
            frame_id += 1
            print("seq: ", str(seq_num).zfill(2), " frame: ", frame_id)
        except:
            print("seq: ", str(seq_num).zfill(2), " finished.")

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass