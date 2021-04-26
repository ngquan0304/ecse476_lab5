// detect_object
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// uses published transform for table frame
// extracts pointcloud points above the table surface
// return a coordinate of the object pose (for now can only detect 1 single object)

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_utils/pcl_utils.h> //a local library with some utility fncs
#include <xform_utils/xform_utils.h>

#include "detect_object/DetectObjectServiceMsg.h"


int g_ans;

using namespace std;
PclUtils *g_pcl_utils_ptr;

void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr, vector<int> &indices)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object
    pass.setInputCloud(input_cloud_ptr);     //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z");            // we will "filter" based on points that lie within some range of z-value
    pass.setFilterLimits(0.02, 0.05);        //retain points with z values between these limits
    pass.filter(indices);                    //  this will return the indices of the points in given cloud that pass our test
    cout << "number of points passing the filter = " << indices.size() << endl;
    //This fnc populates the reference arg "indices", so the calling fnc gets the list of interesting points
}

Eigen::Affine3f get_table_frame_wrt_camera()
{
    bool tferr = true;
    int ntries = 0;
    XformUtils xformUtils;
    tf::TransformListener tfListener;
    tf::StampedTransform table_frame_wrt_cam_stf;

    Eigen::Affine3f affine_table_wrt_camera;
    while (tferr)
    {
        tferr = false;
        try
        {
            tfListener.lookupTransform("camera_depth_optical_frame", "table_frame", ros::Time(0), table_frame_wrt_cam_stf);
        }
        catch (tf::TransformException &exception)
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
            ntries++;
            if (ntries > 5)
            {
                ROS_WARN("did you launch robot's table_frame_wrt_cam.launch?");
                ros::Duration(1.0).sleep();
            }
        }
    }
    ROS_INFO("tf is good for table w/rt camera");
    xformUtils.printStampedTf(table_frame_wrt_cam_stf);

    tf::Transform table_frame_wrt_cam_tf = xformUtils.get_tf_from_stamped_tf(table_frame_wrt_cam_stf);

    affine_table_wrt_camera = xformUtils.transformTFToAffine3f(table_frame_wrt_cam_tf);

    //ROS_INFO("affine: ");
    //xformUtils.printAffine(affine_table_wrt_camera);
    return affine_table_wrt_camera;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_object"); //node name
    ros::NodeHandle nh;
    ROS_INFO("instantiating a pclUtils object");
    PclUtils pclUtils(&nh);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_wrt_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts_above_right_table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB block_centroid;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, table_frame_pts and pts_above_table");

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcd", 1);
    ros::Publisher pubTableFrame = nh.advertise<sensor_msgs::PointCloud2>("table_frame_pts", 1);
    ros::Publisher pubPointsAboveTable = nh.advertise<sensor_msgs::PointCloud2>("pts_above_table", 1);
    ros::Publisher pubPointsAboveRightTable = nh.advertise<sensor_msgs::PointCloud2>("pts_above_right_table", 1);

    sensor_msgs::PointCloud2 ros_cloud_wrt_table, ros_pts_above_table, ros_cloud_orig, ros_pts_above_right_table;
    //ros_cloud_wrt_table.header.frame_id = "table_frame";

    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud_orig); //convert from PCL cloud to ROS message this way
    ros_cloud_orig.header.frame_id = "camera_depth_optical_frame";

    //find the transform of table w/rt camera and convert to an affine
    Eigen::Affine3f affine_table_wrt_cam, affine_cam_wrt_table;
    affine_table_wrt_cam = get_table_frame_wrt_camera();
    affine_cam_wrt_table = affine_table_wrt_cam.inverse();

    pclUtils.transform_cloud(affine_cam_wrt_table, pclKinect_clr_ptr, output_cloud_wrt_table_ptr);
    pcl::toROSMsg(*output_cloud_wrt_table_ptr, ros_cloud_wrt_table);
    ros_cloud_wrt_table.header.frame_id = "table_frame";

    //cout<<"enter 1: ";
    //cin>>g_ans;
    //find indicies of points above table:
    vector<int> indices;
    find_indices_of_plane_from_patch(output_cloud_wrt_table_ptr, indices);
    pcl::copyPointCloud(*output_cloud_wrt_table_ptr, indices, *pts_above_table_ptr); //extract these pts into new cloud
    pcl::toROSMsg(*pts_above_table_ptr, ros_pts_above_table);
    ros_pts_above_table.header.frame_id = "table_frame";

    Eigen::Vector3f box_pt_min, box_pt_max;
    box_pt_min << 0.05, -1, 0.029;
    box_pt_max << 1, 0.2, 0.1;

    vector<int> indices2;
    pclUtils.box_filter(pts_above_table_ptr, box_pt_min, box_pt_max, indices);
    pcl::copyPointCloud(*pts_above_table_ptr, indices, *pts_above_right_table_ptr); //extract these pts into new cloud
    pcl::toROSMsg(*pts_above_right_table_ptr, ros_pts_above_right_table);           //convert to ros message for publication and display
    ros_pts_above_right_table.header.frame_id = "table_frame";

    // Reduce the height of the block by half for the correct centroid calculation
    int pts_block_size = pts_above_right_table_ptr->points.size();
    float tot_x = 0;
    float tot_y = 0;
    for (int i = 0; i < pts_block_size; i++)
    {
        tot_x += pts_above_right_table_ptr->points[i].x;
        tot_y += pts_above_right_table_ptr->points[i].y;
    }

    float block_x = tot_x/pts_block_size;
    float block_y = tot_y/pts_block_size;
    float block_z = 0;

    while (ros::ok())
    {
        pubTableFrame.publish(ros_cloud_wrt_table);
        pubCloud.publish(ros_cloud_orig); // will not need to keep republishing if display setting is persistent
        pubPointsAboveTable.publish(ros_pts_above_table);
        pubPointsAboveRightTable.publish(ros_pts_above_right_table);

        transform.setOrigin(tf::Vector3(block_x, block_y, block_z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_frame", "block_frame"));

        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.3).sleep();
    }

    return 0;
}
