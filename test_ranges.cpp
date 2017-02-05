//
// Created by anna on 1/30/17.
//
/*

#include <pcl/common/projection_matrix.h>
#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleHDLViewer
{
public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer (Grabber& grabber,
                     pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler) :
            cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
            grabber_ (grabber),
            handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)
    {
        boost::mutex::scoped_lock lock (cloud_mutex_);
        cloud_ = cloud;
    }

    void run ()
    {
        cloud_viewer_->addCoordinateSystem (3.0);
        cloud_viewer_->setBackgroundColor (0, 0, 0);
        cloud_viewer_->initCameraParameters ();
        cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
        cloud_viewer_->setCameraClipDistances (0.0, 50.0);

        boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
                &SimpleHDLViewer::cloud_callback, this, _1);
        boost::signals2::connection cloud_connection = grabber_.registerCallback (
                cloud_cb);

        grabber_.start ();

        while (!cloud_viewer_->wasStopped ())
        {
            CloudConstPtr cloud;

            // See if we can get a cloud
            if (cloud_mutex_.try_lock ())
            {
                cloud_.swap (cloud);
                cloud_mutex_.unlock ();
            }

            if (cloud)
            {
                handler_.setInputCloud (cloud);
                if (!cloud_viewer_->updatePointCloud (cloud, handler_, "HDL"))
                    cloud_viewer_->addPointCloud (cloud, handler_, "HDL");

                cloud_viewer_->spinOnce ();
            }

            if (!grabber_.isRunning ())
                cloud_viewer_->spin ();

            boost::this_thread::sleep (boost::posix_time::microseconds (100));
        }

        grabber_.stop ();

        cloud_connection.disconnect ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
    std::string hdlCalibration, pcapFile;

    parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
    parse_argument (argc, argv, "-pcapFile", pcapFile);

    pcl::HDLGrabber grabber (hdlCalibration, pcapFile);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");

    SimpleHDLViewer<PointXYZI> v (grabber, color_handler);
    v.run ();
    return (0);
}