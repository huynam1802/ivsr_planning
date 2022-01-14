
// #include <ros/ros.h>
// #include <ewok/ed_ring_buffer.h>

// #include <thread>
// #include <chrono>
// #include <map>

// #include <Eigen/Core>
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <std_srvs/Empty.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
// #include <tf/message_filter.h>
// #include <message_filters/subscriber.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>

// #include <visualization_msgs/MarkerArray.h>

// //#include <opencv2/imgproc/imgproc.hpp>
// //#include <opencv2/highgui/highgui.hpp>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/image_encodings.h>

// #include <fstream>
// #include <iostream>

// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>

// const int POW = 6;       //static const int POW = 6;

// bool initialized = false;

// std::ofstream f_time, opt_time;

// ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
// ros::Publisher occ_marker_pub, updated_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, upt_marker_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;

// tf::TransformListener * listener;

// //ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;
// void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
//     //ROS_INFO("recieved depth image");
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     //Convert Encoding Image when test with camera real
//     // try
//     // {
//     //     cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
//     // }
//     // catch (cv_bridge::Exception& e)
//     // {
//     //     ROS_ERROR("cv_bridge exception: %s", e.what());
//     //     return;
//     // }



//     // r200
//     const float fx = 554.254691191187;//check?
//     const float fy = 554.254691191187;
//     const float cx = 320.5;
//     const float cy = 240.5;

//     //aligned depth
//     // const float fx = 383.848;
//     // const float fy = 383.848;
//     // const float cx = 313.883;
//     // const float cy = 238.281;


//     tf::StampedTransform transform;

//     try{
//         // Camera Simulation
//         listener->lookupTransform("/map", "/camera_link", msg->header.stamp, transform); //world              //"camera_link", msg->header.frame_id
//         //lookupTransform(target_frame,msg->header.frame_id,  msg_in.header.stamp, transform) where transform "frame_id" to "target_frame"

//         // Camera Real
//         //listener->lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);
       
//         // ros::Time now = ros::Time::now();
//         // listener->waitForTransform("/map", "/camera_link", now, ros::Duration(10.0) );
//         // listener->lookupTransform("/map", "/camera_link", now, transform); 

        
//         // ros::Time now = ros::Time::now();
//         // ros::Time past = now - ros::Duration(5.0);
//         // listener->waitForTransform("/map", now,
//         //                         "/camera_link", past,
//         //                         "/world", ros::Duration(1.0));
//         // listener->lookupTransform("/map", now,
//         //                         "/camera_link", past,
//         //                         "/world", transform);
//     }
//     catch (tf::TransformException &ex) {
//         ROS_INFO("Couldn't get transform");
//         ROS_WARN("%s",ex.what());
//         return;
//     }

//     Eigen::Affine3d dT_w_c;
//     tf::transformTFToEigen(transform, dT_w_c);

//     Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

//     std :: cout << "\n\n================================" << std::endl;
//     std :: cout << "\n" << T_w_c.matrix()<< std::endl;
    
//     float * data = (float *) cv_ptr->image.data;
   
//     auto t1 = std::chrono::high_resolution_clock::now();

//     ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;
    
//     for(int u=0; u < cv_ptr->image.cols; u+=4 ) {           //u+=4
//         for(int v=0; v < cv_ptr->image.rows; v+=4) {        //v+=4
//             float val = data[v*cv_ptr->image.cols + u];

//             //ROS_INFO_STREAM(val);
//             if(std::isfinite(val)) {
//                 Eigen::Vector4f p;
//                 p[0] = val*(u - cx)/fx;
//                 p[1] = val*(v - cy)/fy;
//                 p[2] = val;
//                 p[3] = 1;

//                 p = T_w_c * p;                              //(4x4)x(4x1)
          
//                 cloud.push_back(p);
//             }       
//         }
//     }
//     Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

//     auto t2 = std::chrono::high_resolution_clock::now();

//     if(!initialized) {
//         Eigen::Vector3i idx;
//         edrb->getIdx(origin, idx);

//         ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

//         edrb->setOffset(idx);

//         initialized = true;
//     } else {
//         Eigen::Vector3i origin_idx, offset, diff;
//         edrb->getIdx(origin, origin_idx);
 
//         offset = edrb->getVolumeCenter();
//         diff = origin_idx - offset;

//         while(diff.array().any()) {
//             //ROS_INFO("Moving Volume");
//             edrb->moveVolume(diff);

//             offset = edrb->getVolumeCenter();
//             diff = origin_idx - offset;
//         }
//     }

//     auto t3 = std::chrono::high_resolution_clock::now();

//     edrb->insertPointCloud(cloud, origin);

//     auto t4 = std::chrono::high_resolution_clock::now();

//     f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
//               std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
//               std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

//     visualization_msgs::Marker m_occ, m_free, m_updated;
//     edrb->getMarkerOccupied(m_occ);
//     edrb->getMarkerFree(m_free);
//     edrb->getMarkerUpdated(m_updated);


//     occ_marker_pub.publish(m_occ);
//     free_marker_pub.publish(m_free);
//     upt_marker_pub.publish(m_updated); 
// }

// int main(int argc, char** argv) {

//     ros::init(argc, argv, "rolling_buffer_test");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");
    
//     std::string path = "benchmarking/";

//     ROS_INFO_STREAM("path: " << path);

//     f_time.open(path + "mapping_time.txt");
//     opt_time.open(path + "optimization_time.txt");

//     listener = new tf::TransformListener;

//     occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
//     free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
//     updated_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
//     dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
//     //command_pt_viz_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/command_pt", 5);
//     upt_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
//     //ros::Subscriber depth_img_sub = nh.subscribe<sensor_msgs::Image>("camera/depth/image_raw", 1, depthImageCallback);

//     message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;

//     //Simulation 
//     depth_image_sub_.subscribe(nh, "/camera/depth/image_raw", 5);

//     //Test 
//     //depth_image_sub_.subscribe(nh, "/camera/depth/image_rect_raw", 5);//aligned_depth_to_color
    

//     //Simulation
//     tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5); 

//     //Test with D435
//     //tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_depth_optical_frame", 5);
    
//     tf_filter_.registerCallback(depthImageCallback);

//     double resolution;
//     pnh.param("resolution", resolution, 0.15);
//     edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));
//     std::cout << "Matrix Transform" << std::endl;

//     ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
//     ros::Rate r(1);
//     while (ros::ok())
//     {
//         r.sleep();

//         visualization_msgs::Marker m_occ, m_free, m_dist, m_updated;
//         rrb.getMarkerOccupied(m_occ);
//         rrb.getMarkerFree(m_free);
//         rrb.getMarkerUpdated(m_updated);
//         rrb.getMarkerDistance(m_dist, 0.9);

//         occ_marker_pub.publish(m_occ);
//         free_marker_pub.publish(m_free);
//         upt_marker_pub.publish(m_updated);
//         dist_marker_pub.publish(m_dist);

//         rrb.moveVolume(Eigen::Vector3i(1,0,0)); 

//         ros::spinOnce();
//     }

//     ros::spin();  // goi lai ham callbacks trong main

//     f_time.close();
//     opt_time.close();
//     return 0;
    
// }

#include <ros/ros.h>
#include <ewok/ed_ring_buffer.h>
#include <thread>
#include <chrono>
#include <map>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <fstream>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>



const int POW = 6;       //static const int POW = 6;
//static const int N = (1 << POW);

// double dt ;
// int num_opt_points;      // number optimization points

bool initialized = false;

std::ofstream f_time, opt_time;


ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;

ros::Publisher occ_marker_pub, updated_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, upt_marker_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;

tf::TransformListener * listener;

//ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO("recieved depth image");

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    const float fx = 554.254691191187;
    const float fy = 554.254691191187;
    const float cx = 320.5;
    const float cy = 240.5;

    tf::StampedTransform transform;

    try{

        listener->lookupTransform("/map", "/camera_link", msg->header.stamp, transform); //world              //"camera_link", msg->header.frame_id
        //lookupTransform(target_frame,msg->header.frame_id,  msg_in.header.stamp, transform); 
        // transform "frame_id" to "target_frame"
    }
    catch (tf::TransformException &ex) {
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s",ex.what());
        return;
    }


    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    std :: cout << "\n\n================================" << std::endl;
    std :: cout << "\n" << T_w_c.matrix();

    float * data = (float *) cv_ptr->image.data;


    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    for(int u=0; u < cv_ptr->image.cols; u+=4) {
        for(int v=0; v < cv_ptr->image.rows; v+=4) {
            float val = data[v*cv_ptr->image.cols + u];

            //ROS_INFO_STREAM(val);

            if(std::isfinite(val)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;

                p = T_w_c * p;                         //(4x4)x(4x1)

                cloud.push_back(p);
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;
        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);
 
        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }
    }

    //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());

    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud, origin);

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free, m_updated;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);
    edrb->getMarkerUpdated(m_updated);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    upt_marker_pub.publish(m_updated); 
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "rolling_buffer_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string path = "benchmarking/";

    ROS_INFO_STREAM("path: " << path);

    f_time.open(path + "mapping_time.txt");
    opt_time.open(path + "optimization_time.txt");

    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
    updated_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
    //command_pt_viz_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/command_pt", 5);
    upt_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
    //ros::Subscriber depth_img_sub = nh.subscribe<sensor_msgs::Image>("camera/depth/image_raw", 1, depthImageCallback);

    std::cout << "Hello1_________________" << std::endl;

    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;
    depth_image_sub_.subscribe(nh, "/camera/depth/image_raw", 5);      // add camera_link
    //ros::Subscriber depth = nh.subscribe("camera/depth/image_raw", 1, depthImageCallback);
    
    std::cout << "Hello2_________________" << std::endl;

    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);  //camera_link
    tf_filter_.registerCallback(depthImageCallback);

    std::cout << "Hello3_________________" << std::endl;
    
    double resolution;
    pnh.param("resolution", resolution, 0.15);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));
    std::cout << "Hello4_________________" << std::endl;

    ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);

    ros::Rate r(1);
    while (ros::ok())
    {
        r.sleep();

        visualization_msgs::Marker m_occ, m_free, m_dist, m_updated;
        rrb.getMarkerOccupied(m_occ);
        rrb.getMarkerFree(m_free);
        rrb.getMarkerUpdated(m_updated);
        rrb.getMarkerDistance(m_dist, 0.9);

        occ_marker_pub.publish(m_occ);
        free_marker_pub.publish(m_free);
        upt_marker_pub.publish(m_updated);
        dist_marker_pub.publish(m_dist);

        rrb.moveVolume(Eigen::Vector3i(1,0,0)); 

        ros::spinOnce();
    }


    //ros::spin();

    f_time.close();
    opt_time.close();
    std::cout << "Hello5_________________" << std::endl;
    return 0;
    
}


/*
#include <ros/ros.h>
#include <ewok/ed_ring_buffer.h>

#include <thread>
#include <chrono>
#include <map>

#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <visualization_msgs/MarkerArray.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <fstream>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

const int POW = 6;       //static const int POW = 6;

bool initialized = false;

std::ofstream f_time, opt_time;

ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
ros::Publisher occ_marker_pub, updated_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, upt_marker_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;

tf::TransformListener * listener;

//ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO("recieved depth image");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Convert Encoding Image
    // try
    // {
    //     cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }



    // r200
    const float fx = 554.254691191187;//check?
    const float fy = 554.254691191187;
    const float cx = 320.5;
    const float cy = 240.5;

    //aligned depth
    // const float fx = 383.848;
    // const float fy = 383.848;
    // const float cx = 313.883;
    // const float cy = 238.281;


    tf::StampedTransform transform;

    try{
        // Camera Simulation
        //listener->lookupTransform("/map", "/camera_link", msg->header.stamp, transform); //world              //"camera_link", msg->header.frame_id
        //lookupTransform(target_frame,msg->header.frame_id,  msg_in.header.stamp, transform); // transform "frame_id" to "target_frame"

        // Camera Real
        listener->lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);
       
        // ros::Time now = ros::Time::now();
        // listener->waitForTransform("/map", "/camera_link", now, ros::Duration(10.0) );
        // listener->lookupTransform("/map", "/camera_link", now, transform); 

        
        // ros::Time now = ros::Time::now();
        // ros::Time past = now - ros::Duration(5.0);
        // listener->waitForTransform("/map", now,
        //                         "/camera_link", past,
        //                         "/world", ros::Duration(1.0));
        // listener->lookupTransform("/map", now,
        //                         "/camera_link", past,
        //                         "/world", transform);
    }
    catch (tf::TransformException &ex) {
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s",ex.what());
        return;
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    std :: cout << "\n\n================================" << std::endl;
    std :: cout << "\n" << T_w_c.matrix()<< std::endl;
    
    float * data = (float *) cv_ptr->image.data;

    
   
    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;
    
    for(int u=0; u < cv_ptr->image.cols; u+=2 ) {           //u+=4
        for(int v=0; v < cv_ptr->image.rows; v+=2) {        //v+=4
            float val = data[v*cv_ptr->image.cols + u];

            //ROS_INFO_STREAM(val);
            if(std::isfinite(val)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;
                if( (u==0)&&(v > 200)&&(v < 220)){
                std::cout<<"\n\n" "p[21]= " <<p[2] << std::endl;
                }
                p = T_w_c * p; 
                //std::cout<<"\n\n" "p[22]= " <<p[2] << std::endl;
                                             //(4x4)x(4x1)
          
                cloud.push_back(p);
            }       
        }
    }
    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;
        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);
 
        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud, origin);

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free, m_updated;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);
    edrb->getMarkerUpdated(m_updated);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    upt_marker_pub.publish(m_updated); 
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "rolling_buffer_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    std::string path = "benchmarking/";

    ROS_INFO_STREAM("path: " << path);

    f_time.open(path + "mapping_time.txt");
    opt_time.open(path + "optimization_time.txt");

    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
    updated_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
    //command_pt_viz_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/command_pt", 5);
    upt_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 5);
    //ros::Subscriber depth_img_sub = nh.subscribe<sensor_msgs::Image>("camera/depth/image_raw", 1, depthImageCallback);

    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;

    //Simulation 
    //depth_image_sub_.subscribe(nh, "/camera/depth/image_raw", 5);

    //Test 
    depth_image_sub_.subscribe(nh, "/camera/depth/image_rect_raw", 5);//aligned_depth_to_color
    

    //Simulation
    //tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5); 

    //Test with D435
    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_depth_optical_frame", 5);
    
    tf_filter_.registerCallback(depthImageCallback);

    double resolution;
    pnh.param("resolution", resolution, 0.15);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));
    std::cout << "Matrix Transform" << std::endl;

    ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    ros::Rate r(1);
    // while (ros::ok())
    // {
    //     r.sleep();

    //     visualization_msgs::Marker m_occ, m_free, m_dist, m_updated;
    //     rrb.getMarkerOccupied(m_occ);
    //     rrb.getMarkerFree(m_free);
    //     rrb.getMarkerUpdated(m_updated);
    //     rrb.getMarkerDistance(m_dist, 0.9);

    //     occ_marker_pub.publish(m_occ);
    //     free_marker_pub.publish(m_free);
    //     upt_marker_pub.publish(m_updated);
    //     dist_marker_pub.publish(m_dist);

    //     rrb.moveVolume(Eigen::Vector3i(1,0,0)); 

    //     ros::spinOnce();
    // }

    ros::spin();  // goi lai ham callbacks trong main

    f_time.close();
    opt_time.close();
    return 0;
    
}*/



