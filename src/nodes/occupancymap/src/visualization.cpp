#include "visualization.h"
#include <libks/base/sdlwindow.h>
#include <libks/base/cvwindow.h>
#include <rosbag/bag.h>
#include "settings.h"

namespace occupancymap {
using namespace std;
using namespace ks;
using namespace cv;
using namespace ros;
using namespace octomap;
using namespace tf;

Visualization::Visualization(NodeHandle& nh, boost::shared_ptr<Parameters> parameters)
    :parameters(parameters), publisher(nh.advertise<visualization_msgs::MarkerArray>("/occupancymap/visualization", 1)),
      seqNum(0), visualizationCalls(0) {

    const char* windowTitle = "Disparity Visualization";
    if(parameters->graphicalSDL)
        window.reset(new SDLWindow(640, 480, windowTitle));
    else if(parameters->graphicalCV)
        window.reset(new CVWindow(640, 480, windowTitle));

    stereoColCoder.reset(new ColorCoder(0, parameters->maxDisp, true, false, ColorCoder::BLUE_WHITE_RED));
    octomapColCoder.reset(new ColorCoder(parameters->occupancyThreshold, parameters->clampingThresholdMax,
                                         false, false, ColorCoder::BLUE_WHITE_RED));
}

void Visualization::visualizeAll(const Mat_<float>& dispMap, const Mat_<Point3f>& pointsMap, OcTreeType& ocTree,
                                 const Vector3& position, const Quaternion& orientation, ros::Time stamp) {
    if(publisher.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        createPointsMarker(pointsMap, dispMap, markers, position, orientation, stamp);

        if((++visualizationCalls) % parameters->octreePublishInterval == 0)
            createOctreeMarkers(ocTree, markers, stamp);

        publisher.publish(markers);

        // DEBUG code for saving visualization messages
        // Make sure there's a subscriber!
        /*static rosbag::Bag bag("/home/schauwecker/data/octomap_markers.bag", rosbag::bagmode::Write);
            bag.write("/occupancymap/visualization", stamp, markers);

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = position.x();
            pose.pose.position.y = position.y();
            pose.pose.position.z = position.z();
            pose.pose.orientation.x = orientation.x();
            pose.pose.orientation.y = orientation.y();
            pose.pose.orientation.z = orientation.z();
            pose.pose.orientation.w = orientation.w();
            pose.header.stamp = stamp;
            pose.header.frame_id = parameters->worldFrame;
            pose.header.seq = seqNum;
            bag.write("/imufuse/pose_camera", stamp, pose);*/
    }

    if(window != NULL && dispMap.data != NULL)
        visualizeDisparities(dispMap, pointsMap);
}

void Visualization::visualizeOctree(OcTreeType& ocTree, ros::Time stamp) {
    if(publisher.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        createOctreeMarkers(ocTree, markers, stamp);
        publisher.publish(markers);
    }
}

void Visualization::visualizeDisparities(const Mat_<float>& dispMap, const Mat_<Point3f>& pointsMap) {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    // Resize window if neccessary
    int width = dispMap.cols*parameters->subsampling;
    int height = dispMap.rows*parameters->subsampling;
    if(height != screen.rows || width != screen.cols) {
        screen = stereoColCoder->createLegendBorder(width, height);
        window->resize(screen.cols, screen.rows);
    }

    // Render disparity map
    Mat_<Vec3b> colImg(dispMap.size());
    stereoColCoder->codeImage(dispMap, colImg);
    for(int y=0; y<height; y++)
        for(int x=0; x<width; x++) {
            Vec3b col = colImg(y/parameters->subsampling, x/parameters->subsampling);
            screen(y,x) = col;
        }

    // Draw horizon
    window->displayImage(screen);
    window->processEvents(false);
}


void Visualization::initMarkerMsg(visualization_msgs::Marker& marker, ros::Time stamp) {
    marker.header.frame_id = parameters->worldFrame;
    marker.header.seq = seqNum;
    marker.header.stamp = stamp;
    marker.id = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
}

void Visualization::createPointsMarker(const Mat_<Point3f>& pointsMap, const Mat_<float>& dispMap,
                                       visualization_msgs::MarkerArray& markers, const Vector3& position, const Quaternion& orientation, ros::Time stamp) {

    Transform trans(orientation, position);

    visualization_msgs::Marker marker;
    initMarkerMsg(marker, stamp);

    marker.ns = "points";
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.type = visualization_msgs::Marker::POINTS;

    unsigned int i=0;
    for(int y=0; y<pointsMap.rows; y++)
        for(int x=0; x<pointsMap.cols; x++) {
            if (dispMap(y,x) > 0 && finite(pointsMap(y,x).z)) {
                Vector3 transformed = trans * Vector3(pointsMap(y,x).x, pointsMap(y,x).y, pointsMap(y,x).z);

                std_msgs::ColorRGBA msgCol;
                if(dispMap.data != NULL) {
                    Vec3b col = stereoColCoder->getColor(dispMap(y,x));
                    msgCol.r = col[2] / 255.0f;
                    msgCol.g = col[1] / 255.0f;
                    msgCol.b = col[0] / 255.0f;
                } else {
                    msgCol.r = msgCol.g = msgCol.b = 1.0;
                }
                msgCol.a = 1.0f;
                marker.colors.push_back(msgCol);

                geometry_msgs::Point msgPoint;
                msgPoint.x = transformed.x();
                msgPoint.y = transformed.y();
                msgPoint.z = transformed.z();
                marker.points.push_back(msgPoint);

                i++;
            }
        }

    markers.markers.push_back(marker);
}

void Visualization::createOctreeMarkers(OcTreeType& ocTree, visualization_msgs::MarkerArray& markers, ros::Time stamp) {
    visualization_msgs::Marker marker;
    initMarkerMsg(marker, stamp);

    // Create marker messages for different scales
    double markerSize = ocTree.getResolution();
    for(int i=0; i<4; i++) {
        char name[50];
        sprintf(name, "occupied-%d", i);
        marker.ns = name;
        marker.scale.x = markerSize;
        marker.scale.y = markerSize;
        marker.scale.z = markerSize;
        markerSize*= 2.0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;

        // Different default colors for diferent scales
        switch(i) {
        case 0: marker.color.r = 0.0; marker.color.g = 0.75; marker.color.b = 0.0; break;
        case 1: marker.color.r = 0.75; marker.color.g = 0.75; marker.color.b = 0.0; break;
        case 2: marker.color.r = 0.75; marker.color.g = 0.0; marker.color.b = 0.0; break;
        default: marker.color.r = 0.75; marker.color.g = 0.0; marker.color.b = 0.75; break;
        }

        marker.color.a = 1.0;
        markers.markers.push_back(marker);
    }

    /*unsigned short minZKey = 0, maxZKey = 0xFFFF;

        OcTreeKey key;
        if(ocTree.coordToKeyChecked(0, 0, parameters->minHeight, key))
            minZKey = key[2];

        if(ocTree.coordToKeyChecked(0, 0, parameters->maxHeight, key))
            maxZKey = key[2];*/

    // Iterate over all leaf nodes
    for(OcTreeType::leaf_iterator it = ocTree.begin_leafs(); it!= ocTree.end_leafs(); it++) {
        if(!ocTree.isNodeOccupied(*it) /*|| it.getKey()[2] < minZKey || it.getKey()[2] > maxZKey*/)
            continue;

        // Find the correct message for this scale
        int scale = it.getSize() / ocTree.getResolution() + 0.5;
        int index = 0;
        switch(scale) {
        case 1: index = markers.markers.size()-4; break;
        case 2: index = markers.markers.size()-3; break;
        case 4: index = markers.markers.size()-2; break;
        default: index = markers.markers.size()-1;
        }

        // Insert into message
        geometry_msgs::Point msgPoint;
        msgPoint.x = it.getX();
        msgPoint.y = it.getY();
        msgPoint.z = it.getZ();
        markers.markers[index].points.push_back(msgPoint);

        Vec3b col = octomapColCoder->getColor((float)it->getOccupancy());
        std_msgs::ColorRGBA colMsg;
        colMsg.a = 1.0;
        colMsg.r = col[2]/255.0;
        colMsg.g = col[1]/255.0;
        colMsg.b = col[0]/255.0;
        markers.markers[index].colors.push_back(colMsg);
    }
}
}
