#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <custom_msgs/pointData.h>
#include <custom_msgs/pointDataArray.h>
#include <custom_msgs/goalControl.h>
#include <custom_msgs/goalDetail.h>
#include <custom_msgs/areaReset.h>
#include <custom_msgs/position.h>

#include <memory>
#include <mutex>
#include <cmath>
#include <vector>
#include <algorithm>
#include <condition_variable>

octomap::OcTree* tree = nullptr;

std::vector<octomap::point3d> centerArray;
std::vector<octomap::point3d> ignorePoints;

octomap::point3d goal;
octomap::point3d currentPosition;

double x_positive, temp_x_positive;
double x_negative, temp_x_negative;
double y_positive, temp_y_positive;
double y_negative, temp_y_negative;
double z_positive;

double resolution;
double percentage;
double minScanRange;

int blockCount;

bool gotPosition = false;
bool gotTree = false;

std::mutex identifierMutex;
std::condition_variable mapReadyCV;
bool mapReady = false;

void updateArea()
{
    x_positive = temp_x_positive;
	x_negative = temp_x_negative;
	y_positive = temp_y_positive;
	y_negative = temp_y_negative;
}

/*
/   The node subscribe to topics 'Octomap' at (octomap_msgs/Octomap) and 'Position' at (custom_msgs/position)
/   mapCallback and positionCallback handles incoming msgs from these two topics respectively.  
*/

void positionCallback(const custom_msgs::position::ConstPtr &msg){
	currentPosition = octomap::point3d(msg->x, msg->y, msg->z);
}

void mapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    octomap::AbstractOcTree* tempTree = octomap_msgs::fullMsgToMap(*msg);

    std::unique_lock<std::mutex> lockExplore(identifierMutex);
    
    if (tree) delete tree;
    tree = (octomap::OcTree*) tempTree;

    mapReady = true;

    lockExplore.unlock();
    mapReadyCV.notify_one();
}

/*
//   Callbacks that implements the main process and other supporting processes
//   
//  executionCallback - main callback that implements the exploration calculation based on octomap. Calculation 
//                      is done upon the server request on the (execute) and response is given as (x,y,z) cooredinates.
//
//  removeCallback    - callback that updates the unreachable goal positions. request contains the removing corridinates
//                      and response contains success state.
//
//  resetCallback      - callback that updates the area limits. request contains the new area limits. and response contains 
//                      success state.
//
*/

bool executionCallback(custom_msgs::goalControl::Request &request, custom_msgs::goalControl::Response &response)
{
    centerArray.clear();

	bool completed;

    updateArea();

    std::unique_lock<std::mutex> lockExplore(identifierMutex);
    mapReadyCV.wait(lockExplore, []{return mapReady;});

    // Cluster count in positive direction

    int x_count_p = (int) (x_positive/(blockCount*resolution));
    int y_count_p = (int) (y_positive/(blockCount*resolution));
    int z_count_p = (int) (z_positive/(blockCount*resolution));

    // Cluster count in negative direction

    int x_count_n = (int) (-1*x_negative/(blockCount*resolution));
    int y_count_n = (int) (-1*y_negative/(blockCount*resolution));

	if (request.execute){
        std::vector<octomap::point3d> unknownPointsArray, tempUnknownPoints;
                
        for (int x=x_count_n; x<x_count_p; x++){
            for (int y=y_count_n; y<y_count_p; y++){
                for (int z=0; z<z_count_p; z++){

                    int counter = 0;
                    octomap::point3d point = octomap::point3d ((blockCount*resolution)*(x+0.5), (blockCount*resolution)*(y+0.5), (blockCount*resolution)*(z+0.5));
            
                    double x_min = point.x()-((blockCount-1)*0.5*resolution);
                    double x_max = point.x()+(blockCount*0.5*resolution);
                    double y_min = point.y()-((blockCount-1)*0.5*resolution);
                    double y_max = point.y()+(blockCount*0.5*resolution);
                    double z_min = point.z()-((blockCount-1)*0.5*resolution);
                    double z_max = point.z()+(blockCount*0.5*resolution);

                    for (double a=x_min; a<x_max; a+=resolution){
                        for (double b=y_min; b<y_max; b+=resolution){
                            for (double c=z_min; c<z_max; c+=resolution){
                                if (!tree->search(a, b, c)){
                                    counter++;                       // number of unidentified voxels
                                }
                            }
                        }
                    }

                    if (counter>=(percentage*blockCount*blockCount*blockCount)){
                        unknownPointsArray.push_back(point);
                    }
                }
            }
        }

        for(int i=0; i<unknownPointsArray.size(); i++){
            bool notfound = true;
            for(int j=0; j<ignorePoints.size(); j++) if (unknownPointsArray[i].distance(ignorePoints[j])<(resolution*2)) notfound = false;
            if (notfound) tempUnknownPoints.push_back(unknownPointsArray[i]);    
        }

        for(int j=0; j<tempUnknownPoints.size(); j++) if (tempUnknownPoints[j].z()<(blockCount*resolution)) centerArray.push_back(tempUnknownPoints[j]);

        if (centerArray.size()>0){
            octomap::point3d nearestCluster = centerArray[centerArray.size()-1];

            double min_distance = std::sqrt(std::pow(currentPosition.x() - nearestCluster.x(), 2) + std::pow(currentPosition.y() - nearestCluster.y(), 2) + pow(currentPosition.z() - nearestCluster.z(), 2));

            for(int i=0; i<centerArray.size(); i++){
                octomap::point3d cluster = centerArray[i];

                double distance = std::sqrt(std::pow(currentPosition.x() - cluster.x(), 2) + std::pow(currentPosition.y() - cluster.y(), 2) + std::pow(currentPosition.z() - cluster.z(), 2));

                if ((distance>minScanRange) && (distance<min_distance)){
                    min_distance = distance;
                    nearestCluster = cluster;
                }
            }

            goal = nearestCluster;
            completed = false;
        } else {
            octomap::point3d origin (0,0,0);
            goal = origin;
            completed = true;
        }
    }

    lockExplore.unlock();
    mapReadyCV.notify_one();

	response.x = goal.x();
	response.y = goal.y();
	response.z = goal.z();
	response.isNull = completed;

	return true;
}

bool removeCallback(custom_msgs::goalDetail::Request &request, custom_msgs::goalDetail::Response &response)
{
	octomap::point3d point(request.x, request.y, request.z);

    std::vector<octomap::point3d>::iterator position = std::find(ignorePoints.begin(), ignorePoints.end(), point);
    
    if (position == ignorePoints.end()) ignorePoints.push_back(point);

	response.success = true;

	return true;
}

bool resetCallback(custom_msgs::areaReset::Request &request, custom_msgs::areaReset::Response &response)
{
	ROS_DEBUG("explore_node : reset request received");

    temp_x_positive = request.x_positive;
    temp_x_negative = request.x_negative;
    temp_y_positive = request.y_positive;
    temp_y_negative = request.y_negative;
    // z_positive = request.z_positive;     ---> not used

	response.success = true;

	ROS_DEBUG("explore_node : succesfully resetted");

	return true;
}


/*  goal and centerPointArray which are returned by identifierObject is published in (custom_msgs/goal) topic and 
/   (custom_msgs/centerArray). These utilize custom messages defined in 'msg' folder.
*/

int main(int argc, char **argv)
{
	ros::init (argc, argv, "explore_node");
	ros::NodeHandle node;

	ROS_INFO("Initialized the explore_node");

    node.param("area/front",                        x_positive,     5.0);
    node.param("area/back",                         x_negative,     0.0);
    node.param("area/left",                         y_positive,     5.0);
    node.param("area/right",                        y_negative,     0.0);
    node.param("area/height",                       z_positive,     0.5);

    node.param("cluster/resolution",                resolution,     0.05);
    node.param("cluster/unexploredPercentage",      percentage,     0.95);
    node.param("cluster/sideBlockCount",            blockCount,     5);

    node.param("map/minScanRange",                  minScanRange,   0.70);

    temp_x_positive = x_positive;
    temp_x_negative = x_negative;
    temp_y_positive = y_positive;
    temp_y_negative = y_negative;

    ROS_INFO("explore_node : loaded parameters");

    ros::AsyncSpinner spinner(6);
    spinner.start();

    ros::Subscriber oct_sub = node.subscribe("octomap", 1, mapCallback);
	ros::Subscriber pos_sub = node.subscribe("position", 1, positionCallback);

	ROS_INFO("explore_node : created subscribers");

	ros::ServiceServer serviceFind = node.advertiseService<custom_msgs::goalControlRequest, custom_msgs::goalControlResponse>("goalCalculate", executionCallback);
	ros::ServiceServer serviceRemo = node.advertiseService<custom_msgs::goalDetailRequest, custom_msgs::goalDetailResponse>("goalRemove", removeCallback);
    ros::ServiceServer serviceRese = node.advertiseService<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetExplore", resetCallback);
	
	ROS_INFO("explore_node : created service");

	ros::Publisher centerArray_pub = node.advertise<custom_msgs::pointDataArray>("goalCenterArray", 1, true);
	ros::Publisher centerPoint_pub = node.advertise<custom_msgs::pointData>("goalCenters", 1, true);
	
	ROS_INFO("explore_node : created publishers");

	while(ros::ok()){
        // publish centerArray data

		custom_msgs::pointDataArray valueArray;
		custom_msgs::pointData goalMsg;

		for(int i=0; i<centerArray.size(); i++){
			custom_msgs::pointData msgInstance;

			msgInstance.x = centerArray[i].x();
			msgInstance.y = centerArray[i].y();
			msgInstance.z = centerArray[i].z();

			valueArray.centerPointsArray.push_back(msgInstance);
		}

		centerArray_pub.publish(valueArray);

		goalMsg.x = goal.x();
		goalMsg.y = goal.y();
		goalMsg.z = goal.z();

		centerPoint_pub.publish(goalMsg);
	}

	return 0;
}