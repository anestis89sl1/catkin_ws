#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rosbag/bag.h"
#include "rosbag/player.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h>
#include "std_msgs/Int16.h"
#include <sys/types.h>
#include <signal.h>
#include <wait.h>
#include <nav_msgs/Odometry.h>


bool ConfComp(float *i1, float *i2){return (i1[5]<=i2[5])&&(i1[6]>=i2[6]);};


#define POP_LEN 15
#define RET_POP 5

int numfeatures=0;
bool stop=false;
void featureNumHandler(const std_msgs::Int16::ConstPtr& stopSig)
{
    numfeatures+=stopSig->data;
    if(stopSig->data<5){
        stop=true;
    }
}
float poseDist=0;
void odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::Point cur = odom->pose.pose.position;
    poseDist=sqrt(pow(cur.x+5.1665,2)+pow(cur.y-0.5616722,2)+pow(cur.z+26.09049,2));
    //std::cout<<poseDist<<std::endl;
}

int main (int argc, char *argv[])
{
    rosbag::Bag bag;
    std::string filename;

    ros::init(argc, argv, "genetic");
    ros::NodeHandle nh;
    ros::Publisher pubFeatParams;
    pubFeatParams = nh.advertise<std_msgs::Float32MultiArray> ("/SUREparams", 5);
    ros::Subscriber subStop = nh.subscribe<std_msgs::Int16> ("/stoper", 1, featureNumHandler);
    ros::Subscriber subPose= nh.subscribe<nav_msgs::Odometry> ("/laser_odom_to_init", 5, odomHandler);
    float best_score=1000;
    //float ssize=0.25,srate=0.25,nsrate=1,nscale=1,mcorn=0.001;
    std::vector<float*> Samples;
    float limitsUp[] = {2,2,2,2,0.5};
    float limitsLo[] = {0.1,0.1,0.1,0.1,0.001};
    std_msgs::Float32MultiArray msgParams;
    ros::Rate rate(1);

    for(int i=0;i<POP_LEN;i++)
    {
        float *klk= new float[7];
        for(int j=0;j<5;j++)
            klk[j]=((float) rand()) /(RAND_MAX)*(limitsUp[j]-limitsLo[j])+limitsLo[j];
        klk[5]=100000;
        klk[6]=1;
        Samples.push_back(klk);
    }
    for(int Nit=0;Nit<200;Nit++)
    {
        for (std::vector<float*>::iterator params = Samples.begin()+RET_POP ; params!=Samples.end(); ++params)
        {
            //////////////////////////mutate
            std::memcpy(*params, Samples[rand()%RET_POP],5*(sizeof(float)));
            int index=rand()%5;
            if(rand()%2)
            {
                (*params)[index]=((float) rand()) /(RAND_MAX)*(limitsUp[index]-limitsLo[index])+limitsLo[index];
            }
            else
            {
                int whoI=rand()%RET_POP;
                (*params)[index]=Samples[whoI][index];
            }
            msgParams.data.clear();
            for(int kp=0;kp<7;kp++)
            {
                std::cout<<(*params)[kp]<<":";
                msgParams.data.push_back((*params)[kp]);
            }
            std::cout<<std::endl;
            pubFeatParams.publish(msgParams);
            ///////////////////////////o
            numfeatures=0;
            poseDist=0;
            stop=false;
            pid_t pid=fork();
            if(pid==0)
            {
                setpgid(getpid(),getpid());
                system("rosbag play --clock --quiet --duration 20  ~/Downloads/velodyne.bag");
                _exit(EXIT_SUCCESS);
            }
            else
            {
                int status;
                while(!stop&&!waitpid(pid,&status,WNOHANG)){
                    ros::spinOnce();
                    rate.sleep();
                }
                (*params)[5]=poseDist>0?poseDist:10000;
                (*params)[6]=numfeatures;
                kill(-pid, SIGTERM);
            }

      
        }
        std::sort(Samples.begin(),Samples.end(),ConfComp);
        for(int kp=0;kp<7;kp++)
            std::cout<<Samples[0][kp]<<":";
        std::cout<<"Best   "<<std::endl;
    }
    return 0;
}
