#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <stdlib.h>
#include <algorithm>

#include <sure/sure_estimator.h>

sure::Configuration config;
sure::SURE_Estimator<pcl::PointXYZRGB> sure1;

pcl::PointCloud<pcl::PointXYZI>::Ptr InterestfromPCD(char *fileName)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fileName, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    sure1.setInputCloud(cloud);
    sure1.calculateFeatures();
    pcl::PointCloud<pcl::InterestPoint>::Ptr features = sure1.getInterestPoints();
    //std::vector<sure::Feature> fff = sure1.getFeatures();
    //cout << fff.size() << " :::: " << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr featureCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(pcl::PointCloud<pcl::InterestPoint>::const_iterator it=features->points.begin(); it != features->points.end(); ++it)
    {
        pcl::PointXYZI *p;
        p = new pcl::PointXYZI;
        p->x = (*it).x;
        p->y = (*it).y;
        p->z = (*it).z;
        p->intensity = (*it).strength;
        (*featureCloud).push_back(*p);
    }
    return featureCloud;
}
bool ConfComp(float *i1, float *i2){return (i1[5]<i2[5])&&(i1[6]>=i2[6]);};


#define POP_LEN 60
#define RET_POP 10


int main (int argc, char *argv[])
{
    float best_score=1000;
    //float ssize=0.25,srate=0.25,nsrate=1,nscale=1,mcorn=0.001;
    std::vector<float*> Samples;
    float limitsUp[] = {2,2,2,2,0.5};
    float limitsLo[] = {0.1,0.1,0.1,0.1,0.001};
    for(int i=0;i<POP_LEN;i++)
    {
        float *klk= new float[7];
        for(int j=0;j<5;j++)
            klk[j]=((float) rand()) /(RAND_MAX)*(limitsUp[j]-limitsLo[j])+limitsLo[j];
        klk[5]=100000;
        klk[6]=1;
        Samples.push_back(klk);
    }
    for(int Nit=0;Nit<500;Nit++)
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
            for(int kp=0;kp<7;kp++)
                std::cout<<(*params)[kp]<<":";
            cout<<endl;
            ///////////////////////////

            config.setSize((*params)[0]);
            config.setSamplingRate((*params)[1]);
            config.setNormalsScale((*params)[2]);
            config.setNormalSamplingRate((*params)[3]);
            config.setMinimumCornerness((*params)[4]);
            sure1.setConfig(config);
            pcl::PointCloud<pcl::PointXYZI>::Ptr Final(new pcl::PointCloud<pcl::PointXYZI>);
            (*Final)+= *InterestfromPCD(argv[1]);
            float score_sum=0;
            int featurenum=Final->size();
            int Nframes=1;
            for(int fN=2;fN<argc;fN++)
            {
                (*params)[5]=score_sum>0?score_sum/Nframes:100000;
                (*params)[6]=Nframes;
                pcl::PointCloud<pcl::PointXYZI>::Ptr featureCloud = InterestfromPCD(argv[fN]);
                if(featureCloud->size()<4)
                    break;
                featurenum+=featureCloud->size();
                pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
                icp.setInputSource(Final);
                icp.setInputTarget(featureCloud);
                icp.align((*Final));
                Final=featureCloud;
                score_sum+=icp.getFitnessScore()/std::min(featureCloud->size(),Final->size());
                Nframes++;
            }
      
        }
        std::sort(Samples.begin(),Samples.end(),ConfComp);
        for(int kp=0;kp<7;kp++)
            std::cout<<Samples[0][kp]<<":";
        cout<<"Best   "<<endl;
    }
    return 0;
}
