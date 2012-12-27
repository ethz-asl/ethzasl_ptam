#include <ros/ros.h>
#include <sfly_srvs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ptam/PC2ParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>




typedef dynamic_reconfigure::Server<ptam::PC2ParamsConfig> ReconfigureServer;
ReconfigureServer *reconfServer_;

// global variables
bool publish_pc_;
sensor_msgs::PointCloud2* pPC2;
sensor_msgs::PointCloud2 pubPC2;
struct passwd *pw;

void Config(ptam::PC2ParamsConfig &config, uint32_t level);
void exportPC(std::string prefix);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_publisher");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<sfly_srvs::PointCloud>("vslam/pointcloud");
  ros::Publisher pub_cloud= n.advertise<sensor_msgs::PointCloud2> ("vslam/pc2", 1);
  sfly_srvs::PointCloud srv_pc2;

  pw = getpwuid(getuid());

  reconfServer_ = new ReconfigureServer(ros::NodeHandle("~"));
  ReconfigureServer::CallbackType f = boost::bind(&Config , _1, _2);
  reconfServer_->setCallback(f);

  pPC2=NULL;

  srv_pc2.request.flags = 0;

  while(ros::ok())
  {
    if(publish_pc_)
      if(client.call(srv_pc2))
      {
        pPC2=&(srv_pc2.response.pointcloud);


        int dimension   = 4;
        resp.pointcloud.header.seq=pPC2->header.seq;
        pubPC2.header.stamp = pPC2->header.stamp;
        pubPC2.height = 1;
        pubPC2.header.frame_id = "/world";
        pubPC2.width = pPC2->width;
        pubPC2.fields.resize(dimension);
        pubPC2.fields[0].name = "x";
        pubPC2.fields[0].offset = 0*sizeof(float);
        pubPC2.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pubPC2.fields[0].count = 1;
        pubPC2.fields[1].name = "y";
        pubPC2.fields[1].offset = 1*sizeof(float);
        pubPC2.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pubPC2.fields[1].count = 1;
        pubPC2.fields[2].name = "z";
        pubPC2.fields[2].offset = 2*sizeof(float);
        pubPC2.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pubPC2.fields[2].count = 1;
        pubPC2.fields[3].name = "rgb";
        pubPC2.fields[3].offset = 3*sizeof(float);
        pubPC2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        pubPC2.fields[3].count = 1;

        pubPC2.point_step = dimension*sizeof(float);
        pubPC2.row_step = pubPC2.point_step * pubPC2.width;
        pubPC2.data.resize(pubPC2.row_step * pubPC2.height);
        pubPC2.is_dense = false;

        unsigned char* dat = &(pubPC2.data[0]);
        for(int i=0;i<pPC2->width;i++)
        {
          memcpy(dat, &(pPC2->data[0]),pubPC2.point_step);
          dat+=pubPC2.point_step;
        }

        pub_cloud.publish(pubPC2);
      }
    usleep(1e6);
    ros::spinOnce();
  }

  delete reconfServer_;
  return 0;
}


void exportPC(std::string prefix)
{
  FILE* fid;
  std::string strnsec;
  std::stringstream out;
  out << ros::Time::now().nsec;
  strnsec = out.str();

  std::string slash ="/";
  std::string filename = pw->pw_dir+slash+prefix+strnsec;
  fid = fopen(filename.c_str(),"w");
  if(fid!=NULL && pPC2!=NULL)
  {
    for(unsigned int i=0;i<pPC2->width;++i)
    {
      float* elem = (float*)&(pPC2->data[i*pPC2->point_step]);
      fprintf(fid,"%f\t%f\t%f\t%f\t%f\t%f\n",*elem,*(elem+1),*(elem+2),*(elem+3),*(elem+4),*(elem+5));
    }
    fclose(fid);
  }
  else if(fid==NULL)
    ROS_WARN_STREAM("could not open file: " << filename);
  else
    ROS_WARN_STREAM("Could not export pointcloud to file. Do we actually have one to export?");
}


void Config(ptam::PC2ParamsConfig& config, uint32_t level)
{
  publish_pc_=config.PublishPC;
  if(config.ExportPC)
  {
    config.ExportPC=false;
    exportPC(config.ExportPrefix);
  }
}
