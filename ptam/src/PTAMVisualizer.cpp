#include <ros/ros.h>
#include <ptam_com/PointCloud.h>
#include <ptam_com/KeyFrame_srv.h>
#include <ptam_com/KeyFrame_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <ptam/PTAMVisualizerParamsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <ptam/AxesArray.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


typedef dynamic_reconfigure::Server<ptam::PTAMVisualizerParamsConfig> ReconfigureServer;
ReconfigureServer *reconfServer_;

// global variables
bool show_pc_;
bool show_kfs_;
bool show_all_kfs_;
unsigned int kf_lifetime_;
bool show_path_;
unsigned int path_length_;
sensor_msgs::PointCloud2* pPC2;
struct passwd *pw;
int KFFlags_;
unsigned int lastKFid;
visualization_msgs::Marker path;
ros::Publisher pub_path;
AxesArray tripods;
AxesArray tripodshistory;

void Config(ptam::PTAMVisualizerParamsConfig &config, uint32_t level);
void exportPC(std::string prefix);
void pathCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ptam_visualizer");
  ros::NodeHandle n;
  ros::ServiceClient PC2client = n.serviceClient<ptam_com::PointCloud>("vslam/pointcloud");
  ros::ServiceClient KFclient = n.serviceClient<ptam_com::KeyFrame_srv>("vslam/keyframes");
  ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("vslam/pc2", 1);
  ros::Publisher pub_kfs = n.advertise<ptam_com::KeyFrame_msg> ("vslam/kfs",1);
  ros::Publisher pub_marker = n.advertise<visualization_msgs::MarkerArray>("vslam/kf_visualization_array", 1);
  pub_path = n.advertise<visualization_msgs::Marker>("vslam/path_visualization", 1);

  ros::Subscriber sub_path = n.subscribe("vslam/pose",1,&pathCallback);

  ptam_com::PointCloud srv_pc2;
  ptam_com::KeyFrame_srv srv_kfs;
  unsigned int pathidx=0;

  pw = getpwuid(getuid());

  reconfServer_ = new ReconfigureServer(ros::NodeHandle("~"));
  ReconfigureServer::CallbackType f = boost::bind(&Config , _1, _2);
  reconfServer_->setCallback(f);

  pPC2=NULL;

  srv_pc2.request.flags = 0;
  lastKFid=0;
  tripodshistory.init(1);

  path.id=0;
  path.lifetime=ros::Duration(1);
  path.header.frame_id = "/world";
  path.header.stamp = ros::Time::now();
  path.ns = "pointcloud_publisher";
  path.action = visualization_msgs::Marker::ADD;
  path.type = visualization_msgs::Marker::LINE_STRIP;
  path.color.r=1.0;
  path.color.g=1.0;
  path.color.a=1.0;
  path.scale.x=0.01;
  path.pose.orientation.w=1.0;

  while(ros::ok())
  {
    visualization_msgs::MarkerArray cubes;
    if(show_pc_)
      if(PC2client.call(srv_pc2))
      {
        pPC2=&(srv_pc2.response.pointcloud);
        pub_cloud.publish(srv_pc2.response.pointcloud);
      }
    if(KFFlags_==1)
      srv_kfs.request.flags=lastKFid;
    else
      srv_kfs.request.flags = KFFlags_;

    if(KFclient.call(srv_kfs))
    {
      pub_kfs.publish(srv_kfs.response);

      tripods.init(kf_lifetime_);
      double pos[3], att[4];
      for(int i=srv_kfs.response.KFids.size()-1;!(i<0);--i)	//first element is newest KF
      {
        memcpy(pos,&(srv_kfs.response.KFs[i].pose.pose.position.x),sizeof(double)*3);
        att[0] = srv_kfs.response.KFs[i].pose.pose.orientation.w;
        memcpy(att+1,&(srv_kfs.response.KFs[i].pose.pose.orientation.x),sizeof(double)*3);
        tripods.addAxes(pos,att,srv_kfs.response.KFids[i]);
        if(lastKFid<=srv_kfs.response.KFids[i])
        {
          tripodshistory.addAxes(pos,att,srv_kfs.response.KFids[i]);
          lastKFid=srv_kfs.response.KFids[i]+1;
        }
      }
    }
    else
    {
      //reset if there is no map
      tripods.clearAxes();
      tripodshistory.clearAxes();
      path.points.clear();
      pathidx=0;
    }

    if(show_kfs_ & !show_all_kfs_)
    {
      cubes = tripods.getAxes();
      pub_marker.publish(cubes);
    }

    if(show_all_kfs_)
    {
      cubes = tripodshistory.getAxes();
      pub_marker.publish(cubes);
    }

    if(show_path_)
      pub_path.publish(path);

    usleep(1e6);
    ros::spinOnce();
  }

  delete reconfServer_;
  return 0;
}

void pathCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
{
  double pos[3], att[4];
  memcpy(pos,&(msg->pose.pose.position.x),sizeof(double)*3);
  att[0] = msg->pose.pose.orientation.w;
  memcpy(att+1,&(msg->pose.pose.orientation.x),sizeof(double)*3);

  TooN::Vector<3> center = tripods.getCenter(pos,att);
  geometry_msgs::Point p;
  memcpy(&(p.x),&(center[0]),sizeof(double)*3);
  path.points.push_back(p);
  while(path.points.size()>path_length_)
    path.points.erase(path.points.begin());

}

void exportPC(std::string prefix)
{
  FILE* fid;
  std::string strnsec;
  std::stringstream out;
  out << ros::Time::now().nsec;
  strnsec = out.str();

  std::string slash ="/";
  std::string strpc ="_PC";
  std::string filename = pw->pw_dir+slash+prefix+strnsec+strpc;
  fid = fopen(filename.c_str(),"w");
  if(fid!=NULL && pPC2!=NULL)
  {
    for(unsigned int i=0;i<pPC2->width;++i)
    {
      float* elemf = (float*)&(pPC2->data[i*pPC2->point_step]);
      int* elemi = (int*)&(pPC2->data[i*pPC2->point_step+3*sizeof(float)]);
      fprintf(fid,"%f\t%f\t%f\t%d\t%d\t%d\n",*elemf,*(elemf+1),*(elemf+2),*(elemi),*(elemi+1),*(elemi+2));
    }
    fclose(fid);
    ROS_INFO_STREAM("Point cloud exported to: " << filename);
  }
  else if(fid==NULL)
    ROS_WARN_STREAM("could not open file: " << filename);
  else
    ROS_WARN_STREAM("Could not export point cloud to file. Do we actually have one to export?");
}

void saveMap(std::string prefix)
{
  FILE* fid;
  std::string strnsec;
  std::stringstream out;
  out << ros::Time::now().nsec;
  strnsec = out.str();

  std::string slash ="/";
  std::string strkf ="_KF";
  std::string filename = pw->pw_dir+slash+prefix+strnsec+strkf;
  fid = fopen(filename.c_str(),"w");
  if((fid!=NULL) & (pPC2!=NULL) & (tripodshistory.getAxes().markers.size()>0) & (path.points.size()>0))
  {

    std::vector<visualization_msgs::Marker> cubes = tripodshistory.getAxes().markers;
    for(unsigned int i=0;i<cubes.size();i+=3)	// 3 markers per tripod
    {
      fprintf(fid,"%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",cubes[i].id/10,	// ID is stored by *10 since for each tripod there are 3 markers
              cubes[i].pose.position.x,cubes[i].pose.position.y,cubes[i].pose.position.z,
              cubes[i].pose.orientation.w,cubes[i].pose.orientation.x,cubes[i].pose.orientation.y,cubes[i].pose.orientation.z);
    }
    fclose(fid);
    ROS_INFO_STREAM("KFs exported to: " << filename);

    std::string strpath ="_Path";
    filename = pw->pw_dir+slash+prefix+strnsec+strpath;
    fid = fopen(filename.c_str(),"w");
    for(unsigned int i=0;i<path.points.size();++i)	// 3 markers per tripod
    {
      fprintf(fid,"%f\t%f\t%f\n",path.points[i].x,path.points[i].y,path.points[i].z);
    }
    fclose(fid);
    ROS_INFO_STREAM("Path exported to: " << filename);

    std::string strpc ="_PC";
    filename = pw->pw_dir+slash+prefix+strnsec+strpc;
    fid = fopen(filename.c_str(),"w");
    for(unsigned int i=0;i<pPC2->width;++i)
    {
      float* elemf = (float*)&(pPC2->data[i*pPC2->point_step]);
      int* elemi = (int*)&(pPC2->data[i*pPC2->point_step+3*sizeof(float)]);
      fprintf(fid,"%f\t%f\t%f\t%d\t%d\t%d\n",*elemf,*(elemf+1),*(elemf+2),*(elemi),*(elemi+1),*(elemi+2));
    }
    fclose(fid);
    ROS_INFO_STREAM("Point cloud exported to: " << filename);
  }
  else if(fid==NULL)
    ROS_WARN_STREAM("could not open file: " << filename);
  else
    ROS_WARN_STREAM("Could not export KFs and path to file. Do we actually have one to export?");
}

void Config(ptam::PTAMVisualizerParamsConfig& config, uint32_t level)
{
  show_pc_ = config.ShowPC;
  show_kfs_ = config.ShowKFs;
  show_all_kfs_ = config.ShowAllKFs;
  kf_lifetime_ = config.KFLifetime;
  show_path_ = config.ShowPath;
  KFFlags_ = config.KFFlags;
  path_length_ = config.PathLength;
  if(config.ExportPC)
  {
    config.ExportPC=false;
    exportPC(config.ExportPrefix);
  }
  if(config.SaveMap)
  {
    config.SaveMap=false;
    saveMap(config.ExportPrefix);
  }
}
