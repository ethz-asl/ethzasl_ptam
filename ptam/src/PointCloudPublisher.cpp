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
				pub_cloud.publish(srv_pc2.response.pointcloud);
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
			fprintf(fid,"%f\t%f\t%f\t%f\n",*elem,*(elem+1),*(elem+2),*(elem+3));
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
