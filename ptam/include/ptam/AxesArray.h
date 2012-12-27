/*
 * AxesArray.h
 *
 *  Created on: Jul 21, 2011
 *      Author: sweiss
 */

#ifndef AXESARRAY_H_
#define AXESARRAY_H_

#define AX_DIST 0.1

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <TooN/TooN.h>
#include <TooN/so3.h>

class AxesArray
{
private:
  visualization_msgs::MarkerArray cubes;
  unsigned int ID;
  TooN::Matrix<3,3> quaternion2matrix(const double q[4]);

  // temporary variables
  geometry_msgs::Point p;
  geometry_msgs::Point buffp;
  TooN::SO3<double> rot;
  TooN::Vector<3,double> center;
  TooN::Vector<3,double> buffvec;
  visualization_msgs::Marker buffcube;
  static const float red[4];
  static const float green[4];
  static const float blue[4];
  static const double dirx[3];
  static const double diry[3];
  static const double dirz[3];


public:
  AxesArray(){ID=0;};
  void init(double lifetime);	// shall we add here some init arguments?
  bool addAxes(const double pos[3], const double att[4],unsigned int id);
  void clearAxes() {cubes.markers.clear();};
  visualization_msgs::MarkerArray getAxes(){return cubes;};
  TooN::Vector<3> getCenter(const double pos[3], const double att[4]);
};

const float AxesArray::red[4]={1,0,0,1};
const float AxesArray::green[4]={0,1,0,1};
const float AxesArray::blue[4]={0,0,1,1};
const double AxesArray::dirx[3]={AX_DIST,0.01,0.01};
const double AxesArray::diry[3]={0.01,AX_DIST,0.01};
const double AxesArray::dirz[3]={0.01,0.01,AX_DIST};

void AxesArray::init(double lifetime)
{
  clearAxes();
  buffcube.lifetime=ros::Duration(lifetime);
  buffcube.header.frame_id = "/world";
  buffcube.header.stamp = ros::Time::now();
  buffcube.ns = "pointcloud_publisher";
  buffcube.action = visualization_msgs::Marker::ADD;
  buffcube.type = visualization_msgs::Marker::CUBE;
  memcpy(&(buffcube.scale.x),dirx,sizeof(double)*3);
  memcpy(&(buffcube.color.r),red,sizeof(float)*4);
}

TooN::Vector<3> AxesArray::getCenter(const double pos[3], const double att[4])
{
  rot = quaternion2matrix(att);
  //	return -(rot*TooN::makeVector(pos[0],pos[1],pos[2]));
  return -(rot.inverse()*TooN::makeVector(pos[0],pos[1],pos[2]));
}

bool AxesArray::addAxes(const double pos[3], const double att[4], unsigned int id)
{
  center = getCenter(pos, att);

  // set cube attitude
  buffcube.pose.orientation.w=att[0];
  buffcube.pose.orientation.x=-att[1];
  buffcube.pose.orientation.y=-att[2];
  buffcube.pose.orientation.z=-att[3];
  //	memcpy(&(buffcube.pose.orientation.x),&(att[1]),sizeof(double)*3);

  // add x-axis
  buffcube.id = 10*id;
  buffvec=center+rot.inverse()*TooN::makeVector(AX_DIST/2,0.0,0.0);
  memcpy(&(buffcube.pose.position.x),&(buffvec[0]),sizeof(double)*3);
  memcpy(&(buffcube.scale.x),dirx,sizeof(double)*3);
  memcpy(&(buffcube.color.r),red,sizeof(float)*4);
  cubes.markers.push_back(buffcube);

  // add y-axis, keep orientation
  buffcube.id = 10*id+1;
  buffvec=center+rot.inverse()*TooN::makeVector(0.0, AX_DIST/2,0.0);
  memcpy(&(buffcube.pose.position.x),&(buffvec[0]),sizeof(double)*3);
  memcpy(&(buffcube.scale.x),diry,sizeof(double)*3);
  memcpy(&(buffcube.color.r),green,sizeof(float)*4);
  cubes.markers.push_back(buffcube);

  // add z-axis, keep orientation
  buffcube.id = 10*id+2;
  buffvec=center+rot.inverse()*TooN::makeVector(0.0,0.0,AX_DIST/2);
  memcpy(&(buffcube.pose.position.x),&(buffvec[0]),sizeof(double)*3);
  memcpy(&(buffcube.scale.x),dirz,sizeof(double)*3);
  memcpy(&(buffcube.color.r),blue,sizeof(float)*4);
  cubes.markers.push_back(buffcube);

  return true;
}

TooN::Matrix<3,3> AxesArray::quaternion2matrix(const double q[4])
{
  // stolen from Eigen3 and adapted to TooN

  TooN::Matrix<3, 3, double> res;

  const double tx = 2 * q[1];
  const double ty = 2 * q[2];
  const double tz = 2 * q[3];
  const double twx = tx * q[0];
  const double twy = ty * q[0];
  const double twz = tz * q[0];
  const double txx = tx * q[1];
  const double txy = ty * q[1];
  const double txz = tz * q[1];
  const double tyy = ty * q[2];
  const double tyz = tz * q[2];
  const double tzz = tz * q[3];

  res(0, 0) = 1 - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1 - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1 - (txx + tyy);

  return res;
}


#endif  /// AXESARRAY_H_
