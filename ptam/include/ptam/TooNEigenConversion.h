/*
 * TooNEigenConversion.h
 *
 *  Created on: Mar 8, 2013
 *      Author: slynen
 */

#ifndef TOONEIGENCONVERSION_H_
#define TOONEIGENCONVERSION_H_

#include <Eigen/Dense>
#include <TooN/TooN.h>

namespace TooNToEigen{

void SE3PTAMToEigenWorld(const TooN::SE3<>& pose, Eigen::Matrix<double, 3, 1>& position, Eigen::Quaterniond& orientation){
  TooN::Matrix<3, 3, double> r = pose.get_rotation().get_matrix().T();
      TooN::Vector<3, double> t = - r * pose.get_translation();

      tf::Transform transform(tf::Matrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2),
                                              r(2, 0), r(2, 1), r(2, 2)), tf::Vector3(t[0], t[1], t[2]));

      //a little bit of copying, because bullet and Eigen are so very much compatible
      const tf::Vector3& position_tf = transform.getOrigin();
      const tf::Quaternion& orientation_tf = transform.getRotation();
      position<<position_tf.x(), position_tf.y(), position_tf.z();
      orientation.w() = orientation_tf.w();
      orientation.x() = orientation_tf.x();
      orientation.y() = orientation_tf.y();
      orientation.z() = orientation_tf.z();

}


}

#endif /* TOONEIGENCONVERSION_H_ */
