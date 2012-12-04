/*
 * agast_corner_detect.h
 *
 *  Created on: 01.12.2010
 *      Author: slynen
 *      wrapper for the agast detector
 *      to be used as PTAM corner detector functor
 */

#ifndef AGAST_CORNER_DETECT_H_
#define AGAST_CORNER_DETECT_H_

#include "agast7_12d.h"
#include "oast9_16.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <opencv/cv.h>

namespace agast{

namespace agast7_12d{

static AgastDetector7_12d detector;

//libcvd
__inline__ void agast_corner_detect12d(const CVD::BasicImage<CVD::byte>& im, std::vector<CVD::ImageRef>& corners, int barrier)
{
	detector.set_imageSize(im.size().x, im.size().y);
	detector.set_threshold(barrier);
	std::vector<CvPoint> keypoints;
	detector.detect((unsigned char*)im.data(),keypoints);
	corners.reserve(keypoints.size());
	for(size_t i=0;i<keypoints.size();++i) //convert to imageRefs
	{
		corners.push_back(CVD::ImageRef(keypoints.at(i).x, keypoints.at(i).y));
	}
	keypoints.clear();
}

//opencv
__inline__ void agast_corner_detect12d(const cv::Mat& im, std::vector<CVD::ImageRef>& corners, int barrier)
{
	detector.set_imageSize(im.size().width, im.size().height);
	detector.set_threshold(barrier);
	std::vector<CvPoint> keypoints;
	detector.detect((unsigned char*)im.data,keypoints);
	corners.reserve(keypoints.size());
	for(size_t i=0;i<keypoints.size();++i) //convert to imageRefs
	{
		corners.push_back(CVD::ImageRef(keypoints.at(i).x, keypoints.at(i).y));
	}
	keypoints.clear();
}

}

namespace oast9_16{
static OastDetector9_16 detector;
//libcvd
__inline__ void oast_corner_detect16(const CVD::BasicImage<CVD::byte>& im, std::vector<CVD::ImageRef>& corners, int barrier)
{
	detector.set_imageSize(im.size().x, im.size().y);
	detector.set_threshold(barrier);
	std::vector<CvPoint> keypoints;
	detector.detect((unsigned char*)im.data(),keypoints);
	corners.reserve(keypoints.size());
	for(size_t i=0;i<keypoints.size();++i) //convert to imageRefs
	{
		corners.push_back(CVD::ImageRef(keypoints.at(i).x, keypoints.at(i).y));
	}
	keypoints.clear();
}

//opencv
__inline__ void oast_corner_detect16(const cv::Mat& im, std::vector<CVD::ImageRef>& corners, int barrier)
{	
	detector.set_imageSize(im.size().width, im.size().height);
	detector.set_threshold(barrier);
	std::vector<CvPoint> keypoints;
	detector.detect((unsigned char*)im.data,keypoints);
	corners.reserve(keypoints.size());
	for(size_t i=0;i<keypoints.size();++i) //convert to imageRefs
	{
		corners.push_back(CVD::ImageRef(keypoints.at(i).x, keypoints.at(i).y));
	}
	keypoints.clear();
}

}

}
#endif /* AGAST_CORNER_DETECT_H_ */
