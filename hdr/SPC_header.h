/*
 * SPC_header.h
 *
 *  Created on: Mar 3, 2015
 *      Author: loyd-hook
 */

#ifndef SPC_HEADER_H_
#define SPC_HEADER_H_

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace cv;

class StereoMatch
{
public:
	StereoMatch(
			const char *l_image,
			const char *r_image,
			const char* intrinsic_file,
			const char *extrinsic_file,
			const char *output_path);
	void RunStereoMatch();
	void ProduceImagesAndPointCloud();
	void RectifyAndRemap();
private:
	Mat r_image_mat;
	Mat l_image_mat;
	Mat truth_disp;
    Ptr<StereoSGBM> sgbm;
    Mat disp, disp8;
    char disp_map_path [200];
    char intrinsic_filepath [200];
    char extrinsic_filepath [200];
    char output_path [200];
    Rect roi1, roi2;
    Mat Q;
};



#endif /* SPC_HEADER_H_ */
