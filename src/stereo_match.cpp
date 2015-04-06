/*
 * stereo_match.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: loyd-hook
 */
/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "SPC_header.h"

StereoMatch::StereoMatch(const char *l_image,
		const char *r_image, const char* intrinsic_file, const char *extrinsic_file,
		const char *output_path)
{
	Mat l_o_img;
	Mat r_o_img;
    l_o_img = imread(l_image, 0); //Black and white images
    r_o_img = imread(r_image, 0);
	Size img_size(640,480);
	resize(l_o_img, l_image_mat, img_size);
	resize(r_o_img, r_image_mat, img_size);
    if (l_image_mat.empty())
	{
		printf("Command-line parameter error: could not load the left input image file\n");
		exit(-1);
	}
	if (r_image_mat.empty())
	{
		printf("Command-line parameter error: could not load the right input image file\n");
		exit(-1);
	}
	strcpy(this->intrinsic_filepath, intrinsic_file);
	strcpy(this->extrinsic_filepath, extrinsic_file);
	strcpy(this->output_path, output_path);

	sgbm = StereoSGBM::create(0,16,3);
}

void StereoMatch::RunStereoMatch()
{
	Rect roi1, roi2;
	Mat Q;
	int SADWindowSize = 0, numberOfDisparities = 0;

	Size img_size = l_image_mat.size();
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

	sgbm->setPreFilterCap(63);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = l_image_mat.channels();

	sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(10);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);

	int64 t = getTickCount();

	sgbm->compute(l_image_mat, r_image_mat, disp);
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t*1000/getTickFrequency());


	disp.convertTo(disp8, CV_8U);

	char disparity_filename[200];
	sprintf(disparity_filename, "%s//SD03_D0000_0.bmp", output_path);

	imwrite(disparity_filename, disp8);

}
static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}
void StereoMatch::ProduceImagesAndPointCloud()
{
	char point_cloud_file[200];
	sprintf(point_cloud_file, "%s/SD03_P0000_0.pcd", this->output_path);

	namedWindow("left", 1);
	imshow("left", l_image_mat);
	namedWindow("right", 1);
	imshow("right", r_image_mat);
	namedWindow("disparity", 0);
	imshow("disparity", disp8);
	//Produce the point cloud
	printf("storing the point cloud...");
	fflush(stdout);
	Mat xyz;
	reprojectImageTo3D(disp, xyz, Q, true);
	saveXYZ(point_cloud_file, xyz);
	printf("\n");

}

void StereoMatch::RectifyAndRemap()
{
	//reading intrinsic parameters
	FileStorage fs(intrinsic_filepath, FileStorage::READ);
//	if(!fs.isOpened())
//	{
//		printf("Failed to open file %s\n", intrinsic_filepath);
//		exit (-1);
//	}
//
//	Mat M1, D1, M2, D2;
//	fs["M1"] >> M1;
//	fs["D1"] >> D1;
//	fs["M2"] >> M2;
//	fs["D2"] >> D2;

	fs.open(extrinsic_filepath, FileStorage::READ);
	if(!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filepath);
		exit (-1);
	}
	Mat M1, D1, M2, D2;
	fs["CM1"] >> M1;
	fs["D1"] >> D1;
	fs["CM2"] >> M2;
	fs["D2"] >> D2;
	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

	stereoRectify( M1, D1, M2, D2, l_image_mat.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, l_image_mat.size(), &roi1, &roi2 );

	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, l_image_mat.size(), CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, l_image_mat.size(), CV_16SC2, map21, map22);

	Mat img1r, img2r;
	remap(l_image_mat, img1r, map11, map12, INTER_LINEAR);
	remap(r_image_mat, img2r, map21, map22, INTER_LINEAR);

	l_image_mat = img1r;
	r_image_mat = img2r;
}
