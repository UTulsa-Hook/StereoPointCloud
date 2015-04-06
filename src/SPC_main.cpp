/*
 * SPC_main.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: loyd-hook
 */

#include "SPC_header.h"

int main(int argc, char** argv)
{
	StereoMatch stereo_match("/home/loyd-hook/0Projects/SV/Images/DevelopmentPics06/SD06_O0000_0.bmp", //l_image
			"/home/loyd-hook/0Projects/SV/Images/DevelopmentPics06/SD06_O0000_1.bmp", //r_image
			"/home/loyd-hook/0Projects/SV/software/eclipse_ws/CameraCalibration/Debug/mycalib.yml", //intrinsic
			"/home/loyd-hook/0Projects/SV/software/eclipse_ws/StereoCalibration/Debug/mystereocalib.yml", //extrinsic
			"/home/loyd-hook/0Projects/SV/Images/DevelopmentPics06/"); //output path

	stereo_match.RectifyAndRemap();

	stereo_match.RunStereoMatch();

	stereo_match.ProduceImagesAndPointCloud();


	printf("press any key to continue...");
	fflush(stdout);
	waitKey();
	printf("\n");


    return 0;
}


