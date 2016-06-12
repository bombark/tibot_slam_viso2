/*
 * Copyright 2012. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * This file is part of libviso2.
 * Authors: Andreas Geiger
 *
 * libviso2 is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or any later version.
 *
 * libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
 * Street, Fifth Floor, Boston, MA 02110-1301, USA 
 *
 * Documented C++ sample code of stereo visual odometry (modify to your needs)
 * To run this demonstration, download the Karlsruhe dataset sequence
 * '2010_03_09_drive_0019' from: www.cvlibs.net!
 * Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/*=====================================  HEADER  ======================================*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_stereo.h>
#include <tiobj.hpp>
#include <tiobj-cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <png++/png.hpp>

using namespace std;

/*-------------------------------------------------------------------------------------*/



/*======================================  MAIN  =======================================*/

int main (int argc, char** argv) {
	VisualOdometryStereo::parameters param;
	

	// calibration parameters for sequence 2010_03_09_drive_0019 
	param.calib.f  = 645.24; // focal length in pixels
	param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
	param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
	param.base     = 0.5707; // baseline in meters

	// init visual odometry
	VisualOdometryStereo viso(param);

	// current pose (this matrix transforms a point from the current
	// frame's camera coordinates to the first frame's camera coordinates)
	Matrix pose = Matrix::eye(4);

	TiObj o_pose;
	o_pose.set("x", 0.0);
	o_pose.set("y", 0.0);
	o_pose.set("th", 0.0);
	
	TiVar& ox = o_pose["x"];
	TiVar& oy = o_pose["y"];
	TiVar& oh = o_pose["th"];
	
	string text;
	cv::Mat rgb_left, rgb_right, left, right;
	while( cin.good() ){
		cin >> text;

		TiObj _left(true,"left");
		TiObj _right(true,"right");

		rgb_left  << _left;
		rgb_right << _right;

		cv::cvtColor(rgb_left, left, CV_BGR2GRAY);
		cv::cvtColor(rgb_right, right, CV_BGR2GRAY);
		
		// image dimensions
		int32_t width  = left.cols;
		int32_t height = left.rows;

		// catch image read/write errors here
		try {
			// compute visual odometry
			int32_t dims[] = {width,height,width};
			if (viso.process(left.data,right.data,dims)) {
				pose = pose * Matrix::inv(viso.getMotion());

				// output some statistics
				double num_matches = viso.getNumberOfMatches();
				double num_inliers = viso.getNumberOfInliers();
				//cerr << ", Matches: " << num_matches;
				//cerr << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
				//cerr << pose << endl << endl;
				
				ox = pose.val[0][3];
				oy = pose.val[2][3];
				oh = pose.val[0][2];
				
				o_pose.save("odom", true);
			} else {
				cerr << " ... failed!" << endl;
			}

		// catch image read errors here
		} catch (...) {
			cerr << "ERROR: Couldn't read input files!" << endl;
			return 1;
		}
		cv::imshow("left", rgb_left);
		cv::waitKey(20);
	}


	return 0;
}

/*-------------------------------------------------------------------------------------*/

