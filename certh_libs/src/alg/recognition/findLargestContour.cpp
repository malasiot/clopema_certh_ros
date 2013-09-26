

/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/


#include "common_matlab.h"
#include "common_cpp.h"





void findLargestContour(vector<cv::Point_<double> > &cntrsd,  cv::Mat &mask)
{


		cv::Mat bin2=mask.clone();		
		
		vector< vector<cv::Point> > contours;

		cv::findContours(bin2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		
 
		if(!contours.empty()){
			vector<double> areas(contours.size());
			for(int i = 0; i <(int) contours.size(); i++)
				areas[i] = cv::contourArea(cv::Mat(contours[i]));
			double max;
			cv::Point maxPosition;
			cv::minMaxLoc(cv::Mat(areas),0,&max,0,&maxPosition);
			int c_indx=maxPosition.y;
			vector<cv::Point> cntrs=contours[c_indx];

			int npts1=(int) cntrs.size();
		
						
			cv::Point_<double> pD;
			for (int i=0;i< npts1;i++){

				pD.x=(double) cntrs[i].x;
				pD.y=(double) cntrs[i].y;
				cntrsd.push_back(pD);
			}
		}
			
			
	
}