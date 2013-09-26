/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/



#ifndef COMMON_CPP
#define COMMON_CPP

#include <cv.h>
#include <vector>

using namespace std;


void build_graph_contour(double *E, int &n_E,
						 double *fg_mask, int nRow, int nCol,
						 int *X, int *Y, int n_V,
						 bool	bSmoothCont
						 );	

void bellman_ford_allpair(double *dis_mat, double *ang_mat,
						  double *X, double *Y, int n_V,
						  double *E, int n_E);

void sc_hist_inputs(double* Dist, double* Ang, double* X,
				    double* Y, double* X1, double* X2,
					bool bTangentV, int m, int m1, int dim);

void comp_sc_hist(double* sc_hist, double* X1,int m1,
				  double* X2, int m2, int dim,
				  int n_dist,int n_theta);

void compu_dist_matrix( double *pSC1, double *pSC2, 
					    int nSamp1, int nSamp2, int nBin1, int nBin2,
						int n_dist, int n_theta, int nType,
						double *pDist, double *cost_mat);					   

void DPMatching( double *cvec, double *match_cost, double *A, 
				double thre, int M, int N, int n_start, int n_search);

void innerdist_matching_C(double *cvec, double *match_cost, double *pX1, double *pY1, int n_V1,
						  double *fg_mask1, int nRow1, int nCol1, double *pX2, double *pY2, int n_V2,
	                      int nRow2, int nCol2, double *fg_mask2, int n_dist, int n_theta,
						  bool bTangentV,  bool bSmoothCont, int nType, double thre, int n_search);

bool myDist(double* D, double* X1,int m1, double* X2, int m2, int dim);


void compute_IDSCdescriptors_C(double *D1, double *pX1, double *pY1, int n_V1,
						  double *fg_mask1, int nRow1, int nCol1, int n_dist, int n_theta,
						  bool bTangentV,  bool bSmoothCont);


void uniform_interp(double *XIs, double *YIs, int n_samp, 
					double *Xs,  double *Ys,  int n_pt);

void trans_inputs_id(double  *pX1, double *pY1, double *fg_mask1,
					 int n_samp, vector<cv::Point_<double> > &cntr, cv::Mat &mask);


void findLargestContour(vector<cv::Point_<double> > &cntrsd,  cv::Mat &mask);





#endif
