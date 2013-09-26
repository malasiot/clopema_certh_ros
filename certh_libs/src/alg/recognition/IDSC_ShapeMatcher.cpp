// Class declaration of IDSC_ShapeMatcher
// Objects of this class can be used for matching shapes
// to predefined prototypes

/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/


#include <iostream> 
#include <certh_libs/ShapeMatcher.h>
#include "common_cpp.h"


using namespace std;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


namespace certh_libs {



IDSCmodel::Parameters::Parameters() {

    n_samp         = (int)60;                     // number of contour samples
	n_dist         = (int)3;                      // number of distance bins
   	n_theta        = (int)6;                      // number of angle bins
	bTangentV      = true;                        // if ( true ) then rotation invariant descriptors
	bSmoothCont    = false;                       // if true the contour is smoothed
	
  }



bool IDSCmatching::checkParams(IDSCmodel *m1,IDSCmodel *m2){

	IDSCmodel::Parameters param1,param2;
	param1=m1->params;
	param2=m2->params;

	return param1.n_dist==param2.n_dist && param1.n_theta==param2.n_theta;


}


IDSCmodel::IDSCmodel(const cv::Mat &Mask) {

	IDSCmodel::Parameters param;
	params=param;
	BinMask=Mask;
	setEstClass(-1);
	int sizeD1=params.n_dist*params.n_theta*params.n_samp;
	D1 = new double[sizeD1];
	ComputeDescriptors();

	};

IDSCmodel::IDSCmodel(const cv::Mat &Mask,const Parameters &param) {
	params=param;
	BinMask=Mask;
	setEstClass(-1);
	int sizeD1=params.n_dist*params.n_theta*params.n_samp;
	D1 = new double[sizeD1];
	ComputeDescriptors();
}

IDSCmodel::~IDSCmodel() {

	delete []D1; 
	
}


void IDSCmodel::ComputeDescriptors(){

	vector<cv::Point_<double> > cntr;

	findLargestContour(cntr,BinMask);

	

	double *pX1		= new double[params.n_samp];
	double *pY1		= new double[params.n_samp];

	
	int nRow1	=	BinMask.rows;
	int nCol1	=	BinMask.cols;
	int rc1		=	nRow1*nCol1;

	double *fg_mask1	=	new double[rc1];



	trans_inputs_id(pX1, pY1, fg_mask1,
					 params.n_samp, cntr, BinMask);
	
	vector<cv::Point_<double> > cntr1;
	for(int i=0;i<params.n_samp;i++){
		cv::Point2d cvP;
		cvP.x=pX1[i];
		cvP.y=pY1[i];
		cntr1.push_back(cvP);
	}
	Cnt1=cntr1;

	compute_IDSCdescriptors_C(D1, pX1, pY1,params.n_samp,
						 fg_mask1, nRow1, nCol1, params.n_dist,  params.n_theta,
						   params.bTangentV,  params.bSmoothCont);



	delete []pY1;
	delete []pX1;
	delete []fg_mask1;
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ProtModels::ProtModels(const Prototypes & Prot){ 

	
	int i;
	int n_Prot=Prot.Pmasks.size();
	for (i=0;i<n_Prot;i++){
		IDSCmodel* Tmodel=new IDSCmodel(Prot.Pmasks.at(i));
		Tmodel->setEstClass(Prot.Pclasses.at(i));
		Pmodels.push_back(Tmodel);
	}

}

ProtModels::ProtModels(const Prototypes Prot, IDSCmodel::Parameters param){

	int i;
	int n_Prot=Prot.Pmasks.size();
	for (i=0;i<n_Prot;i++){
		IDSCmodel* Tmodel=new IDSCmodel(Prot.Pmasks.at(i),param);
		Tmodel->setEstClass(Prot.Pclasses.at(i));
		Pmodels.push_back(Tmodel);
	}



}


ProtModels::~ProtModels(){
	int i;
	int n_ProtMod=Pmodels.size();
	for (i=0;i<n_ProtMod;i++){
		IDSCmodel* Tmodel;
		Tmodel=Pmodels.at(i);
		delete Tmodel;
	}
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IDSCmatching::Parameters::Parameters() {
   
	nType          = (int)0;                      // default 0
	thre           = (double)0.6;                 // thre*mean(match_cost) is the cost for mismatches
	n_search       = (int)1;                      // number of points used as step for the matching algorithm

  }

  
IDSCmatching::IDSCmatching( IDSCmodel *m1, IDSCmodel *m2) {
  
	IDSCmatching::Parameters param;
	params=param;
	bool chF=checkParams(m1,m2);
	if (chF==false){
		cout<<"Warning: Tried to match models created using different parameters"<<endl
			<<"query model has been re-computed using prototypes' parameters"<<endl;
		IDSCmodel::Parameters param2=m2->params;
		param2.n_samp=m1->params.n_samp;
		m1=new IDSCmodel(m1->BinMask,param2);		
	}
		int n_V1=m1->params.n_samp;
		int n_V2=m2->params.n_samp;
		n_pt=MAX(n_V1,n_V2);
		Cvec=new double[n_pt];
		match_cost=100000;
		match_cost=Match(m1,m2);
		IDSCmatching::MatchInfo MatchInfs(n_pt,Cvec,m1->Cnt1,m2->Cnt1);
		MatchInf= MatchInfs;
		if (chF==false)
		delete m1;
	
  }

IDSCmatching::IDSCmatching( IDSCmodel *m1, IDSCmodel *m2, const Parameters &param) {
  
    params=param;
	int n_V1=m1->params.n_samp;
	int n_V2=m2->params.n_samp;
	n_pt=MAX(n_V1,n_V2);
	bool chF=checkParams(m1,m2);
	if (chF==false){
		cout<<"Warning: Tried to match models created using different parameters"<<endl
			<<"query model has been re-computed using prototypes' parameters"<<endl;
		IDSCmodel::Parameters param2=m2->params;
		param2.n_samp=m1->params.n_samp;
		m1=new IDSCmodel(m1->BinMask,param2);		
	}
		Cvec=new double[n_pt];
		match_cost=100000;
		match_cost=Match(m1,m2);
		IDSCmatching::MatchInfo MatchInfs(n_pt,Cvec,m1->Cnt1,m2->Cnt1);
		 MatchInf= MatchInfs;
	if (chF==false)
		delete m1;


  }



IDSCmatching::MatchInfo::MatchInfo(int n_pt,double* Cvec,vector <cv::Point2d> Cnt1,vector <cv::Point2d> Cnt2){

	int Tindx,i;
	int n_pt1=0;

	for (i=0;i<n_pt;i++){
		Tindx=(int)Cvec[i]-1;
		if (Tindx<n_pt){
			id_gd1.push_back(i);
			id_gd2.push_back(Tindx);
			n_pt1++;
		}
	}
	n_match=n_pt1;

	int Tindx1,Tindx2;
	cv::Point2d Tp1;
	cv::Point2d Tp2;

	for (i=0;i<n_pt1;i++){
		Tindx1=id_gd1.at(i);
		Tindx2=id_gd2.at(i);
		if (Tindx1<n_pt && Tindx2<n_pt+1){
			Tp1=Cnt1.at(Tindx1);
			pt_from.push_back(Tp1);
			Tp2=Cnt2.at(Tindx2);
			pt_to.push_back(Tp2);
		}
			
	}

}


double IDSCmatching::Match( IDSCmodel *img, 
                                           IDSCmodel *mod) 
{

	
	int n_V1=img->params.n_samp;
	int n_V2=mod->params.n_samp;

	int n_d1=img->params.n_dist;
	int n_d2=mod->params.n_dist;

	int n_a1=img->params.n_theta;
	int n_a2=mod->params.n_theta;

	int nBin1=n_d1*n_a1;
	int nBin2=n_d2*n_a2;

	if (n_d1==n_d2 && n_a1==n_a2) {

		int nType=params.nType;
		double thre=params.thre;
		int n_search=params.n_search;


		double	*cost_mat= new double[n_V1*n_V2];
		double  *pDist = new double;
	
	

		/* Analyse input data and parameters*/
    
		compu_dist_matrix( img->D1, mod->D1, n_V1, n_V2, 
					   nBin1, nBin2,  n_d1, n_a1, nType,
					   pDist, cost_mat);


	   double *pmatch_cost=&match_cost;
	
	   DPMatching( Cvec, pmatch_cost, cost_mat, thre, n_V1, n_V2, n_V1 , n_search);

	

		delete []cost_mat;
		delete pDist;

		return match_cost;
	}
	else 
		return -1;

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


PrototypesMatcher::PrototypesMatcher(ProtModels *ProtM, IDSCmodel *Qmodel1) : LoadedProtModels(ProtM), Qmodel(Qmodel1) {

	int min_indx;
	cv::Mat Qimage=Qmodel->BinMask;
	QueryImage=Qimage;
	MirrorFlag=false;
	MinMatchCost=100000;
	MaxMatchCost=0;
	estPclass=-1;
	min_indx=MatchQIm2Prot();
	estPclass=LoadedProtModels->Pmodels.at(min_indx)->estClass;	
	

}

PrototypesMatcher::PrototypesMatcher(ProtModels *ProtM, IDSCmodel *Qmodel1, IDSCmatching::Parameters param) : LoadedProtModels(ProtM), Qmodel(Qmodel1) {


	int min_indx;
	cv::Mat Qimage=Qmodel->BinMask;
	QueryImage=Qimage;
	MirrorFlag=false;
	MinMatchCost=100000;
	MaxMatchCost=0;
	estPclass=-1;
	min_indx=MatchQIm2Prot(param);
	estPclass=LoadedProtModels->Pmodels.at(min_indx)->estClass;	

}

int PrototypesMatcher::MatchQIm2Prot(){

int min_indx1;
int min_indx2;
min_indx1=PrototypesMatcher::MatchQIm2Prot(false);
min_indx2=PrototypesMatcher::MatchQIm2Prot(true);
if (MirrorFlag==false)
	return min_indx1;
else
	return min_indx2;

}


int PrototypesMatcher::MatchQIm2Prot(IDSCmatching::Parameters param){

int min_indx1;
int min_indx2;
min_indx1=PrototypesMatcher::MatchQIm2Prot(false,param);
min_indx2=PrototypesMatcher::MatchQIm2Prot(true,param);
if (MirrorFlag==false)
	return min_indx1;
else
	return min_indx2;

}


int PrototypesMatcher::MatchQIm2Prot(bool mirror){

	int i,min_indx=-1;
	int n_mod=LoadedProtModels->Pmodels.size();

	
	if (mirror==false){
		
		for (i=0;i<n_mod;i++){
			IDSCmodel *Tmodel=LoadedProtModels->Pmodels.at(i);
			IDSCmatching Tmatch(Qmodel, Tmodel);
		    if (MinMatchCost>Tmatch.match_cost){
				MinMatchCost=Tmatch.match_cost;
				BestMatchInfo=Tmatch.MatchInf;
				min_indx=i;
				MirrorFlag=false;
			}
			 if (MaxMatchCost<Tmatch.match_cost){
				MaxMatchCost=Tmatch.match_cost;		
				
			}

			MatchCosts.push_back(Tmatch.match_cost);
			MatchInfos.push_back(Tmatch.MatchInf);

		}
	}
	else{
		cv::Mat QueryImageF;
	    cv::flip(QueryImage,QueryImageF,1);
		IDSCmodel::Parameters paramMod=Qmodel->params;
		IDSCmodel *Qmodel2= new IDSCmodel(QueryImageF,paramMod);
		for (i=0;i<n_mod;i++){
			IDSCmodel *Tmodel=LoadedProtModels->Pmodels.at(i);
			IDSCmatching Tmatch(Qmodel2, Tmodel);
		    if (MinMatchCost>Tmatch.match_cost){
				MinMatchCost=Tmatch.match_cost;
				BestMatchInfo=Tmatch.MatchInf;
				min_indx=i;
				MirrorFlag=true;
			}

			 if (MaxMatchCost<Tmatch.match_cost){
				MaxMatchCost=Tmatch.match_cost;		
				
			}

			MatchCostsMir.push_back(Tmatch.match_cost);
			MatchInfosMir.push_back(Tmatch.MatchInf);

		}
	}

	return min_indx;
}
	

	int PrototypesMatcher::MatchQIm2Prot(bool mirror,IDSCmatching::Parameters param){

	int i,min_indx=-1;
	int n_mod=LoadedProtModels->Pmodels.size();

	
	if (mirror==false){
		
		for (i=0;i<n_mod;i++){
			IDSCmodel *Tmodel=LoadedProtModels->Pmodels.at(i);
			IDSCmatching Tmatch(Qmodel, Tmodel, param);
		    if (MinMatchCost>Tmatch.match_cost){
				MinMatchCost=Tmatch.match_cost;
				BestMatchInfo=Tmatch.MatchInf;
				min_indx=i;
				MirrorFlag=false;
			}

			 if (MaxMatchCost<Tmatch.match_cost){
				MaxMatchCost=Tmatch.match_cost;		
				
			}

			MatchCosts.push_back(Tmatch.match_cost);
			MatchInfos.push_back(Tmatch.MatchInf);

		}
	}
	else{
		cv::Mat QueryImageF;
	    cv::flip(QueryImage,QueryImageF,1);
		IDSCmodel::Parameters paramMod=Qmodel->params;
		IDSCmodel *Qmodel2= new IDSCmodel(QueryImageF,paramMod);
		for (i=0;i<n_mod;i++){
			IDSCmodel *Tmodel=LoadedProtModels->Pmodels.at(i);
			IDSCmatching Tmatch(Qmodel2, Tmodel, param);
		    if (MinMatchCost>Tmatch.match_cost){
				MinMatchCost=Tmatch.match_cost;
				BestMatchInfo=Tmatch.MatchInf;
				min_indx=i;
				MirrorFlag=true;
			}

			 if (MaxMatchCost<Tmatch.match_cost){
				MaxMatchCost=Tmatch.match_cost;		
				
			}

			MatchCostsMir.push_back(Tmatch.match_cost);
			MatchInfosMir.push_back(Tmatch.MatchInf);

		}
	}
	return min_indx;
}


}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
