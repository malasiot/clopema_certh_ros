
// Class declaration of four classes used for recognizing a shape using
// Inner Distance Shape Contexts (IDSC) matching and a set of prototypes

//1.IDSCmodel          -Class used to compute  IDSC descriptors of a shape in a cv::Mat binary image.
//					    No default constructor is defined. Objects are created based on the cv::Mat binary image
//                      used as input. The inner distance shape contexts descriptors of the image are extracted and 
//                      stored in the public member D1, which is a dynamic array of doubles.

//2.ProtModels         -Class used to compute and store IDSC descriptors of the loaded prototypes. No default
//                      constructor is defined. Objects are created based on the Prototypes object  used as 
//			            input. For each prototype in the input an IDSCmodel is created on the heap and a pointer to
//                      this object is pushed into the Pmodels vector member variable.

//3.IDSCmatching       -Class used to perform IDSC matching between two IDSCmodel objects given as inputs. The matching method
//				        checks if the models are using the same set of parameters. In case they don't, a warning is issued and
//                      the first model is re-computed using the parameters of the second model.

//4.PrototypesMatcher  -Class used to perform multiple IDSC matching between a query image model and a ProtModels object.
//                      Matching is performed twice for each prototype, using a mirror image of the query. The class of the
//						prototype presenting the minimum matching cost is assigned to the query image. Additional information
//                      on the best match are stored in the MatchingInf struct member variable.


/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/

#ifndef IDSC_SHAPE_MATCHER
#define IDSC_SHAPE_MATCHER

#include <iostream> 
#include <cv.h>

#include <vector>
#include <certh_libs/ApplicationSettings.h>

namespace certh_libs {

typedef std::vector < cv::Mat > vM; //The binary images are stored in a vector as CV matrices


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Prototypes {
	
	bool Load_Prot;										//Binary flag returned by Load_Prototypes();
    bool Load_Prototypes(vM&, std::vector <int> &, std::string);  //Private method for reading images in the directory given as second argument.
													    //The method stores the prototype images in a vM variable, that is a vector of cv::Mat.
													    //The class assigned to each prototype is stored in a vector of int
  public:

    vM Pmasks;											//Binary images of prototypes
	std::vector <int> Pclasses;								//Class assigned to each prototype
														//Load_Prototypes() supports class numbers from 01 to 99.
	
	Prototypes ();										//if this constructor is used a default location is selected for loading the prototypes
	Prototypes (std::string);							    //constructor overloaded when the directory of the prototypes is provided as a string
	
    
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class IDSCmodel {

    public:

    friend class IDSCmatching ;
	friend class PrototypesMatcher;
	
    struct Parameters {  // these parameters are used by the ComputeDescriptors
						  // method in order to extract IDSC descriptors
    
        Parameters() ;
    
        int   n_samp;         // number of contour points
        int   n_dist;         // number of distance bins
        int   n_theta;        // number of angle bins
        bool  bTangentV;      // if ( true ) then rotation invariant descriptors
        bool  bSmoothCont;    // if ( true ) the contour is smoothed

        void save(ApplicationSettings &sts, const std::string &prefix) const {
            sts.set_value(prefix + ".n_samp", n_samp) ;
            sts.set_value(prefix + ".n_dist", n_dist) ;
            sts.set_value(prefix + ".n_theta", n_theta) ;
            sts.set_value(prefix + ".bTangentV", bTangentV) ;
            sts.set_value(prefix + ".bSmoothCont", bSmoothCont) ;
        }
        void load(ApplicationSettings &sts, const std::string &prefix) {
            n_samp = sts.value(prefix + ".n_samp", n_samp) ;
            n_dist = sts.value(prefix + ".n_dist", n_dist) ;
            n_theta = sts.value(prefix + ".n_theta", n_theta) ;
            bTangentV = sts.value(prefix + ".bTangentV", bTangentV) ;
            bSmoothCont = sts.value(prefix + ".bSmoothCont", bSmoothCont) ;
        }
    } ;

	IDSCmodel(const cv::Mat &Mask); // if this constructor is employed default parameters are used	
    IDSCmodel(const cv::Mat &Mask, const Parameters &param) ;
	~IDSCmodel() ;         // custom destructor is needed to avoid memory leaking by D1

	bool setEstClass(int Nclass){  // method for setting the estimated class of the model's image
		estClass=Nclass;
		return true;
	}

    int getEstClass(){			  // this method returns the estimated class of the model's image
        int Nclass=estClass;
        return Nclass;
    }

    
	void ComputeDescriptors();	  // this method computes the IDSC descriptors using parameters in params

	private:
	Parameters params ;			  // employed parametrers for IDSC descriptor extraction
	cv::Mat BinMask;			  // binary image of the shape
	int estClass;				  // estimated class if query, target class if prototype
	std::vector <cv::Point2d> Cnt1;    // largest contour extracted from the binary image
	double	*D1;                  // the extracted descriptors
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ProtModels {

  public:

    
	std::vector <IDSCmodel*> Pmodels; // this vector holds pointers to the IDSCmodel of each prototype
	
	
	ProtModels(const Prototypes & Prot); // constructor using the default parameters to estimate the IDSCmodel objects
	ProtModels(const Prototypes Prot, IDSCmodel::Parameters param); 
	
	~ProtModels(); // custom destructor is needed because the IDSCmodel objects pointed in Pmodels 
				   // are created by the constructor on the heap
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class IDSCmatching
{
  public:
 
  friend class PrototypesMatcher; // no longer necessary 

  int n_pt;						// number of contour points (if different for the two models the maximum is selected)
  double *Cvec;					// vector returning the matched indices
  double match_cost;            // IDSC matching cost
  struct MatchInfo {			// additional matching information
	 
	  MatchInfo(){}
      MatchInfo(int n_pt,double* Cvec, std::vector <cv::Point2d> Cnt1,std::vector <cv::Point2d> Cnt2); // overloaded constructor using indices in Cvec to match points in contours Cnt1 and Cnt2

	  int n_match;                    // number of matched points         
	  std::vector <int> id_gd1;			  // indices of matched points in Cnt1
	  std::vector <int>  id_gd2;			  // indices of matched points in Cnt2
	  std::vector <cv::Point2d> pt_from;   // actual cv::Points of Cnt1 matched to
	  std::vector <cv::Point2d> pt_to;     // actual cv::Points of Cnt2
  };


  MatchInfo MatchInf;

  struct Parameters {
    
        Parameters() ;
    
	
        int    nType;         // type	:  0 - using the "Hausdorff" distance as in Belongie's paper
                              //           1 - find the minimum distance with respect all possible global
                              //               rotations ( default 0 )
        double thre;          // thre*mean(match_cost) is the cost for mismatches
        int    n_search;      // number of points used as step for the matching algorithm

        void save(ApplicationSettings &sts, const std::string &prefix) const {
            sts.set_value(prefix + ".nType", nType) ;
            sts.set_value(prefix + ".thre", thre) ;
            sts.set_value(prefix + ".n_search", n_search) ;
        }
        void load(ApplicationSettings &sts, const std::string &prefix) {
            nType = sts.value(prefix + ".nType", nType) ;
            thre = sts.value(prefix + ".thre", thre) ;
            n_search = sts.value(prefix + ".n_search", n_search) ;
        }
   
  } ;
    
  


  IDSCmatching(IDSCmodel *m1, IDSCmodel *m2); // if this constructor is employed default matching parameters are used	

  IDSCmatching( IDSCmodel *m1, IDSCmodel *m2, const Parameters &param);


  

  private:
  
  Parameters params ; // matching parameters
  bool checkParams(IDSCmodel *m1,IDSCmodel *m2); // checks if the input IDSCmodel objectss have the same descriptor parameters
  double Match(IDSCmodel *m1, IDSCmodel *m2) ;  // this method performs the actual IDSC matching using the descriptors in the IDSCmodel objects

  
} ;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PrototypesMatcher {

	public:

	ProtModels *LoadedProtModels;			// pointer to a ProtModels object containing the loaded Prototypes
	IDSCmodel *Qmodel;						// pointer to an IDSCmodel object of the Query image
	cv::Mat QueryImage;						// the binary mask used to generate Qmodel
	bool MirrorFlag;						// if true best match is achieved after mirroring QueryImage
	double MinMatchCost;					// the minimum matching cost between Qmodel and LoadedProtModels
	double MaxMatchCost;					// the maximum matching cost between Qmodel and LoadedProtModels
	IDSCmatching::MatchInfo BestMatchInfo;  // additional matching information on best match
	int estPclass;							// estimated class assigned to the QueryImage
	

	PrototypesMatcher(ProtModels *ProtM, IDSCmodel *Qmodel1);    // if this constructor is used default matching parameters are employed
	PrototypesMatcher(ProtModels *ProtM, IDSCmodel *Qmodel1,IDSCmatching::Parameters param);

private:

    std::vector <double> MatchCosts;                      // vector containing all matching costs
    std::vector <double> MatchCostsMir;					 // vector containing all matching costs using the mirror image
    std::vector <IDSCmatching::MatchInfo> MatchInfos;     // vector containing additional matching information for all matches
    std::vector <IDSCmatching::MatchInfo> MatchInfosMir;  // vector containing additional matching information for all matches using the mirror image
	

	int MatchQIm2Prot();											// calls the last two overloaded functions using default matching parameters
	
	int MatchQIm2Prot(IDSCmatching::Parameters param);				// calls the last two overloaded functions using custom matching parameters

	int MatchQIm2Prot(bool mirror);									// performs matching using default matching parameters. If ( mirror == false ) uses
																	// the original query image, otherwise uses the mirror image.

	int MatchQIm2Prot(bool mirror,IDSCmatching::Parameters param);  // performs matching using custom matching parameters. If ( mirror == false ) uses
																	// the original query image, otherwise uses the mirror image.

};

}

#endif

