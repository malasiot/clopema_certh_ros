
// This is an example demonstrating the use of IDSC_ShapeMatching library
// In this example 8 prototypes are loaded by a default location and one 
// of them is selected as a query image. The selected prototype is matched
// to all prototypes and the best match is returned in Pmatch.
// Finally, the target class of the selected prototype and the estimated
// class assigned after matching are displayed in a console window.


/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/


#include "Prototypes.h"			// class used to laod the prototypes
#include "IDSC_ShapeMatcher.h"  // classes used to perform IDSC shape matching and classification
#include <iostream>








int main(){
	cv::Mat img1;

	IDSCmodel::Parameters param; // this declaration invokes the constructor of IDSCmodel::Parameters struct
								 // setting the descriptor parameters to the default values
	param.n_dist=5;				 // for demonstration purposes a non default value is set to the number of distance bins



    Prototypes MyProt;			// the constructor of Prototypes class loads the prototypes stored in the default location
							    // if the user wants to load her own set of prototypes she can use 
							    // string MyLocation="MyFolder/"; Prototypes MyProt( MyLocation );

	ProtModels *myPmodels= new ProtModels(MyProt,param);    // a ProtModels object is constucted on the heap and a pointer is assigned
															// *myPmodels contains pointers to the IDSCmodels of the prototypes in MyProt
															// the models have been constructed using the parameters in param

	img1=MyProt.Pmasks.at(2);								// for demonstration purposes one of the prototypes is used as a query image
	
	
	
	IDSCmodel *m1= new IDSCmodel(img1,param);				// the corresponding IDSCmodel is constucted for the query image using the same parameters
	
	
	
	IDSCmatching::Parameters paramM;						// this declaration invokes the constructor of IDSCmatching::Parameters struct
															// setting the matching parameters to the default values
	paramM.thre=0.5;										// for demonstration purposes a non default value is set to the threshold employed for mismatch cost
	
	
	PrototypesMatcher Pmatch(myPmodels,m1,paramM);                                          // the constructed ProtModels, IDSCmodel and matching parameters are used by the PrototypesMatcher constructor
																							// in order to perform matching and return the results in Pmatch


    std::cout << "Target Class= " << myPmodels->Pmodels.at(2)->getEstClass() <<std::endl;   // the target class assigned to the prototype selected as query image is displayed in the console window
    std::cout << "estimated Class= " << Pmatch.estPclass<<std::endl;						// the estimated class assigned to the query image after matching is displayed in the console window

	

	return 0;
	
}
