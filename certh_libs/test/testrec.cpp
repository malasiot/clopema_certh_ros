#include <certh_libs/ShapeMatcher.h>  // classes used to perform IDSC shape matching and classification
#include <iostream>

using namespace certh_libs ;
using namespace std ;

int main()
{

    cv::Mat img1;

    char *userName = getenv("USER") ;

    string dataPath = "/home/" ;
    dataPath += userName ;
    dataPath += "/.ros/data/shapematcher/Masks/" ;


    Prototypes MyProt(dataPath);			// the constructor of Prototypes class loads the prototypes stored in the default location
                                // if the user wants to load her own set of prototypes she can use
                                // string MyLocation="MyFolder/"; Prototypes MyProt( MyLocation );

    IDSCmodel::Parameters params ;

    ProtModels *myPmodels= new ProtModels(MyProt, params);    // a ProtModels object is constucted on the heap and a pointer is assigned
                                                            // *myPmodels contains pointers to the IDSCmodels of the prototypes in MyProt
                                                            // the models have been constructed using the parameters in param

    img1=MyProt.Pmasks.at(2);								// for demonstration purposes one of the prototypes is used as a query image



    IDSCmodel *m1= new IDSCmodel(img1, params);				// the corresponding IDSCmodel is constucted for the query image using the same parameters


    IDSCmatching::Parameters mparams ;
    mparams.thre = 0.5 ;

    PrototypesMatcher Pmatch(myPmodels, m1, mparams);                                          // the constructed ProtModels, IDSCmodel and matching parameters are used by the PrototypesMatcher constructor
                                                                                            // in order to perform matching and return the results in Pmatch


    std::cout << "Target Class= " << myPmodels->Pmodels.at(2)->getEstClass() <<std::endl;   // the target class assigned to the prototype selected as query image is displayed in the console window
    std::cout << "Estimated Class= " << Pmatch.estPclass<<std::endl;						// the estimated class assigned to the query image after matching is displayed in the console window



    return 0;

}
