
// This is the implementation of the Prototypes class that is used
// to load the binary images used as prototypes for matching


/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/


#include <stdlib.h> //This is needed for atoi function
#include <certh_libs/ShapeMatcher.h>

#include <vector>
#include <highgui.h>

using namespace std;


int DirTXT(vector<string> &fileNamesFull, vector<string> &fileNames, char*  dirName);


namespace certh_libs {




Prototypes::Prototypes() {

    string DirNameMods="/home/clopema/source/clopema/src/alg/recognition/PrototypesIDSC/";
	Load_Prot=Load_Prototypes(Pmasks,Pclasses,DirNameMods);
}


Prototypes::Prototypes(string DirNameMods) {
	
	Load_Prot=Load_Prototypes(Pmasks,Pclasses,DirNameMods);
}




bool Prototypes::Load_Prototypes(vM &Pmasks,vector <int> &Pclasses, string dirNameMods)
{

	// enum pClass {Pants, Shorts, Shirt, Tshirt, Skirt, Towel} Pclass;

	int Pclass;
	char ClassNum[2];

	char* dirNameMod=&dirNameMods[0];
	
	vector<string> fileNamesMod1,fileNamesMod2;

	DirTXT(fileNamesMod1,fileNamesMod2,dirNameMod);

	int NoModF=(int) fileNamesMod1.size();



		for (int j=0;j<NoModF;j++){
			string t1,t2;
			
			t1=fileNamesMod1[j];
			t2=fileNamesMod2[j];
			ClassNum[0]=t2.at(1);
			ClassNum[1]=t2.at(2);
			
			cv::Mat maskP=cv::imread(t1, -1 );
			
			Pmasks.push_back(maskP);

			Pclass=atoi(ClassNum);	
			Pclasses.push_back(Pclass);
			
		}

		return true;
			
}

}
