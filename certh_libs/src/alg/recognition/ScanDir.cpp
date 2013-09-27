/**************************************************************
Author	:	Mariolis Ioannis,  ymariolis@iti.gr
				Information Technologies Institute,
				Centre of Research and Technology Hellas,
				Thessaloniki, Greece

Date	:		15 February 2013

***************************************************************/

#ifdef _WIN32
#include "dirent.h"
#else
#include <dirent.h>
#endif

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>

using namespace std;



int DirTXT(vector<string> &fileNamesFull, vector<string> &fileNames, char*  dirName){
DIR *dir;

struct dirent *ent;
    dir = opendir (dirName);
    string dot1,dot2;
    dot1=".";
    dot2="..";
if (dir != NULL) {

  /* print all the files and directories within directory */
  while ((ent = readdir (dir)) != NULL) {
    //printf ("%s\n", ent->d_name);



        string temp;

        temp.append(ent->d_name);
        if (temp.compare(dot1)!=0 && temp.compare(dot2)!=0 ){
        string temp1;
        temp1=dirName;
        temp1.append(ent->d_name);
        fileNames.push_back(ent->d_name);
        fileNamesFull.push_back(temp1);
      }

	  

  }
  closedir (dir);
} else {
  /* could not open directory */
 // perror ("");
  return 1;
}
return 0;
}

