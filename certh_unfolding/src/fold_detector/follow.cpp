#include<fstream>
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>


using namespace cv;
using namespace std;

struct ret_fol{
	Mat im;
	bool line;
	int winner;
};

ret_fol follow(vector<vector<int> > junctions, int c_i,Mat im2,vector<vector<int> > table){
	
	//c_i current i=disconnected.at(i)
	//elegxw an apo to junction mporw na paw pros 2 kateythynseis
		bool line;
		int ii,jj,count_times,dim=0,orio=4;
		//oi times apo to table pou antistoixoun sta pixel pou brethhkan
		vector<int> pixels_edges(orio);

		jj=junctions.at(c_i).at(0);
		ii=junctions.at(c_i).at(1);
		for (int k=ii-1;k<ii+2;k++){
			for (int l=jj-1;l<jj+2;l++){
				im2.at<uchar>(k,l)=0;
			}
		}
		/*x_pixels.at(0)=jj;
		y_pixels.at(0)=ii;*/
		bool stop=true;
		
		while(stop==true && dim<orio){//oso briskei epomeno pixel
	
		count_times=1;
		stop=false;
		//follow edge
		while (count_times<3 && stop==false){
			
			count_times++;
			for (int i=ii-count_times;i<ii+count_times+1 && stop==false;i++){
				for (int j=jj-count_times;j<jj+count_times+1;j++){
					
					if (im2.at<uchar>(i,j)==255){
					/*	cout<<"ok";
						cout<<" "<<table.at(i).at(j)<<" ";*/

						pixels_edges.at(dim)=table.at(i).at(j);

						//krataw syntatagmenes
						dim++;
						/*x_pixels.at(dim)=j;
						y_pixels.at(dim)=i;*/
						ii=i;
						jj=j;
						
					//	cout<<" "<<j;
					//diagrafw ta pixel sthn perioxh gia na mhn epistrepsei se ayta
						for (int k=ii-1;k<ii+2;k++){
							for (int l=jj-1;l<jj+2;l++){
								im2.at<uchar>(k,l)=0;
							}
						}
				
						stop=true;
						break;
					}
				}
			}
		}//while
		
		}//while stop ==true
		//epistrefei an yparxei h oxi akmh
		
		if (dim==orio){
			line=true;
		}
		else{
			line=false;
		}
		//ginetai "pshfoforia" kai lew poia
		//akmh entopisa
		vector<int> n_edge (orio);
		vector<int> votes(orio,0);
		n_edge.at(0)=pixels_edges.at(0);
		votes.at(0)++;
		int c=1;
		bool f;
		for (int i=1;i<orio;i++){
			f=false;
			for (int j=0;j<c;j++){
				
				if (pixels_edges.at(i)==n_edge.at(j)){
					votes.at(j)++;
					f=true;
				}
			}
			if (f==false){
				
				n_edge.at(c)=pixels_edges.at(i);
				
				votes.at(c)++;
				c++;
			}
		}
		
		int winner=n_edge.at(0);
		int max=votes.at(0);
		for (int i=0;i<n_edge.size();i++){
			if (max<votes.at(i)){
				max=votes.at(i);
				winner=n_edge.at(i);
			}
		}
		
	//	cout<<"dim "<<dim<<endl;
		ret_fol ret;
		ret.line=line;
		ret.im=im2;
		ret.winner=winner;
		return ret;
}
