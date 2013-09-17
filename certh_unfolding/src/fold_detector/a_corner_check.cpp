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

struct corner_depths{
	vector<float> average;
	vector<float> averagex;
	vector<float> averagey;
	bool layers3;
};

struct a_corners{
	vector<int> certain_c;
	vector<int> str_l_c;
	vector<int> distant_c;
};

int calc_closest_edge_side(vector<int> ,vector<int>);
corner_depths a_corner_depth(Mat ,Mat ,vector<int> );
bool check_depths(float,corner_depths);
vector<int> check_disconnected(vector<vector<int> >,vector<vector<int> >,vector<vector<int> >& ,Mat ,Mat ,vector<vector<int> > );

//prepei:!1) 3 akmes apo to idio junction, 2)na exei toulaxiston 1 akmh pou na "feygei" panw apo to junction,
//3)1 akmh pou na 'feygei' katw apo to junction (eksairesh an eimaste sto pio xamhlo shmeio tou kremomenou royxoy),
//4)thewrw oti h amkh h "pio katheth" xwrizei to epipedo se 2 hmiepipeda-prepei oi 2 alles akmes na briskontai sto idio
//hmiepipedo,5) h pio "katheth akmh" na kateythynetai pros to hanging point, 6) sth meria tou hmiepipedou pou oristhke,
//to "katw" epipedo na einai pio bathia apo to "panw",7)h gwnia panw kai katw akmhs na einai mikroterh apo 180 moires kai
//oxi konta sto orio

a_corners a_corner_check(vector<vector<int> > junctions,vector<vector<int> > edges,vector<vector<int> > edges_of_junct,Mat im,Mat imd,vector<vector<int> > table){

	vector<int> certain_c;
	vector<int> str_l_c;
	Mat im_cand,impos;
	im.copyTo(impos);
	//im.copyTo(im_cand1);
	cvtColor(im, im_cand, CV_GRAY2BGR);
	//////////////////////////////////////////
	//to ind deixnei poio junction syndeei 3 akmes
	//an den yparxei tetoio junction einai -1
	int ind=-1;
	int count;
	bool a_c_detected=false;
	for (int i=0;i<edges_of_junct.size();i++){
		count=0;
			
		while (edges_of_junct.at(i).at(count)!=-90 && count<edges_of_junct.at(0).size()){
			count++;
		}

		// "T" junction me 3 akmes
		if (count==3 && edges_of_junct.at(i).at(1)==edges_of_junct.at(i).at(2)){
			ind=i;
			//zwgrafizw osa junction kanoun 3ada kai apoteloun enwsh eytheias me akmh
			for (int o1=-2;o1<3;o1++){
				for (int o2=-2;o2<3;o2++){
					for (int c=0;c<3;c++){
					im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[c]=255-c*50;
					}
				}
			}
			/*namedWindow("str_line candidate",0);
			imshow("str_line candidate",im_cand);
			waitKey(0);*/
			//upologizw thn klish ths eutheias
			float f1;
			int t1=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(1)).at(0);
			int t2=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(1)).at(1);
			if (t1==0){
				f1=500;
			}
			else{
				f1=abs(atan(float(t2/t1)));
			}
				//bres poia akrh ths mesaias eutheias den einai to junction
				int clt=calc_closest_edge_side(edges.at(edges_of_junct.at(ind).at(0)),junctions.at(ind));
				int ot=0;
				if (clt==0){
					ot=2;
				}
			
			//an h gwnia einai megalyterh apo 45 kai h mesaia akmh na phgainei pros ta panw
			if (f1>3.14/4 && junctions.at(ind).at(1)>edges.at(edges_of_junct.at(ind).at(0)).at(ot+1) ){
				
				//upologise se poio hmiepipedo brisketai h mesaia akmh ths gwnias
				//bres thn eksiswsh ths "anwterhs" eytheias (h eytheia pou apoteleitai apo tis alles 2)
				float sidet;
				
				int par=edges.at(edges_of_junct.at(ind).at(1)).at(0)-edges.at(edges_of_junct.at(ind).at(1)).at(2);
				int ar=edges.at(edges_of_junct.at(ind).at(1)).at(1)-edges.at(edges_of_junct.at(ind).at(1)).at(3);
				float av,bv;
				if (par!=0){
					av=(float)ar/par;
					bv=(float)(-av*edges.at(edges_of_junct.at(ind).at(1)).at(2)+edges.at(edges_of_junct.at(ind).at(1)).at(3));
				}

			
				//an einai entelws katheth
			
				if (par==0){
					//an ta x einai apo thn idia pleura tote 
					//anhkoun sto idio hmiepipedo
					int c=0;
						sidet=(float)(junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(0)).at(ot));
					
					}
				else{
					 //briskw to x pou antistoixei sthn pio "katheth" eytheia gia to y 
					 //ths akrhs pou den antistoixei sto junction ths allhs eutheias 
					 int tt=edges.at(edges_of_junct.at(ind).at(0)).at(ot+1);
					 sidet=(float)((tt-bv)/av)-(float)(edges.at(edges_of_junct.at(ind).at(0)).at(ot));
				}
				
				//elegxw to bathos
				corner_depths rt=a_corner_depth(im,imd,junctions.at(ind));
				bool ok;
				//an den dhmiourgountai 3 kai panw epipeda h an uparxei asyndeth akmh
				if (rt.layers3==false){
					ok=false;
				}
				else{
				//elegxw pws einai moirasmena ta bathh
					ok=check_depths(sidet,rt);
				}
				
				if (ok==true){
					for (int o1=-5;o1<6;o1++){
						for (int o2=-5;o2<6;o2++){
							for (int c=1;c<3;c++){
								im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[c]=0;
							}
							im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[0]=255;
						}
					}
					a_c_detected=true;
					str_l_c.push_back(ind);
				}
			}
		}




		////////////////////////////////////////////////////////////////////////////////////////////
		//otan exoume 3 akmes kai h dyo den anhkoun sthn idia akmh(enwsh eleutherou junction sto meso ths akmhs)
		if (count==3 && edges_of_junct.at(i).at(1)!=edges_of_junct.at(i).at(2)){
			ind=i;
			
			//zwgrafizw osa junction kanoun 3ada kai den apoteloun enwsh eytheias me akmh
			for (int o1=-2;o1<3;o1++){
				for (int o2=-2;o2<3;o2++){
					for(int c=0;c<3;c++){
						im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[c]=255;
					}
				}
			}
			
	
	Mat ims=Mat::zeros(21,21,CV_8UC1);
	Mat ims_d=Mat::zeros(21,21,CV_8UC1);
	Mat im9;
	im.copyTo(im9);
	if (ind!=-1){
		for (int k=-10;k<11;k++){
			for (int k2=-10;k2<11;k2++){
				ims.at<uchar>(k+10,k2+10)=im9.at<uchar>(junctions.at(ind).at(1)+k,junctions.at(ind).at(0)+k2);	
				
			}			
		}
		for (int k=-2;k<3;k++){
			for (int k2=-2;k2<3;k2++){
				im9.at<uchar>(junctions.at(ind).at(1)+k,junctions.at(ind).at(0)+k2)=255;	
			}			
		}
	}
	vector<int> cl(3),o(3);
	


	int count_up=0;//poses akmes einai panw apo to junction
	vector<int> keep_u(3,0);//poies akmes einai panw apo to junction
	for (int i=0;i<3;i++){
		//briskw gia kathe mia apo tis 3 akmes poia pleura ths einai konta sto junction
		cl.at(i)=calc_closest_edge_side(edges.at(edges_of_junct.at(ind).at(i)),junctions.at(ind));
		if (cl.at(i)==0){
			o.at(i)=2;
		}
		else{
			o.at(i)=0;
		}

	
		//bres poies akmes einai panw apo to junction
		if (junctions.at(ind).at(1)>edges.at(edges_of_junct.at(ind).at(i)).at(o.at(i)+1)){
			
			for (int o1=-3;o1<4;o1++){
				for (int o2=-3;o2<4;o2++){
					im9.at<uchar>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)=255;
					im9.at<uchar>(edges.at(edges_of_junct.at(ind).at(i)).at(o.at(i)+1)+o1,edges.at(edges_of_junct.at(ind).at(i)).at(o.at(i))+o2)=255;
				}
			}
			//krata tis akmes pou einai panw apo  to junction
			//keep_u.at(count_up)=edges_of_junct.at(ind).at(i);
			keep_u.at(count_up)=i;
			count_up++;
			
		}
	}

	//an einai 1-2 akmes panw apo to junction
	//bres thn pio "katheth"=h_edge
	int h_edge;
	
	if (count_up==2 || count_up==1){
		int ind_vertical;

			//////oi akmes
	Mat im3;
	im.copyTo(im3);
	for (int oo=0;oo<3;oo++){
		for (int o1=-3;o1<4;o1++){
			for (int o2=-3;o2<4;o2++){
				im3.at<uchar>(edges.at(edges_of_junct.at(ind).at(oo)).at(o.at(oo)+1)+o1,edges.at(edges_of_junct.at(ind).at(oo)).at(o.at(oo))+o2)=255;
			}
		}
	}
	for (int o1=-3;o1<4;o1++){
		for (int o2=-3;o2<4;o2++){
			im3.at<uchar>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)=255;
		}
	}
	/*namedWindow("3lines",0);
	imshow("3lines",im3);*/
	///////

		
		//bres poia akmh einai pio katheth
		float f1,f2,fu;
		if (count_up==1){
			ind_vertical=0;
			int t1=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_u.at(0))).at(o.at(keep_u.at(0)));
			int t2=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_u.at(0))).at(o.at(keep_u.at(0))+1);
			if (t1==0){
				fu=500;
			}
			else{
				fu=abs(atan(float(t2/t1)));
			}
		}
		else{
	
		
		int t1=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_u.at(0))).at(o.at(keep_u.at(0)));
		int t2=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_u.at(0))).at(o.at(keep_u.at(0))+1);
		if (t1==0){
			f1=500;
			//h_edge=edges_of_junct.at(ind).at(keep_u.at(0));
		}
		else{
			f1=abs(atan(float(t2/t1)));
		}
		
		int t3=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_u.at(1))).at(o.at(keep_u.at(1)));
		int t4=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_u.at(1))).at(o.at(keep_u.at(1))+1);
		if (t3==0){
			f2=500;
			//h_edge=edges_of_junct.at(ind).at(keep_u.at(0));
		}
		else{
			f2=abs(atan(float(t4/t3)));
		}
		
		if (f1>f2){
			h_edge=edges_of_junct.at(ind).at(keep_u.at(0));
			ind_vertical=0;
			fu=f1;
		}
		else{
			h_edge=edges_of_junct.at(ind).at(keep_u.at(1));
			ind_vertical=1;
			fu=f2;
		}
		//emfanizw thn anwterh akmh
	/*	Mat imc;
		im.copyTo(imc);
		for (int o1=-3;o1<4;o1++){
			for (int o2=-3;o2<4;o2++){
				imc.at<uchar>(edges.at(h_edge).at(1)+o1,edges.at(h_edge).at(0)+o2)=255;
				imc.at<uchar>(edges.at(h_edge).at(3)+o1,edges.at(h_edge).at(2)+o2)=255;
			}
		}
		cout<<"f1= "<<f1<<" f2= "<<f2<<endl;
		imshow("higher",imc);
		waitKey(0);*/

		}

		//briskw thn "katwterh" akmh
		vector<int> keep_d(2);
		int cd=0,loweredgeh;
		if (count_up==1){
			//bres poies einai oi 2 akmes pou phgainoun pros ta katw
			for(int h=0;h<3;h++){
				if (h!=keep_u.at(ind_vertical)){
					keep_d.at(cd)=h;
					cd++;
				}
			}
			//paromoia diadikasia eyreshs ths klishs opws kai gia to
			//"anwtero" shmeio
			int t1=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_d.at(0))).at(o.at(keep_d.at(0)));
			int t2=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_d.at(0))).at(o.at(keep_d.at(0))+1);
			if (t1==0){
				f1=500;
				//h_edge=edges_of_junct.at(ind).at(keep_u.at(0));
			}
			else{
				f1=abs(atan(float(t2/t1)));
			}
		
			int t3=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_d.at(1))).at(o.at(keep_d.at(1)));
			int t4=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_d.at(1))).at(o.at(keep_d.at(1))+1);
			if (t3==0){
				f2=500;
			
			}
			else{
				f2=abs(atan(float(t4/t3)));
			}
		
			if (f1>f2){
				loweredgeh=keep_d.at(0);
				
			}
			else{
				loweredgeh=keep_d.at(1);
			}
			//
		}
		else{
			for (int h=0;h<3;h++){
				if (h!=keep_u.at(0) && h!=keep_u.at(1)){
					loweredgeh=h;
				}
			}
		}
		 
		//upologizw th gwnia metaksi "anwterhs" kai "katwterhs" akmhs
		float c_up,c_down,y1,x1;
		y1=(float)(edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical))+1)-junctions.at(ind).at(1));
		x1=(float)(edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical)))-junctions.at(ind).at(0));
		c_up=fastAtan2(y1,x1);
		y1=(float)(edges.at(edges_of_junct.at(ind).at(loweredgeh)).at(o.at(loweredgeh)+1)-junctions.at(ind).at(1));
		x1=(float)(edges.at(edges_of_junct.at(ind).at(loweredgeh)).at(o.at(loweredgeh))-junctions.at(ind).at(0));
		c_down=fastAtan2(y1,x1);
		//cout<<endl<<"C_UP-C_DOWN "<<c_up-c_down<<endl;



		bool same_semiplane=false;
		vector<float> side(2);//apo to proshmo fainetai h pleyra apo thn opoia brisketai h akmh
		//elegxw an h "anwterh" akmh phgainei pros to hanging point 
		//if (abs(fu)>3.14/4 && fu*(im.cols/2)>0 && (abs(c_up-c_down)<172) || (abs(c_up-c_down)>187)){
			if (abs(fu)>3.14/4 && fu*(im.cols/2)>0 ){
		//gia na einai a-corner prepei kai oi alles 2 akmes na 
		//briskontai sto idio hmiepipedo ths pio "katheths"
		//ypologizw thn eksiswsh ths pio "katheths" eytheias
		////bres thn eksiswsh ths eytheias
		int par=junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical)));
		int ar=junctions.at(ind).at(1)-edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical))+1);
		float av,bv;
		if (par!=0){
			av=(float)ar/par;
			bv=(float)(-av*edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical)))+edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical))+1));
		}

		
		//an einai entelws katheth
		
		if (par==0){
			
			//an ta x einai apo thn idia pleura tote 
			//anhkoun sto idio hmiepipedo
			// vector<float> side(2);
			 int c=0;
			for (int k=0;k<3;k++){
				if (keep_u.at(ind_vertical)!=k){
					side.at(c)=(float)(junctions.at(ind).at(0)-edges.at(edges_of_junct.at(ind).at(k)).at(o.at(k)));
					c++;
				}
			}
			//idio hmiepipedo
			
			if (side.at(0)*side.at(1)>0){
				same_semiplane=true;
			}

		}
		else{
			
			 //vector<float> side2(2);
			 int c=0;

			 Mat imcheck;
			 im.copyTo(imcheck);
			 //briskw to x pou antistoixei sthn pio "katheth" eytheia gia to y 
			 //ths akrhs pou den antistoixei sto junction ths allhs eutheias 
			 for (int k=0;k<3;k++){
				 if (keep_u.at(ind_vertical)!=k){
					// int tt=edges.at(edges_of_junct.at(ind).at(keep_u.at(ind_vertical))).at(o.at(keep_u.at(ind_vertical))+1);
					  int tt=edges.at(edges_of_junct.at(ind).at(k)).at(o.at(k)+1);
					 side.at(c)=(float)((tt-bv)/av)-(float)(edges.at(edges_of_junct.at(ind).at(k)).at(o.at(k)));
					 c++;
					//cout<<"tt-.."<<(float)((tt-bv)/av)<<" "<<(float)(edges.at(edges_of_junct.at(ind).at(k)).at(o.at(k)))<<endl;
				 }
				
			 }

			 if (side.at(0)*side.at(1)>0){
				 same_semiplane=true;
			 }
		}
	//	cout<<"SIDE2"<<side.at(0)<<" "<<side.at(1)<<endl;
		}//if (abs(fu))>3.14/4...
		else{
			//anaferetai sthn klish kai oxi sto hmiepipedo alla
			//to xrhsimopoiw san apokleismo ypopshfiou
			same_semiplane=false;
		}


	//	cout<<"SIDE"<<side.at(0)<<" "<<side.at(1)<<endl;
		if (same_semiplane==true){
			
		//	cout<<"same semiplane"<<endl;
			corner_depths r=a_corner_depth(im,imd,junctions.at(ind));
			bool ok;
			//elegxw an epistrefei <3 epipeda h an uparxei asyndeth akmh
			if (r.layers3==false){
				ok=false;
			}
			else{
			//elegxw pws einai moirasmena ta barh
			ok=check_depths(side.at(0),r);
			}
			//zwgrafise me kokkino tis sigoures gwnies me prasino tis pithanes
			if(ok==true){
				
				//for (int o1=-5;o1<6;o1++){
					//for (int o2=-5;o2<6;o2++){
						if ((abs(c_up-c_down)<160) || (abs(c_up-c_down)>200)){
							for (int o1=-5;o1<6;o1++){
								for (int o2=-5;o2<6;o2++){
									for (int c=0;c<3;c++){
										im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[c]=0;
									}
									im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[2]=255;
									
								}
							}
							certain_c.push_back(ind);
						}
						else{
							for (int o1=-5;o1<6;o1++){
								for (int o2=-5;o2<6;o2++){
									for (int c=1;c<3;c++){
										im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[c]=0;
									}
								im_cand.at<Vec3b>(junctions.at(ind).at(1)+o1,junctions.at(ind).at(0)+o2)[0]=255;
								}
							}
								str_l_c.push_back(ind);
						}
					//}
				//}
				a_c_detected=true;
			}

		}
		else{
		/*	cout<<"different semiplane"<<endl;
			cout<<"Not ok!!!!!!!"<<endl;*/
		}
		
	}
	

	/*imwrite("imsd.png",ims_d);
	namedWindow("focus",0);
	imshow("focus",ims);
	namedWindow("whole pict",0);
	imshow("whole pict",im9);
	waitKey(0);*/
		}
	//cout<<endl<<"//////////////////"<<endl;
	}
	
	
	//if (a_c_detected==true){
	//	cout<<"detected!";
	//}
	//else{
	//	cout<<"nothing detected";

	//}

	//entopizei a-corners pou h eswterikh akmh htan ligo makria apo th gwnia (eg 3029)
			vector<int> r_a=check_disconnected(junctions,edges,edges_of_junct,impos,imd,table);

		if (r_a.at(0)>=0){//an exei epistrepsei a-corner
			for (int ll=0;ll<r_a.size();ll++){
				for (int o1=-5;o1<6;o1++){
					for (int o2=-5;o2<6;o2++){
						im_cand.at<Vec3b>(junctions.at(r_a.at(ll)).at(1)+o1,junctions.at(r_a.at(ll)).at(0)+o2)[0]=0;
						im_cand.at<Vec3b>(junctions.at(r_a.at(ll)).at(1)+o1,junctions.at(r_a.at(ll)).at(0)+o2)[2]=0;
						im_cand.at<Vec3b>(junctions.at(r_a.at(ll)).at(1)+o1,junctions.at(r_a.at(ll)).at(0)+o2)[1]=255;
					}
				}
			}
		}

	
	/*namedWindow("candidates&winners",0);
	imshow("candidates&winners",im_cand);
*/
	a_corners ret;
	if (certain_c.size()==0){
		certain_c.push_back(-1);
	}
	if (str_l_c.size()==0){
		str_l_c.push_back(-1);
	}
	ret.certain_c=certain_c;
	ret.distant_c=r_a;
	ret.str_l_c=str_l_c;

	return ret;

}
