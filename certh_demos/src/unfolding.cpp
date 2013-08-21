#include "RFLibrary/rf.h"
#include "RFLibrary/rf.cpp"
#include "HFLibrary/hf.h"
#include "HFLibrary/hf.cpp"
#include "robot_helpers/Unfold.h"
using namespace std;

int NUM_STATES, NUM_OBS;
RF* rf;

void processDepth(cv::Mat depth, cv::Mat& depth2, cv::Rect r){
    cv::Mat temp, temp2, temp3;
    temp = depth(r);
    cv::flip(temp, temp2, -1);
    temp3 = temp2.t();
    depth2 = cv::Mat::zeros(temp3.rows, temp3.cols, CV_32FC1);
    for(int i=0; i<depth2.rows; ++i){
        for(int j=0; j<depth2.cols; ++j){
            if( (temp3.at<unsigned short>(i,j) < 900) || (temp3.at<unsigned short>(i,j) > 1500))
                depth2.at<float>(i,j) = 0;
            else
                depth2.at<float>(i,j) = (float)temp3.at<unsigned short>(i,j);
        }
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    robot_helpers::Unfold rb("r2",marker_pub );



    //-------------- Initialization -----------------//
    ifstream fin;
    cout << "Loading obsprob... ";
    fin.open("../forests/conf/obsprob.txt");
    fin >> NUM_STATES >> NUM_OBS;
    vector < vector < double > >  obsprob;
    obsprob.resize(NUM_STATES);
    for(int i=0; i<NUM_STATES; ++i){
        obsprob[i].resize(NUM_OBS);
        for(int j=0; j<NUM_OBS; ++j)
            fin >> obsprob[i][j];
    }
    fin.close();
    cout << "DONE" << endl;

    cout << "Loading policy file... ";
    fin.open("../forests/conf/out.policy");
    int n;
    fin >> n;
    vector < vector < double > >  vectors;
    vector<int> actions;
    vectors.resize(n);
    actions.resize(n);
    char c[18];
    for(int i=0; i<n; ++i){
        vectors[i].resize(NUM_STATES);
        fin.read(c, 17);
        fin >> actions[i];
        fin.read(c, 15);
        for(int j=0; j<NUM_STATES; ++j)
            fin >> vectors[i][j];
        fin.read(c, 10);
    }
    fin.close();
    cout << "DONE" << endl;

    cout << "Loading initial probabilities... ";
    fin.open("../forests/conf/initprob.txt");
	vector<double> initprob(NUM_STATES, 0);
	for(int i=0; i<NUM_STATES; ++i)
		fin >> initprob[i];
	cout << "DONE" << endl;
	
	
	cout << "Loading forest... ";
    rf = new RF( "../forests/rf_forest" );
    cout << " done." << endl;

    vector<double> belief(NUM_STATES, 0);
    for(int i=0; i<NUM_STATES; ++i)
        belief[i] = initprob[i];

    vector<double> RFout(NUM_STATES, 0);

    cv::namedWindow("depth");
    cv::moveWindow("depth", 0, 0);
    cv::namedWindow("hough");
    cv::moveWindow("hough", 340, 0);
    cv::namedWindow("rgb");
    cv::moveWindow("rgb", 680, 0);


    //-------------- Planning -----------------//

    bool cloth_unfolded = false;
    cv::Mat depth2, rgb2;
    cv::Rect r;
    pcl::PointCloud<pcl::PointXYZ> pc2;
    while(!cloth_unfolded){
        rb.graspLowestPoint();
        for(int i=0; i<NUM_STATES; ++i)
            belief[i] = initprob[i];
        int action = 6;
        while(action == 6){
            cv::Mat depth, rgb;
            depth = cv::Mat::zeros(640, 480, CV_32FC1);
            pcl::PointCloud<pcl::PointXYZ> pc;
            cout<< "grabbing image "<< endl;
            rb.grabFromXtion(rgb, depth, pc, r);
            pc2 = pc;
            rgb2 = rgb;
            depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            processDepth(depth, depth2, r);
            int k=0;
            while(k<20){
                imshow("depth", depth2);
                cv::waitKey(1);
                k++;
            }
            //Depth must contain only the cloth (filter background)
            int res = rf->RFDetect(depth2, RFout);
            cout << "Observation: " << res << "  -  ";
            for(int i=0; i<6; ++i)
                cout << RFout[i] << " ";
            cout << endl;
            
			int prob_bar = floor(RFout[res]/0.2f);
			if(prob_bar > 4) prob_bar = 4;			
			int obs = res*5 + prob_bar;
			
			double denom = 0;            
            for(int i=0; i<NUM_STATES; ++i){                
                belief[i] *= obsprob[i][obs];
				denom += belief[i];
			}
			for(int i=0; i<NUM_STATES; ++i)
				belief[i] /= denom;

            double max = -10000000000;
            int max_vector = -1;
            for(int i=0; i<n; ++i){
                double v = 0;
                for(int j=0; j<NUM_STATES; ++j)
                    v += vectors[i][j]*belief[j];
                if(max < v){
                    max = v;
                    max_vector = i;
                }
            }

            action = actions[max_vector];            
            cout << "Action: " << action << endl;
            if(action==6){
                rb.rotateHoldingGripper(15.0f * 3.14f / 180.0f);                
            }
        }
        stringstream shf;
        shf << "../forests/hf" << action;
        HF* hf = new HF( shf.str().c_str());
        cv::Mat hImg;
        cv::Rect hrect;
        double res = 0;
        double stdx, stdy;
        double rot_angle = 0;        
        while(rot_angle<360){
            cv::Mat depth, rgb;
            depth = cv::Mat::zeros(640, 480, CV_32FC1);
            pcl::PointCloud<pcl::PointXYZ> pc;
            rb.grabFromXtion(rgb, depth, pc, r);
            pc2 = pc;
            rgb2 = rgb;
            depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            processDepth(depth, depth2, r);
            hImg = cv::Mat::zeros(530, 260, CV_32FC1);
            res = hf->houghDetect(depth2, hImg, hrect, stdx, stdy);

            int k=0;
            while(k<20){
                cv::imshow("depth", depth2);
                cv::imshow("hough", hImg);
                cv::waitKey(1);
                k++;
            }


            string s_res;
            if(res>0.35f)
                s_res = "Visible Grasp Point: YES";
            else if(res==-1)
                s_res = "Visible Grasp Point: AMBIGUOUS";
            else
                s_res = "Visible Grasp Point: NO";
            cout << s_res.c_str() << " res: " << res << endl;
            if(res>0.35f)
                break;
            else{
                rb.rotateHoldingGripper(15.0f * 3.14f / 180.0f);                
                rot_angle += 15;                
            }
        }
        if((res<0.35f) && (rot_angle>=360))
            continue;
        bool finish = false;
        if(action==3)
            finish = true;
        int x,y;
        x = r.x + r.width - (hrect.y + hrect.height/2);
        y = r.y + r.height - (hrect.x + hrect.width/2);
        cv::Point p(x, y);
        cv::circle(rgb2, p, 10, cv::Scalar(255, 0, 0), 8);
        int k=0;
        while(k<20){
            cv::imshow("rgb", rgb2);
            cv::waitKey(1);
            k++;
        }

        if(action == 4)
            rb.graspPoint(pc2, x, y, finish, hrect.x + hrect.width/2 > 260/2, true);
        else
            rb.graspPoint(pc2, x, y, finish, hrect.x + hrect.width/2 > 260/2);

        if(finish){
            cloth_unfolded = true;
            continue;
        }

        stringstream shf2;
        shf2 << "../forests/";
        if(action==0)
            shf2 << "hf02";
        else if(action==1)
            shf2 << "hf12";
        else if(action==2)
            shf2 << "hf3";
        else if((action==4) || (action==5))
            shf2 << "hf42";
        cout << shf2.str().c_str();

        HF* hf2 = new HF( shf2.str().c_str());
        rot_angle = 0;
        while(rot_angle<360){
            cv::Mat depth, rgb;
            pcl::PointCloud<pcl::PointXYZ> pc;
            depth = cv::Mat::zeros(640, 480, CV_32FC1);
            cout<< "grabbing image "<< endl;
            rb.grabFromXtion(rgb, depth, pc, r);
            pc2 = pc;
            rgb2 = rgb;
            depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            processDepth(depth, depth2, r);
            hImg = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            res = hf2->houghDetect(depth2, hImg, hrect, stdx, stdy);

            k=0;
            while(k<20){
                cv::imshow("depth", depth2);
                cv::imshow("hough", hImg);
                cv::waitKey(1);
                k++;
            }

            string s_res;
            if(res>0.35f)
                s_res = "Visible Grasp Point: YES";
            else if(res==-1)
                s_res = "Visible Grasp Point: AMBIGUOUS";
            else
                s_res = "Visible Grasp Point: NO";
            cout << s_res << " " << " res: " << res << endl;
            if(res>0.35f)
                break;
            else{
                rb.rotateHoldingGripper(15.0f * 3.14f / 180.0f);                
                rot_angle += 15;
            }
        }
        if((res<0.35) && (rot_angle>=360))
            continue;
        x = r.x + r.width - (hrect.y + hrect.height/2);
        y = r.y + r.height - (hrect.x + hrect.width/2);
        cv::Point p2(x, y);
        cv::circle(rgb2, p2, 10, cv::Scalar(255, 0, 0), 8);
        k=0;
        while(k<20){
            cv::imshow("rgb", rgb2);
            cv::imshow("hough", hImg);
            cv::waitKey(1);
            k++;
        }

        rb.graspPoint(pc2, x, y, true, hrect.x + hrect.width/2 > 260/2);

        cloth_unfolded = true;
    }


    //Set servo power off
        clopema_motoros::SetPowerOff soff;
        soff.request.force = false;
        ros::service::waitForService("/joint_trajectory_action/set_power_off");
        if (!ros::service::call("/joint_trajectory_action/set_power_off", soff)) {
            ROS_ERROR("Can't call service set_power_off");
            return -1;
        }
    return 0;


}
