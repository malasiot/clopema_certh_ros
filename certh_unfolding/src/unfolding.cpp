#include "RFLibrary/rf.h"
#include "RFLibrary/rf.cpp"
#include "HFLibrary/hf.h"
#include "HFLibrary/hf.cpp"
#include "Unfold.h"
#include <sstream>
#include <time.h>
#include <camera_helpers/OpenNIServiceClient.h>

using namespace std;

int rf_num_states, rf_num_obs;
RF* rf;

int graspFromFloor(string armName);

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

    ros::init(argc, argv, "unfolding");
    ros::NodeHandle nh;

    system("/home/akargakos/ROS/clopema_certh_ros/certh_pickup/bin/./picking_up");
    ros::Publisher marker_pub;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    system("rosrun camera_helpers openni_service xtion3 &") ;

    ros::Duration(3).sleep() ;
    //cout << "Sleep ended" << endl;

   // graspFromFloor("r2");

    Unfold rb("r2",marker_pub );

//    ofstream ftxt("himg.txt");
//    ofstream frgb("rgb.txt");
//    ofstream fdepth("depth.txt");
//    ofstream flog("log.txt");


    //-------------- Initialization -----------------//
    ifstream fin;
    cout << "Loading rf_obsprob... ";
    fin.open("/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/conf/obsprob.txt");
    fin >> rf_num_states >> rf_num_obs;
    vector < vector < double > >  rf_obsprob;
    rf_obsprob.resize(rf_num_states);
    for(int i=0; i<rf_num_states; ++i){
        rf_obsprob[i].resize(rf_num_obs);
        for(int j=0; j<rf_num_obs; ++j)
            fin >> rf_obsprob[i][j];
    }
    fin.close();
    cout << "DONE" << endl;


    cout << "Loading initial probabilities... ";
    fin.open("/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/conf/initprob.txt");
    vector<double> rf_initprob(rf_num_states, 0);
    for(int i=0; i<rf_num_states; ++i)
        fin >> rf_initprob[i];
    cout << "DONE" << endl;
    fin.close();

    cout << "Loading policy file... ";
    fin.open("/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/conf/out.policy");
    int n;
    fin >> n;
    vector < vector < double > >  rf_vectors;
    vector<int> rf_actions;
    rf_vectors.resize(n);
    rf_actions.resize(n);
    char c[18];
    for(int i=0; i<n; ++i){
        rf_vectors[i].resize(rf_num_states);
        fin.read(c, 18);
        fin >> rf_actions[i];
        fin.read(c, 15);
        for(int j=0; j<rf_num_states; ++j)
            fin >> rf_vectors[i][j];
        fin.read(c, 10);
    }
    fin.close();
    cout << "DONE" << endl;


    cout << "Loading forest... ";
    rf = new RF( "/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/rf_forest" );
    cout << " done." << endl;

    vector<double> rf_belief(rf_num_states, 0);
    for(int i=0; i<rf_num_states; ++i)
        rf_belief[i] = rf_initprob[i];

    vector<double> RFout(rf_num_states, 0);

    cv::namedWindow("depth");
    cv::moveWindow("depth", 0, 0);
    cv::namedWindow("hough");
    cv::moveWindow("hough", 340, 0);
    cv::namedWindow("rgb");
    cv::moveWindow("rgb", 680, 0);


    vector<cv::Mat> dImages(40);
    int dImages_counter = 0;


    time_t start,end;
    int recogn_rotations;
    time(&start);

    //-------------- Planning -----------------//

    bool cloth_unfolded = false;
    cv::Mat depth2, rgb2;
    cv::Rect r;
    pcl::PointCloud<pcl::PointXYZ> pc2;
    while(!cloth_unfolded){
        rb.setClothType(-1);

        while(!rb.graspLowestPoint()){
          //  moveHomeArm(rb.getMovingArm()) ;
        }

        for(int i=0; i<rf_num_states; ++i)
            rf_belief[i] = rf_initprob[i];
        int rf_action = 6;
        recogn_rotations = 0;
        while(rf_action == 6){
            cv::Mat depth, rgb;
            depth = cv::Mat::zeros(640, 480, CV_32FC1);
            pcl::PointCloud<pcl::PointXYZ> pc;
            cout<< "grabbing image "<< endl;
            rb.grabFromXtion(rgb, depth, pc, r);
            pc2 = pc;
            rgb2 = rgb;
            depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            processDepth(depth, depth2, r);
            dImages[dImages_counter++] = depth2.clone();
            int k=0;
            while(k<20){
                imshow("depth", depth2);
                cv::waitKey(1);
                k++;
            }
            //Depth must contain only the cloth (filter background)
            int res = rf->RFDetect(depth2, RFout);
            //flog << "Observation: " << res << endl;
            cout << "Observation: " << res << "  -  ";
            for(int i=0; i<6; ++i)
                cout << RFout[i] << " ";
            cout << endl;

//            cv::Mat log_rgb = rgb2(r);
//            frgb << log_rgb << endl;
//            fdepth << depth2 << endl;

            int prob_bar = floor(RFout[res]/0.2f);
            if(prob_bar > 4) prob_bar = 4;
            int obs = res*5 + prob_bar;

            double denom = 0;
            for(int i=0; i<rf_num_states; ++i){
                rf_belief[i] *= rf_obsprob[i][obs];
                denom += rf_belief[i];
            }
            cout << "Belief: ";
            for(int i=0; i<rf_num_states; ++i){
                rf_belief[i] /= denom;
                cout << "S" << i+1 << ": " << rf_belief[i] << "  ";
                //flog << "S" << i+1 << ": " << rf_belief[i] << "  ";
            }
            cout << endl;
            //flog << endl;

            double max = -10000000000;
            int max_vector = -1;
            for(int i=0; i<n; ++i){
                double v = 0;
                for(int j=0; j<rf_num_states; ++j)
                    v += rf_vectors[i][j]*rf_belief[j];
                if(max < v){
                    max = v;
                    max_vector = i;
                }
            }

            rf_action = rf_actions[max_vector];
            cout << "Action: " << rf_action << endl;
            //flog << "Action: " << rf_action << endl;
            //rf_action = 0;
            if(rf_action==6){
                rb.rotateHoldingGripper(15.0f * M_PI / 180.0f);
                recogn_rotations ++ ;
            }
        }
        rb.setClothType(rf_action);


        //------------------1st Grasping Point-----------------//

        //flog << endl << "Grasp point 1:" << endl << endl;

        stringstream shf;
        shf << "/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/hf" << rf_action;
        HF* hf = new HF( shf.str().c_str());


        //Loading probability files
        cout << "Loading obsprob... ";
        int hf_num_states, hf_num_obs;
        fin.open((shf.str() + "/obsprob.txt").c_str());
        fin >> hf_num_states >> hf_num_obs;
        vector< vector<double> > hf_obsprob;
        hf_obsprob.resize(hf_num_states);
        for(int i=0; i<hf_num_states; ++i){
            hf_obsprob[i].resize(hf_num_obs);
            for(int j=0; j<hf_num_obs; ++j)
                fin >> hf_obsprob[i][j];
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading transprob... ";
        fin.open((shf.str() + "/transprob.txt").c_str());
        vector< vector<double> > hf_transprob;
        hf_transprob.resize(hf_num_states);
        for(int i=0; i<hf_num_states; ++i){
            hf_transprob[i].resize(hf_num_states);
            for(int j=0; j<hf_num_states; ++j)
                fin >> hf_transprob[i][j];
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading policy file... ";
        fin.open((shf.str() + "/out.policy").c_str());
        int hf_num_vectors;
        fin >> hf_num_vectors;
        vector< vector<double> > hf_vectors;
        vector<int> hf_actions(hf_num_vectors);
        hf_vectors.resize(hf_num_vectors);
        char c[18];
        for(int i=0; i<hf_num_vectors; ++i){
            hf_vectors[i].resize(hf_num_states);
            fin.read(c, 18);
            fin >> hf_actions[i];
            fin.read(c, 15);
            for(int j=0; j<hf_num_states; ++j)
                fin >> hf_vectors[i][j];
            fin.read(c, 10);
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading initial probabilities... ";
        fin.open((shf.str() + "/initprob.txt").c_str());
        vector<double> hf_initprob(hf_num_states, 0);
        for(int i=0; i<hf_num_states; ++i)
            fin >> hf_initprob[i];
        cout << "DONE" << endl;
        fin.close();

        vector<double> hf_belief(hf_num_states, 0);
        for(int i=0; i<hf_num_states; ++i)
            hf_belief[i] = hf_initprob[i];
        //-------------------------


        int recogn_rot_angle = (dImages_counter-1)*15;
        int cdI = 0;

        cv::Mat hImg;
        cv::Rect hrect;
        double rot_angle = 0;
        int hf_action = 64;
        bool first_error = true;
        while((hf_action == 64) && (rot_angle<360) ){
            cv::Mat depth, rgb;
            depth = cv::Mat::zeros(640, 480, CV_32FC1);
            pcl::PointCloud<pcl::PointXYZ> pc;
            depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
            hImg = cv::Mat::zeros(530, 260, CV_32FC1);
            int res;
            if(rot_angle > recogn_rot_angle){
                rb.grabFromXtion(rgb, depth, pc, r);
                pc2 = pc;
                rgb2 = rgb;
                processDepth(depth, depth2, r);
                res = hf->houghDetect(depth2, hImg, hrect);
            }
            else
                res = hf->houghDetect(dImages[cdI++], hImg, hrect);

//            cv::Mat log_rgb = rgb2(r);
//            ftxt << hImg << endl;
//            frgb << log_rgb << endl;
//            fdepth << depth2 << endl;

            if(rot_angle > recogn_rot_angle){
                int k=0;
                while(k<20){
                    cv::imshow("depth", depth2);
                    cv::imshow("hough", hImg);
                    cv::waitKey(1);
                    k++;
                }
            }

            cout << "cur_bar: " << res%5 << "  xbar: " << (res/5)%8 << " ybar: " << (res/5)/8 << endl;
            //flog << "cur_bar: " << res%5 << "  xbar: " << (res/5)%8 << " ybar: " << (res/5)/8 << endl;
            cout << "obs: " << res << endl;
            //flog << "obs: " << res << endl;

            vector<double> belief_new(hf_num_states, 0);
            double denom = 0;
            for(int i=0; i<hf_num_states; ++i){
                double sum=0;
                for(int j=0; j<hf_num_states; ++j)
                    sum += hf_transprob[j][i]*hf_belief[j];
                belief_new[i] = hf_obsprob[i][res] * sum;
                denom += belief_new[i];
            }
            for(int i=0; i<hf_num_states; ++i){
                if(denom!=0)
                    hf_belief[i] = belief_new[i]/denom;
                else{
                    if(i!=64)
                        hf_belief[i] = 0;
                    else
                        hf_belief[i] = 1;
                }
            }

            double max = -10000000000000;
            int max_vector = -1;
            for(int i=0; i<hf_num_vectors; ++i){
                double v = 0;
                for(int j=0; j<hf_num_states; ++j)
                    v += hf_vectors[i][j]*hf_belief[j];
                if(max < v){
                    max = v;
                    max_vector = i;
                }
            }

            hf_action = hf_actions[max_vector];
            if(res%5 >= 3) hf_action = -1;
            if(res%5 == 0) hf_action = 64;
            cout << "Action: " << hf_action << endl;
            //flog << "Action: " << hf_action << endl;
            if(hf_action==64){
                if(rot_angle > recogn_rot_angle){
                    rb.rotateHoldingGripper(10.0f * 3.14f / 180.0f);
                    rot_angle += 10;
                }
                else
                    rot_angle += 15;

                if(rf_action == 1) sleep(0.5);
            }
            else{
                //re-estimate
                if(rot_angle < recogn_rot_angle){
                    rb.rotateHoldingGripper(double(rot_angle - recogn_rot_angle) / 180.0f * M_PI);                                        
                }
                recogn_rot_angle = -1;
                sleep(5);
                depth = cv::Mat::zeros(640, 480, CV_32FC1);
                rb.grabFromXtion(rgb, depth, pc, r);
                pc2 = pc;
                rgb2 = rgb;
                depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
                processDepth(depth, depth2, r);
                hImg = cv::Mat::zeros(530, 260, CV_32FC1);
                hf->houghDetect(depth2, hImg, hrect);
                //grasp
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
                bool orientLeft = hrect.x + hrect.width/2 > 260/2;

                if(orientLeft)
                    cout << "Grasp Orientation: Left" << endl;
                else
                    cout << "Grasp Orientation: Right" << endl;

                bool grasp_ok;
                bool finish = false;
                if(rf_action==3)
                    finish = true;
                if(rf_action == 4)
                    grasp_ok = rb.graspPoint(pc2, x, y, finish, orientLeft, true);
                else
                    grasp_ok = rb.graspPoint(pc2, x, y, finish, orientLeft);
                if(grasp_ok)
                    break;
                else{
                    if(!first_error){
                        rb.rotateHoldingGripper(10.0f * 3.14f / 180.0f);
                        rot_angle += 10;
                    }
                    else
                        first_error = false;

                    if(rf_action == 1) sleep(0.5);
                    if ((orientLeft && (rb.getHoldingArm() == "r2")) || (!orientLeft && (rb.getHoldingArm() == "r1"))){
                        rot_angle = 0;
                        for(int i=0; i<hf_num_states; ++i)
                            hf_belief[i] = hf_initprob[i];
                    }

                    hf_action = 64;
                }

            }
        }
        if(hf_action==64)
            continue;
        bool finish = false;
        if(rf_action==3)
            finish = true;


        if(finish){
            cloth_unfolded = true;
            continue;
        }


        //---------------2nd Grasping Point----------------//

        //flog << endl << "Grasp point 2:" << endl << endl;

        stringstream shf2;
        shf2 << "/home/akargakos/ROS/clopema_certh_ros/certh_unfolding/src/forests/";
        if(rf_action==0)
            shf2 << "hf02";
        else if(rf_action==1)
            shf2 << "hf12";
        else if(rf_action==2)
            shf2 << "hf3";
        else if((rf_action==4) || (rf_action==5))
            shf2 << "hf42";

        HF* hf2 = new HF( shf2.str().c_str());

        //Loading probability files

        cout << "Loading obsprob... ";
        fin.open((shf2.str() + "/obsprob.txt").c_str());
        fin >> hf_num_states >> hf_num_obs;
        vector< vector<double> > hf2_obsprob;
        hf2_obsprob.resize(hf_num_states);
        for(int i=0; i<hf_num_states; ++i){
            hf2_obsprob[i].resize(hf_num_obs);
            for(int j=0; j<hf_num_obs; ++j)
                fin >> hf2_obsprob[i][j];
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading transprob... ";
        fin.open((shf2.str() + "/transprob.txt").c_str());
        vector< vector<double> > hf2_transprob;
        hf2_transprob.resize(hf_num_states);
        for(int i=0; i<hf_num_states; ++i){
            hf2_transprob[i].resize(hf_num_states);
            for(int j=0; j<hf_num_states; ++j)
                fin >> hf2_transprob[i][j];
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading policy file... ";
        fin.open((shf2.str() + "/out.policy").c_str());
        fin >> hf_num_vectors;
        vector< vector<double> > hf2_vectors;
        vector<int> hf2_actions(hf_num_vectors);
        hf2_vectors.resize(hf_num_vectors);
        for(int i=0; i<hf_num_vectors; ++i){
            hf2_vectors[i].resize(hf_num_states);
            fin.read(c, 18);
            fin >> hf2_actions[i];
            fin.read(c, 15);
            for(int j=0; j<hf_num_states; ++j)
                fin >> hf2_vectors[i][j];
            fin.read(c, 10);
        }
        fin.close();
        cout << "DONE" << endl;

        cout << "Loading initial probabilities... ";
        fin.open((shf2.str() + "/initprob.txt").c_str());
        vector<double> hf2_initprob(hf_num_states, 0);
        for(int i=0; i<hf_num_states; ++i)
            fin >> hf2_initprob[i];
        cout << "DONE" << endl;
        fin.close();

        vector<double> hf2_belief(hf_num_states, 0);
        for(int i=0; i<hf_num_states; ++i)
            hf2_belief[i] = hf2_initprob[i];

        //-------------------


        rot_angle = 0;
        hf_action = 64;
        first_error = true;
        while((hf_action == 64) && (rot_angle<360) ){
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
            int res = hf2->houghDetect(depth2, hImg, hrect);

//            cv::Mat log_rgb = rgb2(r);
//            ftxt << hImg << endl;
//            frgb << log_rgb << endl;
//            fdepth << depth2 << endl;

            int k=0;
            while(k<20){
                cv::imshow("depth", depth2);
                cv::imshow("hough", hImg);
                cv::waitKey(1);
                k++;
            }

            cout << "cur_bar: " << res%5 << "  xbar: " << (res/5)%8 << " ybar: " << (res/5)/8 << endl;
            //flog << "cur_bar: " << res%5 << "  xbar: " << (res/5)%8 << " ybar: " << (res/5)/8 << endl;
            cout << "obs: " << res << endl;
            //flog << "obs: " << res << endl;

            vector<double> belief_new(hf_num_states, 0);
            double denom = 0;
            for(int i=0; i<hf_num_states; ++i){
                double sum=0;
                for(int j=0; j<hf_num_states; ++j)
                    sum += hf2_transprob[j][i]*hf2_belief[j];
                belief_new[i] = hf2_obsprob[i][res] * sum;
                denom += belief_new[i];
            }
            for(int i=0; i<hf_num_states; ++i){
                if(denom!=0)
                    hf2_belief[i] = belief_new[i]/denom;
                else{
                    if(i!=64)
                        hf2_belief[i] = 0;
                    else
                        hf2_belief[i] = 1;
                }
            }

            double max = -10000000000000;
            int max_vector = -1;
            for(int i=0; i<hf_num_vectors; ++i){
                double v = 0;
                for(int j=0; j<hf_num_states; ++j)
                    v += hf2_vectors[i][j]*hf2_belief[j];
                if(max < v){
                    max = v;
                    max_vector = i;
                }
            }

            hf_action = hf2_actions[max_vector];
            if(res%5 >= 3) hf_action = -1;
            if(res%5 == 0) hf_action = 64;
            cout << "Action: " << hf_action << endl;
            //flog << "Action: " << hf_action << endl;
            if(hf_action==64){
                rb.rotateHoldingGripper(10.0f * 3.14f / 180.0f);
                rot_angle += 10;
            }
            else{
                //re-estimate
                sleep(3);
                depth = cv::Mat::zeros(640, 480, CV_32FC1);
                rb.grabFromXtion(rgb, depth, pc, r);
                pc2 = pc;
                rgb2 = rgb;
                depth2 = cv::Mat::zeros(r.width, r.height, CV_32FC1);
                processDepth(depth, depth2, r);
                hImg = cv::Mat::zeros(r.width, r.height, CV_32FC1);
                hf2->houghDetect(depth2, hImg, hrect);
                //
                int x = r.x + r.width - (hrect.y + hrect.height/2);
                int y = r.y + r.height - (hrect.x + hrect.width/2);
                cv::Point p2(x, y);
                cv::circle(rgb2, p2, 10, cv::Scalar(255, 0, 0), 8);
                k=0;
                while(k<20){
                    cv::imshow("rgb", rgb2);
                    cv::imshow("hough", hImg);
                    cv::waitKey(1);
                    k++;
                }
                bool orientLeft = hrect.x + hrect.width/2 > 260/2;

                if(orientLeft)
                    cout << "Grasp Orientation: Left" << endl;
                else
                    cout << "Grasp Orientation: Right" << endl;

                bool grasp_ok = rb.graspPoint(pc2, x, y, true, orientLeft);
                if(grasp_ok)
                    break;
                else{
                    if(!first_error){
                        rb.rotateHoldingGripper(10.0f * 3.14f / 180.0f);
                        rot_angle += 10;
                    }
                    else
                        first_error = false;

                    if(orientLeft){
                        rot_angle = 0;
                        for(int i=0; i<hf_num_states; ++i)
                            hf2_belief[i] = hf2_initprob[i];
                    }
                    hf_action = 64;
                }
            }
        }
        if(hf_action == 64)
            continue;




        cloth_unfolded = true;
    }

//    ftxt.close();
//    flog.close();
//    fdepth.close();
//    frgb.close();

    time(&end);
    cout << endl << "----------------------" << endl << "unfolding time:  " << difftime(end, start) << "sec" << endl;
    cout << "Recognition rotations: " << recogn_rotations << endl;
    cout << "----------------------" << endl;

    setServoPowerOff(true) ;
       // system("/home/akargakos/ROS/clopema_certh_ros/certh_scripts/./killXtion3.sh");
        //sleep(10) ;
    return 0;


}
