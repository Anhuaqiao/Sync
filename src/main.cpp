#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <deque>

using namespace std;
using namespace cv;


void get_camera_intrinsic(string path, Size& imageSize, Mat& cameraMatrix, Mat& distCoeff,  
                            string& img_folderpath, string& front_folderpath, string& lf_folderpath, string& rf_folderpath,
                            Mat& rvecFront, Mat& tvecFront, Mat& rvecLF, Mat& tvecLF, Mat& rvecRF, Mat& tvecRF){

    FileStorage fs(path, FileStorage::READ);
    fs["imageSize"] >> imageSize;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeff"] >> distCoeff;
    fs["img-folder"] >> img_folderpath;
    fs["front-radar-folder"] >> front_folderpath;
    fs["lf-radar-folder"] >> lf_folderpath;
    fs["rf-radar-folder"] >> rf_folderpath;
    fs["rvec-front"] >> rvecFront;
    fs["tvec-front"] >> tvecFront;
    fs["rvec-lf"] >> rvecLF;
    fs["tvec-lf"] >> tvecLF;
    fs["rvec-rf"] >> rvecRF;
    fs["tvec-rf"] >> tvecRF;
    fs.release();

    cout << "cameraMatrix: " << cameraMatrix<< endl;
    cout << "distCoeff: " << distCoeff<< endl;
    cout << "img-folder: " << img_folderpath<< endl;
    cout << "front-radar-folder: " << front_folderpath<< endl;
    cout << "lf-radar-folder: " << lf_folderpath<< endl;
    cout << "rf-radar-folder: " << rf_folderpath<< endl;
    cout << "rvec-front: " << rvecFront<< endl;
    cout << "tvec-front: " << tvecFront<< endl;
    cout << "rvec-lf: " << rvecLF<< endl;
    cout << "tvec-lf: " << tvecLF<< endl;
    cout << "rvec-rf: " << rvecRF<< endl;
    cout << "tvec-rf: " << tvecRF<< endl;

}

void get_numerical_img(vector<cv::String> filenames, deque<double>& filenames_){
        for(auto &a:filenames){
            int pos = a.find(".jpg");
            string substring = a.substr(pos-17, 17);
            double time_img = stod(substring);
            filenames_.push_back(time_img);

    }
}

void get_numerical_json(vector<cv::String> filenames, deque<double>& filenames_){
        for(auto &a:filenames){            
            int pos = a.find(".json");
            double time_json;
            if(a.find("_front")!=std::string::npos){
                string substring = a.substr(pos-19, 13);
                time_json = stod(substring)/1000.0;   
                filenames_.push_back(time_json);
            }else if(a.find("_lf")!=std::string::npos){
                string substring = a.substr(pos-16, 13);
                time_json = stod(substring)/1000.0;   
                filenames_.push_back(time_json);
            }else if(a.find("_rf")!=std::string::npos){
                string substring = a.substr(pos-16, 13);
                time_json = stod(substring)/1000.0;   
                filenames_.push_back(time_json);
            }
    }
}


void vec2deque(vector<cv::String> filenames, deque<cv::String>& filenames_){
    for ( auto &a: filenames){
        filenames_.push_back(a);
    }
}

void getJson(string path_json, vector<Point3f> &locph, vector<Point3f> &locp0){
        FileStorage reflects(path_json, FileStorage::READ);
        FileNode objects = reflects["objects"];
        for (FileNodeIterator it= objects.begin(); it!= objects.end();++it){
            if((float)(*it)["objDist"]/100.0 < 20000.0){
                float x = floorf((float)(*it)["objContour"]["x"])/100.0;
                float y = floorf((float)(*it)["objContour"]["y"])/100.0;
                float zh = 2.0;
                float z0 = 0;
                locph.push_back(Point3f(x, y, zh));
                locp0.push_back(Point3f(x, y, z0));
            }
        }
}

void plot_results(Mat cameraMatrix, Mat distCoeff, Mat& img, vector<Point3f> &locph, vector<Point3f> &locp0, Mat rVec, Mat tVec, int color){
    vector<Point2f> check_front_image_ptsh;
    vector<Point2f> check_front_image_pts0;    
    // project on to the pixel coordinate
    projectPoints( locph, rVec, tVec, cameraMatrix, distCoeff, check_front_image_ptsh );
    projectPoints( locp0, rVec, tVec, cameraMatrix, distCoeff, check_front_image_pts0 );
    for(int i=0; i<check_front_image_pts0.size() ; i++){
        cout << check_front_image_pts0[i].x << " " <<check_front_image_pts0[i].y << "\n";
        cv::line(img, check_front_image_pts0[i],check_front_image_ptsh[i],Scalar(0,color,255),1,CV_AA);
    }
}


int find_closest( string path_config, Mat cameraMatrix, Mat distCoeff, string img_folderpath, 
                    string front_folderpath, Mat rvecFront, Mat tvecFront, 
                    string lf_folderpath, Mat rvecLF, Mat tvecLF, 
                    string rf_folderpath, Mat rvecRF, Mat tvecRF, 
                    string outpath, int colorFront, int colorLF, int colorRF){

    vector<cv::String> img_filenames;
    cv::glob(img_folderpath, img_filenames);
    deque<cv::String> img_filenames_;
    deque<double> img_id;
    vec2deque(img_filenames, img_filenames_);
    get_numerical_img(img_filenames, img_id);

    vector<cv::String> front_filenames;
    cv::glob(front_folderpath, front_filenames);
    deque<cv::String> front_filenames_;
    deque<double> front_id;
    vec2deque(front_filenames, front_filenames_);
    get_numerical_json(front_filenames, front_id);

    vector<cv::String> lf_filenames;
    cv::glob(lf_folderpath, lf_filenames);
    deque<cv::String> lf_filenames_;
    deque<double> lf_id;
    vec2deque(lf_filenames, lf_filenames_);
    get_numerical_json(lf_filenames, lf_id);

    vector<cv::String> rf_filenames;
    cv::glob(rf_folderpath, rf_filenames);
    deque<cv::String> rf_filenames_;
    deque<double> rf_id;
    vec2deque(rf_filenames, rf_filenames_);
    get_numerical_json(rf_filenames, rf_id);

    for( auto& target : img_id){

        cout.precision(6);
        cout << fixed << "img id: " << target << "\n";
        while( !front_id.empty()&& !lf_id.empty() && !rf_id.empty() ){
            if((fabs(target - front_id.front()) < 0.01) && (fabs(target - lf_id.front()) < 0.01) && (fabs(target - rf_id.front()) < 0.01)){

                cout.precision(6);
                cout << fixed << "plot img id: " << target << "\n";
                cout.precision(3);
                cout << fixed << "plot front id: " << front_id.front() << "\n";
                cout << fixed << "plot lf id: " << lf_id.front() << "\n";
                cout << fixed << "plot rf id: " << rf_id.front() << "\n";

                Mat img = imread( img_filenames_.front(), CV_LOAD_IMAGE_COLOR );
                string json_path_front = front_filenames_.front();
                string json_path_lf = lf_filenames_.front();
                string json_path_rf = rf_filenames_.front();


                vector<Point3f> locph_front;
                vector<Point3f> locp0_front;
                getJson(json_path_front, locph_front, locp0_front);
                vector<Point3f> locph_lf;
                vector<Point3f> locp0_lf;
                getJson(json_path_lf, locph_lf, locp0_lf);
                vector<Point3f> locph_rf;
                vector<Point3f> locp0_rf;
                getJson(json_path_rf, locph_rf, locp0_rf);


                if( !locph_front.empty() && !locph_lf.empty() && !locph_rf.empty()){
                    plot_results(cameraMatrix, distCoeff, img, locph_front, locp0_front, rvecFront, tvecFront, colorFront);
                    plot_results(cameraMatrix, distCoeff, img, locph_lf, locp0_lf, rvecLF, tvecLF, colorLF);
                    plot_results(cameraMatrix, distCoeff, img, locph_rf, locp0_rf, rvecRF, tvecRF, colorRF);
                }
                
                string str_target = to_string(target);
                std::ostringstream name;
                name << outpath << "/" << str_target << ".jpg";
                imwrite(name.str(), img);
                img_filenames_.pop_front();
                break;
            }else if((target - front_id.front() < -0.01) || (target - lf_id.front() < -0.01) || (target - rf_id.front() < -0.01)){
                img_filenames_.pop_front();
                break;
            }else if((target - front_id.front() > 0.01) && (fabs(target - lf_id.front()) < 0.01) && (fabs(target - rf_id.front()) < 0.01)){
                cout.precision(3);
                cout << fixed << "poped front id: " << front_id.front() << "\n";
                front_filenames_.pop_front();
                front_id.pop_front();
            }else if((fabs(target - front_id.front()) < 0.01) && (target - lf_id.front() > 0.01) && (fabs(target - rf_id.front()) < 0.01)){
                cout.precision(3);
                cout << fixed << "poped lf id: " << lf_id.front() << "\n";
                lf_filenames_.pop_front();
                lf_id.pop_front();
            }else if((fabs(target - front_id.front()) < 0.01) && (fabs(target - lf_id.front()) < 0.01) && (target - rf_id.front() > 0.01)){
                cout.precision(3);
                cout << fixed << "poped rf id: " << rf_id.front() << "\n";
                rf_filenames_.pop_front();
                rf_id.pop_front();
            }else if((target - front_id.front() > 0.01) && (target - lf_id.front() > 0.01) && (fabs(target - rf_id.front()) < 0.01)){
                cout.precision(3);
                cout << fixed << "poped front id: " << front_id.front() << "\n";
                front_filenames_.pop_front();
                front_id.pop_front();
                cout << fixed << "poped lf id: " << lf_id.front() << "\n";
                lf_filenames_.pop_front();
                lf_id.pop_front();
            }else if((target - front_id.front() > 0.01) && (fabs(target - lf_id.front()) < 0.01) && (target - rf_id.front() > 0.01)){
                cout.precision(3);
                cout << fixed << "poped front id: " << front_id.front() << "\n";
                front_filenames_.pop_front();
                front_id.pop_front();
                cout << fixed << "poped rf id: " << rf_id.front() << "\n";
                rf_filenames_.pop_front();
                rf_id.pop_front();
            }else if((fabs(target - front_id.front()) < 0.01)  && (target - lf_id.front() > 0.01)  && (target - rf_id.front() > 0.01)){
                cout.precision(3);
                cout << fixed << "poped lf id: " << lf_id.front() << "\n";
                lf_filenames_.pop_front();
                lf_id.pop_front();
                cout << fixed << "poped rf id: " << rf_id.front() << "\n";
                rf_filenames_.pop_front();
                rf_id.pop_front();
            }else if((target - front_id.front() > 0.01)  && (target - lf_id.front() > 0.01)  && (target - rf_id.front() > 0.01)){
                cout.precision(3);
                cout << fixed << "poped front id: " << front_id.front() << "\n";
                front_filenames_.pop_front();
                front_id.pop_front();
                cout << fixed << "poped lf id: " << lf_id.front() << "\n";
                lf_filenames_.pop_front();
                lf_id.pop_front();
                cout << fixed << "poped rf id: " << rf_id.front() << "\n";
                rf_filenames_.pop_front();
                rf_id.pop_front();
            }
        }
    }
};


int main(){
    string path_config = "../config.yaml";

    string img_folderpath;    
    string front_folderpath;
    string lf_folderpath;
    string rf_folderpath;
    Mat rvecFront, tvecFront, rvecLF, tvecLF, rvecRF, tvecRF;


    Size imageSize;
    Mat cameraMatrix;
    Mat distCoeff;
    get_camera_intrinsic(path_config, imageSize, cameraMatrix, distCoeff, 
                            img_folderpath, front_folderpath, lf_folderpath, rf_folderpath, 
                            rvecFront, tvecFront, rvecLF, tvecLF, rvecRF, tvecRF);

    cout << img_folderpath << endl;
    string outpath = "../results";
    int colorFront = 255;
    int colorLF = 155;
    int colorRF = 0;
    find_closest(path_config, cameraMatrix, distCoeff, img_folderpath, 
                 front_folderpath, rvecFront, tvecFront, 
                 lf_folderpath, rvecLF, tvecLF, 
                 rf_folderpath, rvecRF, tvecRF, 
                 outpath, colorFront, colorLF, colorRF);
    return 0;
}
