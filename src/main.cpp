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
                            string& img_folderpath, string& front_folderpath, string& left_folderpath, string& right_folderpath,
                            Mat& rvecFront, Mat& tvecFront, Mat& rvecLF, Mat& tvecLF, Mat& rvecRF, Mat& tvecRF){

    FileStorage fs(path, FileStorage::READ);
    fs["imageSize"] >> imageSize;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeff"] >> distCoeff;
    fs["img-folder"] >> img_folderpath;
    fs["front-radar-folder"] >> front_folderpath;
    fs["lf-radar-folder"] >> left_folderpath;
    fs["rf-radar-folder"] >> right_folderpath;
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
    cout << "lf-radar-folder: " << left_folderpath<< endl;
    cout << "rf-radar-folder: " << right_folderpath<< endl;
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
            string substring = a.substr(pos-19, 13);
            double time_json = stod(substring)/1000.0;   
            filenames_.push_back(time_json);
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

void plot_results(Mat cameraMatrix, Mat distCoeff, Mat img, string target, string outpath, vector<Point3f> &locph, vector<Point3f> &locp0, Mat tVec, Mat rVec, int color){
    vector<Point2f> check_front_image_ptsh;
    vector<Point2f> check_front_image_pts0;    
    // project on to the pixel coordinate
    projectPoints( locph, rVec, tVec, cameraMatrix, distCoeff, check_front_image_ptsh );
    projectPoints( locp0, rVec, tVec, cameraMatrix, distCoeff, check_front_image_pts0 );
    for(int i=0; i<check_front_image_pts0.size() ; i++){
        cout << check_front_image_pts0[i].x << " " <<check_front_image_pts0[i].y << "\n";
        cv::line(img, check_front_image_pts0[i],check_front_image_ptsh[i],Scalar(0,color,255),1,CV_AA);
    }
    std::ostringstream name;
    name << outpath << "/" << target << ".jpg";
    imwrite(name.str(), img);

}


int find_closest( string path_config, string img_folderpath, string radar_folderpath, Mat cameraMatrix, Mat distCoeff, Mat rvec, Mat tvec, string outpath, int color){

    vector<cv::String> img_filenames;
    cv::glob(img_folderpath, img_filenames);
    deque<cv::String> img_filenames_;
    deque<double> img_id;
    vec2deque(img_filenames, img_filenames_);
    get_numerical_img(img_filenames, img_id);

    vector<cv::String> radar_filenames;
    cv::glob(radar_folderpath, radar_filenames);
    deque<cv::String> radar_filenames_;
    deque<double> radar_id;
    vec2deque(radar_filenames, radar_filenames_);
    get_numerical_json(radar_filenames, radar_id);

    for( auto& target : img_id){

        cout.precision(6);
        cout << fixed << "img id: " << target << "\n";
        while(!radar_id.empty()){
            if(fabs(target - radar_id.front()) < 0.01){

                cout.precision(6);
                cout << fixed << "plot img id: " << target << "\n";
                cout.precision(3);
                cout << fixed << "plot radar id: " << radar_id.front() << "\n";
                Mat img = imread( img_filenames_.front(), CV_LOAD_IMAGE_COLOR );
                string json_path = radar_filenames_.front();

                vector<Point3f> locph;
                vector<Point3f> locp0; 
                getJson(json_path, locph, locp0);

                plot_results(cameraMatrix, distCoeff, img, to_string(target), outpath, locph, locp0, tvec, rvec, color);
                img_filenames_.pop_front();
                break;
            }else if(target - radar_id.front() > 0.01){
                cout.precision(3);
                cout << fixed << "poped radar id: " << radar_id.front() << "\n";
                radar_filenames_.pop_front();
                radar_id.pop_front();
            }else{
                img_filenames_.pop_front();
                break;
            }
        }
    }


};


int main(){
    string path_config = "../config.yaml";

    string img_folderpath;    
    string front_folderpath;
    string left_folderpath;
    string right_folderpath;
    Mat rvecFront, tvecFront, rvecLF, tvecLF, rvecRF, tvecRF;


    Size imageSize;
    Mat cameraMatrix;
    Mat distCoeff;
    get_camera_intrinsic(path_config, imageSize, cameraMatrix, distCoeff, 
                            img_folderpath, front_folderpath, left_folderpath, right_folderpath, 
                            rvecFront, tvecFront, rvecLF, tvecLF, rvecRF, tvecRF);

    cout << img_folderpath << endl;
    string outpath = "../results";
    int colorFront = 255;
    find_closest( path_config, img_folderpath, front_folderpath, cameraMatrix, distCoeff, rvecFront, tvecFront, outpath, colorFront);
    find_closest( path_config, img_folderpath, front_folderpath, cameraMatrix, distCoeff, rvecFront, tvecFront, outpath, colorFront);
    return 0;
}
