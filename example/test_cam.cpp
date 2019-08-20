// #include "src/helpers.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/aruco.hpp>

#include "markers.h"
#include "aruco_markers.h"
#include "solver.h"

#include <time.h>
#include <sys/time.h>
#include <unistd.h>

using namespace markers_lib;

ArucoMarkersDetector aruco_detector;
Solver solver;

// Config
string calibration_file = "./log.yml";
string map_url = "./map.txt";
string map_jpeg = "./map.jpg";
int map_jpeg_size = 1000;
int dictinary = 3;
int cam_id = 2;

struct VisionData
{
    Pose pose;
    bool success;
    string ToString();
};

string VisionData::ToString(){
    return string_pose(pose) + " success: " + to_string(success);
}

VisionData process_img(Mat img)
{
    Mat viz;
    img.copyTo(viz);
    VisionData vd;
    Mat objPoints, imgPoints;
    aruco_detector.detect(img, objPoints, imgPoints, viz);
    // aruco_detector.drawViz(viz);

    // cout << objPoints << "img: " << imgPoints << "\n";
    // Pose pose;
    vd.success = solver.solve(objPoints, imgPoints, vd.pose, viz);
    // imshow("Viz", viz);
    return vd;
}

void load_config()
{
    FileStorage fs2("config.yml", FileStorage::READ);
    fs2["map"] >> map_url;
    fs2["calibration_file"] >> calibration_file;

    // fs2["connection_url"] >> connection_url;

    fs2["map_jpeg"] >> map_jpeg;
    fs2["map_jpeg_size"] >> map_jpeg_size;
    fs2["dictinary"] >> dictinary;
    fs2["cam_id"] >> cam_id;
    fs2.release();
}

void init_marker_reg()
{

    solver.load_camera_conf(calibration_file);

    // solver.set_camera_conf(cameraMatrix, distCoeffs);

    aruco_detector.setDictionary(cv::aruco::getPredefinedDictionary(dictinary));

    aruco_detector.loadMap(map_url);
    aruco_detector.genBoard();
    Mat map_img =
        aruco_detector.drawBoard(cv::Size(map_jpeg_size, map_jpeg_size));
    imwrite(map_jpeg, map_img);
}

int main()
{
    load_config();
    init_marker_reg();

    VideoCapture cap(cam_id);
    while (true)
    {
        Mat frame;
        cap >> frame;
        VisionData vd = process_img(frame);
        cout << vd.ToString() << "\n";
        usleep(1000000 / 20);
    }
    cap.release();
}