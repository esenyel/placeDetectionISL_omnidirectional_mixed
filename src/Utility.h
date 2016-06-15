#ifndef UTILITY_H
#define UTILITY_H
#include <opencv2/opencv.hpp>

using namespace cv;

// STATUS OF BASEPOINT
// 0: normal
// 1: uninformative
// 2: incoherent


class BasePoint
{
public:
    uint id;
    float avgVal;
    float varVal;
    float avgLas;
    float varLas;
    cv::Mat invariants;
    cv::Mat invariants_front;
    cv::Mat invariants_front_bar;
    float location_x;
    float location_y;
    int status;
};

class Place
{

public:

    Place();
    Place(int id);
    uint id;
    cv::Mat memberIds;
    std::vector<BasePoint> members;
    Mat memberInvariants;
    Mat meanInvariant;
    void calculateMeanInvariant();

};

class LearnedPlace
{

public:

    LearnedPlace();
    LearnedPlace(int id);
    uint id;
    // This is a Mx1 vector that will store merged place ids
    cv::Mat memberPlaces;
    // This will be MXN matrix instead of Mx1 because multiple places can be merged in one learned place
    cv::Mat memberIds;
   // std::vector <std::vector<BasePoint> > members;
    Mat memberInvariants;
    Mat meanInvariant;
    void calculateMeanInvariant();

};


class TemporalWindow
{
public:
    TemporalWindow();
    bool checkExtensionStatus(uint currentID);
    int id;
    int startPoint;
    int endPoint;
    int tau_w;
    int tau_n;
    std::vector<BasePoint> members;
    std::vector<BasePoint> cohMembers;
    // Holds the value of total incoherency inside the temporal window
    float totalDiff;


};



class PlaceDetector
{
public:

    PlaceDetector();
    PlaceDetector(int tau_w, int tau_n, int tau_p);
    void processImage();

    bool shouldProcess;
    bool isProcessing;
    bool debugMode;
    bool usePreviousMemory;

    std::string previousMemoryPath;
    std::string debugFilePath1;
    std::string debugFilePath2;
    int debugFileNo1;
    int debugFileNo2;

    int tau_w;
    int tau_n;
    int tau_p;
    double tau_e;
    double tau_s;
    double tau_avgdiff;
    double tau_inv;
    double tau_inv_lower;
    double tau_inv_front;
    double tau_inv_front_bar;

    int image_width;
    int image_height;

    int focalLengthPixels;

    double tau_val_mean;
    double tau_val_var;

    int satLower;
    int satUpper;
    int valLower;
    int valUpper;

    int image_cut_size_upper;
    int image_cut_size_lower;

    int angle_offset;
    int image_number;

    int noHarmonics;

    // 1 current image for an omnidirectional image
    cv::Mat currentImage;


    uint image_counter;
    int  twindow_counter;

    Place* currentPlace;
    vector<Place> detectedPlaces;
    vector<Place> detectedPlaces_rev;
    uint placeID;

    std::vector<BasePoint> wholebasepoints;


private:

    TemporalWindow* tempwin;
    int img_counter;
    BasePoint previousBasePoint;
    BasePoint currentBasePoint;
    std::vector<BasePoint> basepointReservoir;

};

class TopologicalMap
{
public:

    std::vector< std::pair<int,int> > connections;

};

class Level
{

public:

    Level();
    ~Level();
    std::vector<int> members;
    std::vector<int> parentNodes;
    double val;
    //  Connection index bize max parent node numarasini veriyor. Bu node bu noktadan bagli
    int connectionIndex;
    // Mean Invariant bize bu level in ortalama invariant vectorunu veriyor
    std::vector<float> meanInvariant;




};



#endif // UTILITY_H
