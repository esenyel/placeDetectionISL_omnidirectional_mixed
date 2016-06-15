#include "bubble/bubbleprocess.h"
#include "imageprocess/imageprocess.h"
#include "database/databasemanager.h"
#include "Utility.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <QDir>
#include <QDebug>
#include <QDateTime>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>

namespace enc = sensor_msgs::image_encodings;

double compareHistHK( InputArray _H1, InputArray _H2, int method );

double compareHKCHISQR(cv::Mat input1, cv::Mat input2);

// TODO Temporal Window ve basepointleri DB ye kaydedecegiz

int bubble_no=0;

ros::Timer timer;
PlaceDetector detector;


DatabaseManager dbmanager;
std::vector<BasePoint> basepoints;
cv::Mat base_point_locations, base_point_locations1, base_point_locations2;

ros::Publisher placedetectionPublisher;
ros::Publisher filePathPublisher;

bool firsttime = true;

bool sendLastPlaceandShutdown = false;

QString mainDirectoryPath;
QString imagesPath;

// create necessary directories
bool createDirectories(QString previousMemoryPath)
{
    QDir dir(QDir::home());

    QString mainDirectoryName = QDateTime::currentDateTime().toString("yyyy-MM-dd-hh:mm:ss");

    if(!dir.mkdir(mainDirectoryName)) return false;

    dir.cd(mainDirectoryName);

    mainDirectoryPath = dir.path();
    qDebug()<<"Main Directory Path"<<mainDirectoryPath;


    QDir mainDir(QDir::homePath().append("/").append(mainDirectoryName));

    QString imageDirectory = "images";

    if(!mainDir.mkdir(imageDirectory)) return false;

    mainDir.cd(imageDirectory);

    imagesPath = mainDir.path();

    qDebug()<<"Image directory path"<<imagesPath;


    QString databasepath = QDir::homePath();

    databasepath.append("/emptydb");

    QString detecplacesdbpath = databasepath;
    detecplacesdbpath.append("/detected_places.db");

    QFile file(detecplacesdbpath);

    if(file.exists())
    {
        QString newdir = mainDirectoryPath;
        newdir.append("/detected_places.db");
        QFile::copy(detecplacesdbpath,newdir);

        if(!dbmanager.openDB(newdir))
            return false;
        //   file.close();
    }
    else
        return false;

    // If we don't have a previous memory than create an empty memory
    if(previousMemoryPath.size() <= 1 || previousMemoryPath.isNull())
    {
        QString knowledgedbpath = databasepath;
        knowledgedbpath.append("/knowledge.db");

        QFile file2(knowledgedbpath);

        if(file2.exists())
        {
            QString newdir = mainDirectoryPath;
            QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
            // file.close();
        }
        else
            return false;

    }
    // If we have supplied a previous memory path, then open that db
    else
    {
        QString knowledgedbpath = previousMemoryPath;
        knowledgedbpath.append("/knowledge.db");

        QFile file2(knowledgedbpath);

        if(file2.exists())
        {
            QString newdir = mainDirectoryPath;
            QFile::copy(knowledgedbpath,newdir.append("/knowledge.db"));
            // file.close();
        }
        else
            return false;

    }

    return true;

}

bool saveParameters(QString filepath)
{
    QString fullpath = filepath;

    fullpath.append("/PDparams.txt");

    QFile file(fullpath);

    if(file.open(QFile::WriteOnly))
    {
        QTextStream str(&file);

        str<<"tau_w "<<detector.tau_w<<"\n";
        str<<"tau_n "<<detector.tau_n<<"\n";
        str<<"tau_p "<<detector.tau_p<<"\n";
        str<<"tau_e "<<detector.tau_e<<"\n";
        str<<"tau_s "<<detector.tau_s<<"\n";
        str<<"tau_inv "<<detector.tau_inv<<"\n";
        str<<"tau_avgdiff "<<detector.tau_avgdiff<<"\n";
        str<<"focal_length_pixels "<<detector.focalLengthPixels<<"\n";
        str<<"tau_val_mean "<<detector.tau_val_mean<<"\n";
        str<<"tau_val_var "<<detector.tau_val_var<<"\n";
        str<<"sat_lower "<<detector.tau_avgdiff<<"\n";
        str<<"sat_upper "<<detector.focalLengthPixels<<"\n";
        str<<"val_lower "<<detector.tau_val_mean<<"\n";
        str<<"val_upper "<<detector.tau_val_var<<"\n";
        str<<"debug_mode "<<detector.debugMode<<"\n";
        /*
        str<<"debug_filePath "<<QString::fromStdString(detector.debugFilePath)<<"\n";
        str<<"debug_fileNo "<<detector.debugFileNo<<"\n";
        */
        str<<"debug_filePath1 "<<QString::fromStdString(detector.debugFilePath1)<<"\n";
        str<<"debug_fileNo1 "<<detector.debugFileNo1<<"\n";
        str<<"debug_filePath2 "<<QString::fromStdString(detector.debugFilePath2)<<"\n";
        str<<"debug_fileNo2 "<<detector.debugFileNo2<<"\n";

        file.close();
    }
    else
    {
        qDebug()<<"Param File could not be opened for writing!!";
        return false;
    }

    return true;
}

void timerCallback(const ros::TimerEvent& event)
{
    detector.shouldProcess = false;
    detector.processImage();

}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    if(detector.shouldProcess)
    {
        Mat imm1 = cv_bridge::toCvCopy(original_image, enc::BGR8)->image;
        cv::Rect rect(0,0,imm1.cols,(imm1.rows/2));
        detector.currentImage = imm1(rect);//cv_bridge::toCvCopy(original_image, enc::BGR8)->image;

        // detector.shouldProcess = false;

    }

    //cv::imshow("win",image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    //cv::waitKey(3);

}
void startStopCallback(const std_msgs::Int16 startstopSignal)
{

    // Start processing
    if(startstopSignal.data == 1)
    {
        detector.shouldProcess = true;

        if(firsttime)
        {
            firsttime = false;

            bool canCreateDir = false;

            if(!detector.usePreviousMemory)
            {
                canCreateDir = createDirectories("");
            }else
            {
                canCreateDir = createDirectories(QString::fromStdString(detector.previousMemoryPath));
            }

            if(canCreateDir)
            {
                qDebug()<<"Directories have been created successfully!!";

                std_msgs::String sstr;

                sstr.data = mainDirectoryPath.toStdString();

                std::cout<<"Ssstr data: "<<sstr.data<<std::endl;

                saveParameters(mainDirectoryPath);

                filePathPublisher.publish(sstr);
            }
            else
            {
                qDebug()<<"Error!! Necessary directories could not be created!! Detector will not work!!";

                detector.shouldProcess = false;
                //  return -1;
            }


        }


    }
    // Stop the node
    else if(startstopSignal.data == -1)
    {
        sendLastPlaceandShutdown = true;

        //   ros::shutdown();

    }
    // Pause the node
    else if(startstopSignal.data == 0)
    {
        detector.shouldProcess = false;
    }

}

double compareHKCHISQR(Mat input1, Mat input2)
{
    double res = -1;

    if(input1.rows != input2.rows)
    {
        qDebug()<<"Comparison failed due to col size mismatch";
        return res;
    }
    double summ  = 0;
    for(int i = 0; i < input1.rows; i++)
    {
        float in1 = input1.at<float>(i,0);
        float in2 = input2.at<float>(i,0);

        double mul = (in1-in2)*(in1-in2);

        double ss =  in1+in2;
        summ += mul/ss;
        //   qDebug()<<i<<mul<<ss<<summ;

    }

    return summ;
}

// For DEBUGGING: Writing invariants to a file
void writeInvariant(cv::Mat inv, int count)
{
    QString pathh = QDir::homePath();
    pathh.append("/invariants_").append(QString::number(count)).append(".txt");
    QFile file(pathh);

    if(file.open(QFile::WriteOnly))
    {
        QTextStream str(&file);

        for(int i = 0; i < inv.rows; i++)
        {
            str<<inv.at<float>(i,0)<<"\n";

        }

        file.close();
    }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "placeDetectionISL");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    image_transport::ImageTransport it(nh);
    image_transport::TransportHints hints("compressed");

    detector.tau_w = 1;
    detector.tau_n = 1;
    detector.tau_p = 20;
    detector.tau_e = 0.5;
    detector.tau_avgdiff = 0.45;
    detector.tau_s=0.6;
    std::string camera_topic = "";
    int img_width = 640;
    int img_height = 480;
    int image_number, angle_offset;
    detector.focalLengthPixels = 525;

    detector.satLower = 30;
    detector.satUpper = 230;
    detector.valLower = 30;
    detector.valUpper = 230;

    detector.image_cut_size_upper=0;
    detector.image_cut_size_lower=0;


    detector.noHarmonics = 10;
    detector.image_counter = 1;

    detector.shouldProcess = false;
    detector.debugMode = false;
    detector.usePreviousMemory = false;

    std_msgs::String filterPath;

    pnh.getParam("tau_w",detector.tau_w);
    pnh.getParam("tau_n",detector.tau_n);
    pnh.getParam("tau_p",detector.tau_p);
    pnh.getParam("tau_e",detector.tau_e);
    pnh.getParam("tau_inv",detector.tau_inv);
    pnh.getParam("tau_s",detector.tau_s);
    //pnh.getParam("tau_inv_front",detector.tau_inv_front);
    //pnh.getParam("tau_inv_front_bar",detector.tau_inv_front_bar);
    pnh.getParam("tau_avgdiff",detector.tau_avgdiff);
    pnh.getParam("camera_topic",camera_topic);
    //pnh.getParam("image_width",img_width);
    //pnh.getParam("image_height",img_height);
    //pnh.getParam("angle_offset",detector.angle_offset);
    pnh.getParam("image_number",detector.image_number);
    //pnh.getParam("focal_length_pixels",detector.focalLengthPixels);
    pnh.getParam("tau_val_mean",detector.tau_val_mean);
    pnh.getParam("tau_val_var",detector.tau_val_var);
    pnh.getParam("sat_lower",detector.satLower);
    pnh.getParam("sat_upper",detector.satUpper);
    pnh.getParam("val_lower",detector.valLower);
    pnh.getParam("image_cut_size_upper",detector.image_cut_size_upper);
    pnh.getParam("image_cut_size_lower",detector.image_cut_size_lower);

    /***** GET THE DEBUG MODE ****************/

    pnh.getParam("debug_mode",detector.debugMode);

    /*******************************************/

    /*************GET DEBUG FILE PATH ************************/
    std_msgs::String file_path1, file_path2;

    pnh.getParam("file_path1", detector.debugFilePath1);
    pnh.getParam("file_path2", detector.debugFilePath2);

    std::string location_path1, location_path2;

    location_path1 = detector.debugFilePath1;
    location_path1.append("\locations.txt");
    location_path2 = detector.debugFilePath2;
    location_path2.append("\locations.txt");
    std::ifstream fin1, fin2;

    std::cout << "location_path1" << location_path1 << std::endl;
    std::cout << "location_path2" << location_path2 << std::endl;

    fin1.open (location_path1.c_str());


    if ( !fin1 ) exit( 1 );

    /*int row=1832;
    int col=2;



    for ( int i = 0; i < row; i++ )
    {
      for ( int j = 0; j < col; j++ )
      {
          fin >> base_point_locations.at<float>(i,j);
      }
    }*/

    int row1 = 1459;
    int row2 = 1832;
    int col=2;

    base_point_locations1.create(row1,col,CV_32F);

    for ( int i = 0; i < row1; i++ )
    {
      for ( int j = 0; j < col; j++ )
      {
          fin1 >> base_point_locations1.at<float>(i,j);
      }
    }

    fin1.close();

    fin2.open(location_path2.c_str());

    if ( !fin2 ) exit( 1 );

    col=2;

    base_point_locations2.create(row2,col,CV_32F);

    for ( int i = 0; i < row2; i++ )
    {
      for ( int j = 0; j < col; j++ )
      {
          fin2 >> base_point_locations2.at<float>(i,j);

      }
    }

    fin2.close();

    //std::cout << "base point locations1: " << base_point_locations1 << std::endl;
    //std::cout << "base point locations2: " << base_point_locations2 << std::endl;


    // base_point_locations.create(1646,2,CV_32F);

    for (int i=1; i < 1459; i=i++){

        base_point_locations.push_back(base_point_locations1.row(i));
        i++;

    }

    for (int i=1; i < 1832; i++){

        base_point_locations.push_back(base_point_locations2.row(i));
        i++;

    }

    //std::cout << "base_point_locations: " << base_point_locations << std::endl;

    //detector.debugFilePath = file_path.data;

    /******************************************/

    /*** Get the number of files that will be processed**/
    pnh.getParam("file_number1",detector.debugFileNo1);
    pnh.getParam("file_number2",detector.debugFileNo2);
    /***********************************************************/
    qDebug()<<"Saturation and Value thresholds"<<detector.satLower<<detector.satUpper<<detector.valLower<<detector.valUpper<<detector.tau_avgdiff;

    if(detector.debugMode)
    {
        qDebug()<<"Debug mode is on!! File Path"<<QString::fromStdString(detector.debugFilePath1)<<"No of files to be processed"<<detector.debugFileNo1;
    }

    /****** GET THE USE PREVIOUS MEMORY PARAM ****/

    pnh.getParam("use_previous_memory",detector.usePreviousMemory);

    if(detector.usePreviousMemory)
    {
        pnh.getParam("previous_memory_path", detector.previousMemoryPath);
    }

    /********************************************/


    QString basepath = QDir::homePath();
    basepath.append("/visual_filters");

    // QString basepath(filterPath.data.data());

    QString path(basepath);
    int filters[5] = {0,6,12,18,36};

    for (int i=0; i<5; i++){
    path.append("/filtre").append(QString::number(filters[i])).append(".txt");
    qDebug()<<path;

    ImageProcess::readFilter(path,29,false,false,false);

    path.clear();
    path = basepath;
    }


    image_transport::Subscriber imageSub = it.subscribe(camera_topic.data(), 1, imageCallback,hints);

    ros::Subscriber sssub = nh.subscribe("placeDetectionISL/nodecontrol",1, startStopCallback);

    placedetectionPublisher = nh.advertise<std_msgs::Int16>("placeDetectionISL/placeID",5);

    filePathPublisher = nh.advertise<std_msgs::String>("placeDetectionISL/mainFilePath",1,true);

    ros::Rate loop(50);

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();

        if(detector.shouldProcess)
        {
            if(!detector.debugMode)
            {
                detector.shouldProcess = false;
                detector.processImage();

                if(sendLastPlaceandShutdown)
                {
                    if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                    {

                        detector.currentPlace->calculateMeanInvariant();

                        if(detector.currentPlace->memberIds.rows >= detector.tau_p){


                            dbmanager.insertPlace(*detector.currentPlace);

                            detector.detectedPlaces.push_back(*detector.currentPlace);

                            std_msgs::Int16 plID;
                            plID.data = detector.placeID;

                            placedetectionPublisher.publish(plID);

                            ros::spinOnce();

                            detector.placeID++;
                        }

                        delete detector.currentPlace;
                        detector.currentPlace = 0;

                    }

                    ros::shutdown();

                }
            }
            else
            {
                QString processingPerImageFilePath = mainDirectoryPath;

                processingPerImageFilePath = processingPerImageFilePath.append("/pperImage.txt");

                QFile file(processingPerImageFilePath);

                QTextStream strm;

                if(file.open(QFile::WriteOnly))
                {
                    qDebug()<<"Processing per Image file Path has been opened";
                    strm.setDevice(&file);

                }
                // FOR DEBUGGING
                for(int i = 2; i <= detector.debugFileNo1; i++)
                {

                    // for omnidirectional images, the bubble surface is contructed by combinin 5 images
                    //for (int j=1; j<image_number+1; j++){

                    //QString path("/home/hakan/fromJaguars/downloadedItems/jaguarY/2015-01-30-15:39:47/images/");
                    QString path = QString::fromStdString(detector.debugFilePath1);

                    path.append("output").append(QString::number(i)).append(".jpg");

                    Mat imm = imread(path.toStdString().data(),CV_LOAD_IMAGE_COLOR);

                    qint64 starttime = QDateTime::currentMSecsSinceEpoch();

                    cv::Rect rect(0,0,imm.cols,(imm.rows/2));

                    detector.currentImage = imm(rect);

                    detector.processImage();

                    qint64 stoptime = QDateTime::currentMSecsSinceEpoch();

                    qDebug()<<(float)(stoptime-starttime);

                    if(strm.device() != NULL)
                        strm<<(float)(stoptime-starttime)<<"\n";

                    ros::spinOnce();

                    loop.sleep();

                    if(!ros::ok())
                        break;

                    std::cout << "point number: " << i << std::endl;
                    std::cout << "image counter: " << detector.image_counter << std::endl;

                    // qDebug()<<i;
                    //}
                    i++;
                }

                for(int i = 2; i <= detector.debugFileNo2; i++)
                {

                    // for omnidirectional images, the bubble surface is contructed by combinin 5 images
                    //for (int j=1; j<image_number+1; j++){

                    //QString path("/home/hakan/fromJaguars/downloadedItems/jaguarY/2015-01-30-15:39:47/images/");
                    QString path = QString::fromStdString(detector.debugFilePath2);

                    path.append("output").append(QString::number(i)).append(".jpg");

                    Mat imm = imread(path.toStdString().data(),CV_LOAD_IMAGE_COLOR);

                    qint64 starttime = QDateTime::currentMSecsSinceEpoch();

                    cv::Rect rect(0,0,imm.cols,(imm.rows/2));

                    detector.currentImage = imm(rect);

                    detector.processImage();

                    qint64 stoptime = QDateTime::currentMSecsSinceEpoch();

                    qDebug()<<(float)(stoptime-starttime);

                    if(strm.device() != NULL)
                        strm<<(float)(stoptime-starttime)<<"\n";

                    ros::spinOnce();

                    loop.sleep();

                    if(!ros::ok())
                        break;

                    std::cout << "point number: " << i << std::endl;
                    std::cout << "image counter: " << detector.image_counter << std::endl;

                    // qDebug()<<i;
                    //}
                    i++;
                }


                if(detector.currentPlace && detector.currentPlace->id > 0 && detector.currentPlace->members.size() > 0)
                {
                    qDebug()<<"I am here";

                    detector.currentPlace->calculateMeanInvariant();

                    qDebug()<<"Current place mean invariant: "<<detector.currentPlace->meanInvariant.rows<<detector.currentPlace->meanInvariant.cols<<detector.currentPlace->members.size();

                    if(detector.currentPlace->memberIds.rows >= detector.tau_p)
                    {


                        dbmanager.insertPlace(*detector.currentPlace);

                        detector.detectedPlaces.push_back(*detector.currentPlace);

                        std_msgs::Int16 plID;
                        plID.data = detector.placeID;



                        placedetectionPublisher.publish(plID);

                        ros::spinOnce();

                        // loop.sleep();

                        detector.placeID++;
                    }

                    delete detector.currentPlace;
                    detector.currentPlace = 0;
                    detector.shouldProcess = false;

                    ros::shutdown();


                } // end if detector.currentPlace

                file.close();

            } // end else


        } // end if detector.should Process

        //  qDebug()<<"New Place";




    } //  while(ros::ok())


    /// Delete the current place
    if(detector.currentPlace)
    {
        delete detector.currentPlace;
        detector.currentPlace = 0;
    }

    /// Insert basepoints to the database
    // esen comment out

    if(detector.wholebasepoints.size()>0)
        //dbmanager.insertBasePoints(detector.wholebasepoints);

 /*   // comparison of detected places and merging the similar ones under detectedPlaces_rev:


    std::cout << "Comparison of detected places:" << std::endl;
    int place_counter_rev=0;
    std::cout << "number of detected places " << detector.detectedPlaces.size() << std::endl;

     for (int i=0; i< detector.detectedPlaces.size()-1; i++){
        Place merged;
        Place merged_new;

        merged = detector.detectedPlaces.at(i);


        while (i < detector.detectedPlaces.size() ){
            std::cout << "difference between mean invariants: " << i << "," << (i+1) << ": " << norm(detector.detectedPlaces.at(i+1).meanInvariant - detector.detectedPlaces.at(i).meanInvariant ) << std::endl;

            if (norm(detector.detectedPlaces.at(i+1).meanInvariant - detector.detectedPlaces.at(i).meanInvariant ) < detector.tau_s){


                cv::vconcat(merged.memberIds, detector.detectedPlaces.at(i+1).memberIds, merged_new.memberIds);


                for (int j=0; j<merged.members.size(); j++){
                    merged_new.members.push_back(merged.members.at(j));
                }
                for (int j=0; j<detector.detectedPlaces.at(i+1).members.size(); j++){
                    merged_new.members.push_back(detector.detectedPlaces.at(i+1).members.at(j));
                }
                merged=merged_new;
                i++;

            }



            else {
                break;
            }



        }

        merged.calculateMeanInvariant();
        merged.id=place_counter_rev;
        place_counter_rev++;
        std::cout << "place counter" << place_counter_rev << std::endl;

        detector.detectedPlaces_rev.push_back(merged);

    }

     int place_size = detector.detectedPlaces.size();

     if (norm(detector.detectedPlaces.at(place_size-1).meanInvariant - detector.detectedPlaces.at(place_size-2).meanInvariant ) > detector.tau_s){
         detector.detectedPlaces.at(place_size-1).id=place_counter_rev;
         detector.detectedPlaces_rev.push_back(detector.detectedPlaces.at(place_size-1));
     }

     for (int i=0; i< detector.detectedPlaces_rev.size(); i++){

         dbmanager.insertPlace(detector.detectedPlaces_rev.at(i));
     }

    std::vector<TemporalWindow> temporal_windows;
    if (detector.detectedPlaces_rev.at(0).memberIds.at<int>(0,0)!=1){
        TemporalWindow temp_win;
        temp_win.id=1;
        temp_win.startPoint=1;
        temp_win.endPoint=detector.detectedPlaces_rev.at(1).memberIds.at<int>(0,0)-1;
        for (int j=0; j<detector.detectedPlaces_rev.at(1).memberIds.at<int>(0,0); j++){
            temp_win.members.push_back(detector.wholebasepoints.at(j));
        }
        temporal_windows.push_back(temp_win);
    }
    else{
        int size = detector.detectedPlaces_rev.size();

        for(int i=0; i<detector.detectedPlaces_rev.size()-1; i++){

            int size_of_place = detector.detectedPlaces_rev.at(i).members.size();
            TemporalWindow temp_win;
            temp_win.id = i;
            temp_win.startPoint=detector.detectedPlaces_rev.at(i).memberIds.at<int>((size_of_place-1),0)+1;
            temp_win.endPoint=detector.detectedPlaces_rev.at(i+1).memberIds.at<int>(0,0)-1;
            temporal_windows.push_back(temp_win);
        }
        TemporalWindow temp_win_last;
        int size_of_last_place = detector.detectedPlaces_rev.at(detector.detectedPlaces_rev.size()-1).members.size();
        temp_win_last.id = detector.detectedPlaces_rev.size()-1;
        temp_win_last.startPoint = detector.detectedPlaces_rev.at(size-1).memberIds.at<int>((size_of_last_place-1),0)+1;
        temp_win_last.endPoint=detector.wholebasepoints.size();
        temporal_windows.push_back(temp_win_last);

    }

    for (int i=0; i<temporal_windows.size(); i++){

        dbmanager.insertTemporalWindow(temporal_windows.at(i));

    }
    */

    for (int i=0; i<detector.detectedPlaces.size(); i++){

        std::cout << "place ID: " << i << std::endl;
        std::cout << "place members:" << detector.detectedPlaces.at(i).memberIds << std::endl;
    }

    dbmanager.closeDB();

    return 0;

}


void PlaceDetector::processImage()
{
    if(!currentImage.empty())
    {
        timer.stop();

        Mat hueChannel= ImageProcess::generateChannelImage(currentImage,0,satLower,satUpper,valLower,valUpper);
        Mat hueChannelFiltered;
        cv::medianBlur(hueChannel, hueChannelFiltered,3);
        vector<bubblePoint> hueBubble = bubbleProcess::convertOmniGrayImage2BubCustom(hueChannelFiltered,180, image_cut_size_lower, image_cut_size_upper);

        Mat valChannel= ImageProcess::generateChannelImage(currentImage,2,satLower,satUpper,valLower,valUpper);
        vector<bubblePoint> valBubble = bubbleProcess::convertOmniGrayImage2BubCustom(valChannel,255,image_cut_size_lower, image_cut_size_upper);

        vector<bubblePoint> reducedHueBubble = bubbleProcess::reduceBubble(hueBubble);
        vector<bubblePoint> reducedValBubble = bubbleProcess::reduceBubble(valBubble);

        //dbmanager.insertBubble(HUE_TYPE, bubble_no+1,reducedHueBubble);
        //dbmanager.insertBubble(VAL_TYPE, bubble_no+1,reducedValBubble);

        bubbleStatistics statsVal =  bubbleProcess::calculateBubbleStatistics(reducedValBubble,255);

        qDebug()<<"Bubble statistics: "<<statsVal.mean<<statsVal.variance;

        currentBasePoint.avgVal = statsVal.mean;
        currentBasePoint.varVal = statsVal.variance;
        currentBasePoint.id = image_counter;
        currentBasePoint.location_x= base_point_locations.at<float>(image_counter,0);
        currentBasePoint.location_y= base_point_locations.at<float>(image_counter,1);


        QString imagefilePath = imagesPath;
        imagefilePath.append("/rgb_");
        imagefilePath.append(QString::number(image_counter)).append(".jpg");
        imwrite(imagefilePath.toStdString().data(),currentImage);

        //imwrite()
        currentBasePoint.status = 0;

        /*********************** WE CHECK FOR THE UNINFORMATIVENESS OF THE FRAME   *************************/
        if(statsVal.mean <= this->tau_val_mean || statsVal.variance <= this->tau_val_var)
        {

            currentBasePoint.status = 1;

            //  this->shouldProcess = true;


            // If we don't have an initialized window then initialize
            if(!this->tempwin)
            {
                this->tempwin = new TemporalWindow();
                this->tempwin->tau_n = this->tau_n;
                this->tempwin->tau_w = this->tau_w;
                this->tempwin->startPoint = image_counter;
                this->tempwin->endPoint = image_counter;
                this->tempwin->id = twindow_counter;

                this->tempwin->members.push_back(currentBasePoint);

            }
            else
            {

                this->tempwin->endPoint = image_counter;

                this->tempwin->members.push_back(currentBasePoint);

            }


            dbmanager.insertBasePoint(currentBasePoint);

            wholebasepoints.push_back(currentBasePoint);

            //  previousBasePoint = currentBasePoint;

            image_counter++;

            detector.currentImage.release();

            //  timer.start();

            detector.shouldProcess = true;

            return;


        }
        /***********************************  IF THE FRAME IS INFORMATIVE *************************************************/
        else
        {

            Mat totalInvariants;

            qint64 start =  QDateTime::currentMSecsSinceEpoch();

            DFCoefficients dfcoeff = bubbleProcess::calculateDFCoefficients(reducedHueBubble,noHarmonics,noHarmonics);
            Mat hueInvariants = bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics, noHarmonics);

            totalInvariants = hueInvariants.clone();


            cv::Mat logTotal;

            //dbmanager.insertBubble(56, bubble_no+1,hue_bubble_front_bar);

            Mat grayImage;

            cv::cvtColor(currentImage,grayImage,CV_BGR2GRAY);

            std::vector<Mat> sonuc = ImageProcess::applyFilters(grayImage);


            std::cout << "Number of filters:" << sonuc.size() << std::endl;
            for(uint j = 0; j < sonuc.size(); j++)
            {
                // calcuting filter invariants from whole, front and not front part of the omnidirectional images
                vector<bubblePoint> imgBubble = bubbleProcess::convertOmniGrayImage2BubCustom(sonuc[j],255,image_cut_size_lower, image_cut_size_upper);

                vector<bubblePoint> resred = bubbleProcess::reduceBubble(imgBubble);

                DFCoefficients dfcoeff =  bubbleProcess::calculateDFCoefficients(resred,noHarmonics,noHarmonics);

                Mat invariants=  bubbleProcess::calculateInvariantsMat(dfcoeff,noHarmonics,noHarmonics);

                cv::hconcat(totalInvariants, invariants, totalInvariants);

            }

            cv::log(totalInvariants,logTotal);
            logTotal = logTotal/25;
            cv::transpose(logTotal,logTotal);


            qint64 stop = QDateTime::currentMSecsSinceEpoch();

            qDebug()<<"Bubble time"<<(stop-start);
            // TOTAL INVARIANTS N X 1 vector
            for(int kk = 0; kk < logTotal.rows; kk++)
            {
                if(logTotal.at<float>(kk,0) < 0)
                    logTotal.at<float>(kk,0) = 0.5;
            }

            //   qDebug()<<logTotal.rows<<logTotal.cols<<logTotal.at<float>(10,0);

            // We don't have a previous base point
            if(previousBasePoint.id == 0)
            {
                currentBasePoint.id = image_counter;
                currentBasePoint.invariants = logTotal;
                previousBasePoint = currentBasePoint;

                currentPlace->members.push_back(currentBasePoint);

                dbmanager.insertBasePoint(currentBasePoint);
                wholebasepoints.push_back(currentBasePoint);

            }
            else
            {

                currentPlace->calculateMeanInvariant();

                std::cout << "number of base points in the place:" << currentPlace->members.size() << std::endl;

                currentBasePoint.id = image_counter;
                currentBasePoint.invariants = logTotal;


                double result = compareHistHK(currentBasePoint.invariants,previousBasePoint.invariants, CV_COMP_CHISQR);

                std::cout << "result: " << result << std::endl;
                double result_first;

                // if we do not have temporal window, we should compare the current base point with the first one
                //in the detected place


                bool coherency_check;

                // compare the new base point with the previous base point and the first base point of the place
                // if both the previous and first base points are different, start a new place, if they are both similar,
                // stay in the same place. If the new base point is different than previous or first base point, then again
                // consider as new place. If the mean invariant of the detected places are very close, merge these places


                if(!tempwin){

                    result_first=compareHistHK(currentBasePoint.invariants,currentPlace->members.at(0).invariants, CV_COMP_CHISQR);
                    std::cout << "first base point in the place: " << currentPlace->members.at(0).id << std::endl;

                    std::cout << "result_first" << result_first << std::endl;

                    // 0 0
                    if (result > tau_inv && result_first > 5*tau_inv){

                        coherency_check=false;
                        std::cout << "state" << image_counter <<": 0 0" << std::endl;
                        std::cout << "coherency" << coherency_check << std::endl;
                    }

                    // 1 1
                    else if (result <= tau_inv && result_first <= 5*tau_inv && result>0 && result_first>0 ){

                        if (image_counter - currentPlace->memberIds.at<int>(0,0) < tau_p*tau_e){
                            coherency_check=true;
                        }
                        else{
                            coherency_check=false;
                        }

                        coherency_check=true;
                        std::cout << "state" << image_counter << ": 1 1" << std::endl;
                        std::cout << "coherency" << coherency_check << std::endl;
                    }

                    // 0 1
                    else if (result <=tau_inv && result>0 && result_first > 5*tau_inv){

                        coherency_check = false;
                        std::cout << "state" << image_counter << ": 0 1" << std::endl;
                        std::cout << "coherency" << coherency_check << std::endl;
                    }

                    else{

                        coherency_check=false;
                        std::cout << "state" << image_counter << ": 1 0" << std::endl;
                        std::cout << "coherency" << coherency_check << std::endl;
                    }

                }


                else{
                    if(result <=tau_inv && result>0){
                        coherency_check=true;
                    }


                }



                ///////////////////////////// IF THE FRAMES ARE COHERENT ///////////////////////////////////////////////////////////////////////////////////////////////////////
                if(coherency_check)
                {

                    dbmanager.insertBasePoint(currentBasePoint);
                    wholebasepoints.push_back(currentBasePoint);

                    /// If we have a temporal window
                    if(tempwin)
                    {
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {

                            tempwin->cohMembers.push_back(currentBasePoint);


                            basepointReservoir.push_back(currentBasePoint);


                        }
                        // Temporal window will not extend anymore, we should check whether it is really a temporal window or not
                        else
                        {

                            float area = this->tempwin->totalDiff/(tempwin->endPoint - tempwin->startPoint+1);

                            qDebug()<<"Temporal Window Area"<<area;

                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w && area>= tau_avgdiff)
                            {
                                qDebug()<<"New Place";
                                currentPlace->calculateMeanInvariant();

                                qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                                if(currentPlace->memberIds.rows >= tau_p){


                                    dbmanager.insertPlace(*currentPlace);

                                    std_msgs::Int16 plID;
                                    plID.data = this->placeID;

                                    placedetectionPublisher.publish(plID);

                                    this->detectedPlaces.push_back(*currentPlace);

                                    this->placeID++;
                                }


                                delete currentPlace;
                                currentPlace = 0;
                                // this->placeID++;

                                /*    cv::Mat result = DatabaseManager::getPlaceMeanInvariant(this->placeID-1);

                                qDebug()<<"Previous place mean invariant: "<<result.rows<<result.cols<<result.at<float>(50,0);

                                result = DatabaseManager::getPlaceMemberIds(this->placeID-1);

                                for(int k = 0; k< result.rows; k++){

                                    qDebug()<<"Previous place members: "<<result.rows<<result.cols<<result.at<unsigned short>(k,0);
                                }*/

                                currentPlace = new Place(this->placeID);

                                //   currentPlace->


                                basepointReservoir.push_back(currentBasePoint);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();

                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published




                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size() ); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;

                                basepointReservoir.clear();

                            }

                        }

                    }
                    else
                    {
                        currentPlace->members.push_back(currentBasePoint);

                    }

                }
                ///////////////////////// IF THE FRAMES ARE INCOHERENT /////////////////////////////////////
                else
                {
                    currentBasePoint.status = 2;

                    dbmanager.insertBasePoint(currentBasePoint);
                    wholebasepoints.push_back(currentBasePoint);

                    // If we don't have a temporal window create one
                    if(!tempwin)
                    {
                        tempwin = new TemporalWindow();
                        this->tempwin->tau_n = this->tau_n;
                        this->tempwin->tau_w = this->tau_w;
                        this->tempwin->startPoint = image_counter;
                        this->tempwin->endPoint = image_counter;
                        this->tempwin->id = twindow_counter;
                        this->tempwin->totalDiff +=result;
                        this->tempwin->members.push_back(currentBasePoint);

                    }
                    // add the basepoint to the temporal window
                    else
                    {
                        // Temporal window will extend, we are still looking for the next incoming frames
                        if(tempwin->checkExtensionStatus(currentBasePoint.id))
                        {

                            this->tempwin->endPoint = image_counter;

                            this->tempwin->members.push_back(currentBasePoint);

                            this->tempwin->totalDiff +=result;

                            basepointReservoir.clear();

                        }
                        else
                        {
                            float avgdiff;

                            avgdiff = this->tempwin->totalDiff/(tempwin->endPoint - tempwin->startPoint+1);


                            std::cout<<"Temporal Window Average Diff"<< avgdiff << std::endl;

                            // This is a valid temporal window
                            if(tempwin->endPoint - tempwin->startPoint >= tau_w && avgdiff >= tau_avgdiff)
                            {
                                //  float summ = 0;
                                //Modifikasyon
                                /* for(int kl = 0; kl < tempwin->members.size(); kl++)
                                {
                                    BasePoint abasepoint = tempwin->members.at(kl);
                                    abasepoint.

                                }*/

                                qDebug()<<"New Place";
                                currentPlace->calculateMeanInvariant();

                                qDebug()<<"Current place mean invariant: "<<currentPlace->meanInvariant.rows<<currentPlace->meanInvariant.cols<<currentPlace->meanInvariant.at<float>(50,0);

                                if(currentPlace->memberIds.rows >= tau_p)
                                {


                                    dbmanager.insertPlace(*currentPlace);

                                    std_msgs::Int16 plID ;
                                    plID.data = this->placeID;

                                    placedetectionPublisher.publish(plID);


                                    this->detectedPlaces.push_back(*currentPlace);

                                    this->placeID++;
                                }

                                delete currentPlace;
                                currentPlace = 0;

                                currentPlace = new Place(this->placeID);

                                currentPlace->members = basepointReservoir;
                                basepointReservoir.clear();


                                dbmanager.insertTemporalWindow(*tempwin);

                                delete tempwin;
                                tempwin = 0;
                                this->twindow_counter++;
                                // A new place will be created. Current place will be published

                            }
                            // This is just a noisy temporal window. We should add the coherent basepoints to the current place
                            else
                            {
                                //  basepointReservoir.push_back(currentBasePoint);

                                delete tempwin;
                                tempwin = 0;

                                std::vector<BasePoint> AB;
                                AB.reserve( currentPlace->members.size() + basepointReservoir.size() ); // preallocate memory
                                AB.insert( AB.end(), currentPlace->members.begin(), currentPlace->members.end() );
                                AB.insert( AB.end(), basepointReservoir.begin(), basepointReservoir.end() );
                                currentPlace->members.clear();
                                currentPlace->members = AB;

                                basepointReservoir.clear();

                            }


                            tempwin = new TemporalWindow();
                            this->tempwin->tau_n = this->tau_n;
                            this->tempwin->tau_w = this->tau_w;
                            this->tempwin->startPoint = image_counter;
                            this->tempwin->endPoint = image_counter;
                            this->tempwin->id = twindow_counter;

                            this->tempwin->members.push_back(currentBasePoint);


                        }

                    }

                }

                previousBasePoint = currentBasePoint;

                //////////////////////////////////////////////////////////////////////////////////////////////////
            }

            // DatabaseManager::insertInvariants(HUE_TYPE,frameNumber,invariants);
            //   qDebug()<<"Image Counter: "<<image_counter;
            image_counter++;

            //this->shouldProcess = true;

        }

    }

    this->currentImage.release();

    this->shouldProcess = true;

    //  timer.start();
    bubble_no=bubble_no+1;
    std::cout << "loop done" << std::endl;


}
PlaceDetector::PlaceDetector()
{

    this->tempwin = 0;
    this->currentBasePoint.id = 0;
    this->previousBasePoint.id = 0;
    this->placeID = 1;
    currentPlace = new Place(this->placeID);
    this->twindow_counter = 1;

}


bool TemporalWindow::checkExtensionStatus(uint currentID)
{
    if(currentID - this->endPoint <= tau_n)
    {
        return true;
    }

    return false;

}

double compareHistHK( InputArray _H1, InputArray _H2, int method )
{
    Mat H1 = _H1.getMat(), H2 = _H2.getMat();
    const Mat* arrays[] = {&H1, &H2, 0};
    Mat planes[2];
    NAryMatIterator it(arrays, planes);
    double result = 0;
    int j, len = (int)it.size;

    CV_Assert( H1.type() == H2.type() && H1.depth() == CV_32F );

    double s1 = 0, s2 = 0, s11 = 0, s12 = 0, s22 = 0;

    CV_Assert( it.planes[0].isContinuous() && it.planes[1].isContinuous() );

    for( size_t i = 0; i < it.nplanes; i++, ++it )
    {
        const float* h1 = (const float*)it.planes[0].data;
        const float* h2 = (const float*)it.planes[1].data;
        len = it.planes[0].rows*it.planes[0].cols*H1.channels();


        for( j = 0; j < len; j++ )
        {
            double a = h1[j] - h2[j];
            double b =  h1[j] + h2[j];
            if( fabs(b) > DBL_EPSILON )
                result += a*a/b;
        }

    }

    return result;

}
