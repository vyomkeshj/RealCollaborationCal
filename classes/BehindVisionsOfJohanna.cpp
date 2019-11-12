//
// Created by rob-ot on 18.09.19.
//
#include "BehindVisionsOfJohanna.h"

#include <headers/CameraTagInBaseCoordinates.h>
#include <headers/async_buf.h>

void BehindVisionsOfJohanna::stopStreaming() {
    isStreaming = false;
}

void BehindVisionsOfJohanna::startStreaming() {
    isStreaming = true;
    int i = 10;
    while(true) {
        try {
            manager.grabNewFrames(); //updates the frameset in the structure

            std::vector<string> deviceNames = manager.getConnectedDeviceIds();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr net(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (const string &currentDevice: deviceNames) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPc =
                        manager.getPointCloudFromCamera(currentDevice);
                *net = *net + *currentPc;

                auto frames = manager.getCVAlignedMatrix(currentDevice);
                auto depth = (std::get<1>(frames));
                auto matDepth = (std::get<0>(frames));

                if(burstStore) {
                    frameData dataStore;
                    boost::chrono::milliseconds ms = boost::chrono::duration_cast< boost::chrono::milliseconds >(
                            boost::chrono::system_clock::now().time_since_epoch()
                    );
                    //std::cout<<"time: "<<ms.count()<<std::endl;
                    dataStore.depthFrame = matDepth;
                    dataStore.timestamp = std::to_string(ms.count());
                    dataStore.centralDistance = computeFrameCentralDistance(depth);
                    persistance.emplace_back(dataStore);
                    //writer.release();
                    burstStore--;

                } else {
                    persist = true;
                }

                if(persist) {
                    int count = 0;
                    for(frameData data : persistance) {
                        persistMatrix(data, count);
                        count++;
                    }
                }


            }
            emit updatePointCloud(net);
        } catch (...) {
            std::cout<<"error"<<std::endl;
        }
    }
        //}
}

bool BehindVisionsOfJohanna::isFrameNormal(cv::Mat &depthFrame) {
    float *frame_data = (float *)depthFrame.data;
    int width = depthFrame.cols;
    int height = depthFrame.rows;
    int strideOffset = 100;
    int strideWidth = 30;

    bool hzMatch = 0;
    bool vertMatch = 0;
    int matchThresh = 50;

    int loopCounter = 0;
    int loopSumA = 0;
    int loopSumB = 0;
    for(int x = strideOffset; x<= width-strideOffset; x++) {
        for (int y = strideOffset; y<= strideOffset+strideWidth; y++) {
            loopCounter++;
            loopSumA+=frame_data[y*width+x];
        }
    }
    loopSumA/=loopCounter;
    loopCounter = 0;
    for(int x = strideOffset; x<= width-strideOffset; x++) {
        for (int y = height-strideOffset-strideWidth; y<= height-strideOffset; y++) {
            loopCounter++;
            loopSumB+=frame_data[y*width+x];
        }
    }

    loopSumB/=loopCounter;
    hzMatch = abs(loopSumA-loopSumB) < matchThresh;
    std::cout<<"horizontalness: "<<abs(loopSumA-loopSumB)<<std::endl;

    loopCounter = 0;
    loopSumA = 0;
    loopSumB = 0;
    for(int y = strideOffset; y<= height-strideOffset; y++) {
        for (int x = strideOffset; x<= strideOffset+strideWidth; x++) {
            loopCounter++;
            loopSumA+=frame_data[y*width+x];

        }
    }
    loopSumA/=loopCounter;
    loopCounter = 0;
    for(int y = strideOffset; y<= height-strideOffset; y++) {
        for (int x = width-strideOffset-strideWidth; x<= width-strideOffset; x++) {
            loopCounter++;
            loopSumB+=frame_data[y*width+x];
        }
    }
    loopSumB/=loopCounter;
    vertMatch = abs(loopSumA-loopSumB) < matchThresh;


    std::cout<<"verticalness: "<<abs(loopSumA-loopSumB)<<std::endl;

    return hzMatch && vertMatch;
}
void BehindVisionsOfJohanna::persistMatrix(BehindVisionsOfJohanna::frameData data, int count) {

    std::string fileName = "./rs2-frame-stream-at2m-"+std::to_string(count)+".yaml";
    async_buf sbuf(fileName);

    //cv::FileStorage writer(fileName, cv::FileStorage::WRITE);
    std::ostream astream(&sbuf);

    astream <<"timestamp: "<< data.timestamp << '\n';
    astream<<"distance: "<<data.centralDistance<<'\n';
    astream<<"matrix: "<<'\n';
    astream<<"rows: "<<data.depthFrame.rows<<'\n';
    astream<<"cols: "<<data.depthFrame.cols<<'\n';
    astream<<"dt: "<<"f"<<'\n';
    astream<<"data: "<<data.depthFrame<<'\n'<<std::flush;

}

double BehindVisionsOfJohanna::computeFrameCentralDistance(rs2::depth_frame &depthFrame) {
    int width = depthFrame.get_width();
    int height = depthFrame.get_height();
    int middleSquareDim = 40;
    int loopCounter = 0;
    float loopSumA = 0;
    for(int y = height/2 - middleSquareDim/2; y<= height/2 + middleSquareDim/2; y++) {
        for (int x = width/2 - middleSquareDim/2; x<= width/2+middleSquareDim/2; x++) {
            loopCounter++;
            loopSumA+=depthFrame.get_distance(x,y);
        }
    }
    return loopSumA/loopCounter;        //mean distance


}

void BehindVisionsOfJohanna::setParent(VisionsOfJohanna* johannaHerself) {
    this->johanna = johannaHerself;
}
