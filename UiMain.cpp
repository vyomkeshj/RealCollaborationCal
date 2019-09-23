//
//  UiMain.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//
#include <librealsense2/rs.hpp>

#include "visionsofjohanna.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    VisionsOfJohanna w;
    w.show();
    rs2::log_to_file(RS2_LOG_SEVERITY_DEBUG, "debugout.log");

    return a.exec();
}
