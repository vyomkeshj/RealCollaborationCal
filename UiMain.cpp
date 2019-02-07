//
//  UiMain.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//

#include "visionsofjohanna.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    VisionsOfJohanna w;
    w.show();
    
    return a.exec();
}
