//
//  visionsofjohanna.cpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//

#include "visionsofjohanna.hpp"
#include <QVTKWidget.h>
#include "ui_visionsofjohanna.h"
VisionsOfJohanna::VisionsOfJohanna(QWidget *parent) :
QMainWindow(parent),
ui(new Ui::VisionsOfJohanna)
{
   ui->setupUi(this);
}

VisionsOfJohanna::~VisionsOfJohanna()
{
    delete ui;
}
