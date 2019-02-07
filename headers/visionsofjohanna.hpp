//
//  visionsofjohanna.hpp
//  RealCollaborationCal
//
//  Created by Vyomkesh Jha on 07/02/19.
//

#ifndef VISIONSOFJOHANNA_H
#define VISIONSOFJOHANNA_H
#define vtkRenderingCore_AUTOINIT4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,tkRenderingOpenGL)
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)

#include <QMainWindow>

namespace Ui {
    class VisionsOfJohanna;
}

class VisionsOfJohanna : public QMainWindow
{
    Q_OBJECT

public:
    explicit VisionsOfJohanna(QWidget *parent = nullptr);
    ~VisionsOfJohanna();
    
private:
    Ui::VisionsOfJohanna *ui;
};

#endif
