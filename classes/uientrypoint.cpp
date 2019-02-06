#include "uientrypoint.h"
#include <QApplication>
#include <QtWidgets>

UiEntrypoint::UiEntrypoint(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UiEntrypoint)
{
    ui->setupUi(this);
}

UiEntrypoint::~UiEntrypoint()
{
    delete ui;
}
