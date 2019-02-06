#ifndef UIENTRYPOINT_H
#define UIENTRYPOINT_H

#include <QMainWindow>

namespace Ui {
class UiEntrypoint;
}

class UiEntrypoint : public QMainWindow
{
    Q_OBJECT

public:
    explicit UiEntrypoint(QWidget *parent = nullptr);
    ~UiEntrypoint();

private:
    Ui::UiEntrypoint *ui;
};

#endif // UIENTRYPOINT_H
