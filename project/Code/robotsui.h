#ifndef ROBOTSUI_H
#define ROBOTSUI_H

#include <QMainWindow>
#include <QTimer>

#include "State.h"

QT_BEGIN_NAMESPACE
namespace Ui { class RobotsUI; }
QT_END_NAMESPACE

class RobotsUI : public QMainWindow
{
    Q_OBJECT

public:
    RobotsUI(QWidget *parent = nullptr);
    ~RobotsUI();
    void setState(State state);
    double getTime();
    unsigned getSpeed();
    void setTime(double newTime);

private slots:
    void on_actionOpen_triggered();

    void on_actionOpenTimeline_triggered();

    void on_actionSave_triggered();

    void on_actionQuit_triggered();

    void on_OpenStateButton_clicked();

    void on_controlButton_clicked();

    void on_beginningButton_clicked();

    void on_OpenTimelineButton_clicked();

    void resizeEvent(QResizeEvent* event);

private:
    void changeScore(Time newTime, State it);
    void OpenState();
    void OpenTimeline();
    void SaveState();
    void timeIncrement();
    void changeTimeBoxValue(Time newTime);
    void resetTimer();
    void preparingLayout(bool needPanel, QString fileName);
    void errorOpeningFile(QString errorMessage);

    Ui::RobotsUI *ui;
    QTimer *timer;
    };

#endif // ROBOTSUI_H
