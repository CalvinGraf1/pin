#include <QFileDialog>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

#include "robotsui.h"
#include "./ui_robotsui.h"
#include "Timeline.h"
#include "StateStream.h"

using namespace std;

TimeLine timeline;

RobotsUI::RobotsUI(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::RobotsUI)
{
    ui->setupUi(this);
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &RobotsUI::timeIncrement); // Calculates the time and modify the timer accordingly
    connect(ui->timeSpinBox, &QDoubleSpinBox::valueChanged,this, &RobotsUI::changeTimeBoxValue); // Event when the timer changes manually or naturally
}

RobotsUI::~RobotsUI(){
    delete ui;
    delete timer;
}

void RobotsUI::setState(State state){
    ui->widget->setState(state);
}

double RobotsUI::getTime(){
    return ui->timeSpinBox->value();
}

void RobotsUI::setTime(double newTime){
    ui->timeSpinBox->setValue(newTime);
}

unsigned RobotsUI::getSpeed(){
    return ui->speedBox->value();
}

// Change to an appropriate layout when loading a new state/timeline
void RobotsUI::preparingLayout(bool needPanel, QString fileName){
    ui->OpenStateButton->hide();
    ui->OpenTimelineButton->hide();
    ui->widgetPanelControl->setEnabled(needPanel);
    QFileInfo fileInfo(fileName);
    QString fileWithoutPath = fileInfo.fileName();
    setWindowTitle(fileWithoutPath);
}

void RobotsUI::resetTimer(){
    timer->stop();
    setTime(0);
    ui->controlButton->setText("|>");
}

void RobotsUI::errorOpeningFile(QString errorMessage){
    QApplication::beep();
    State state;
    setState(state);
    cerr << errorMessage.toStdString() << endl;
}

void RobotsUI::OpenState(){
    try {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                 tr("Open State File"), "", tr("Json Files (*.stat)"));
        if(fileName.isNull())
            return;

        ifstream ifs(fileName.toStdString());
        State state;
        ifs >> state;
        setState(state);

        // Reset parameters
        resetTimer();
        preparingLayout(false, fileName);
        changeScore(state.time, state);
        ui->controlButton->setText("|>");
    }
    catch (std::exception e) {
        errorOpeningFile("Error: This state could not be loaded.");
    }
}

void RobotsUI::OpenTimeline(){
    try {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open State File"), "", tr("Json Files (*.tlin)"));
        if(fileName.isNull())
                return;

        ifstream ifs(fileName.toStdString());
        ifs >> timeline;
        setState(timeline.states[0]);

        // Force to stay in the time limit
        ui->timeSpinBox->setMaximum(timeline.states.back().time + 0.1);
        ui->timeSpinBox->setMinimum(0);

        // Reset parameters
        resetTimer();
        preparingLayout(true, fileName);
        changeScore(0, *timeline.states.begin());
        ui->controlButton->setText("|>");
    }
    catch (std::exception e) {
        errorOpeningFile("Error: This timeline could not be loaded.");
    }
}

void RobotsUI::SaveState(){
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save State File"), "", tr("Json Files (*.stat)"));
    if(fileName.isNull())
            return;

    ofstream ofs(fileName.toStdString());
    ofs << setw(4) << ui->widget->getState();
}

void RobotsUI::on_actionOpen_triggered(){
    this->OpenState();
}

void RobotsUI::on_actionOpenTimeline_triggered(){
    this->OpenTimeline();
}

void RobotsUI::on_actionSave_triggered(){
    this->SaveState();
}

void RobotsUI::on_actionQuit_triggered(){
    qApp->exit(0);
}

void RobotsUI::on_OpenStateButton_clicked(){
    this->OpenState();
}

void RobotsUI::on_OpenTimelineButton_clicked(){
    this->OpenTimeline();
}

void RobotsUI::resizeEvent(QResizeEvent* event){
    int newSize = qMin(ui->widget->size().width(), ui->widget->size().height());
    ui->widget->resize(newSize, newSize); // Resize the width and height to keep a square world
    ui->widgetPanelControl->resize(newSize, ui->widgetPanelControl->size().height()); // Resize the panel control so it has the same width as the world
}

void RobotsUI::timeIncrement(){
    // Allow to increment the timer based on the 24 fps and speed
    const double fps = 24;
    double frameRelative = fps / getSpeed();
    double timeIntermediate = 1.0 / frameRelative;
    double time = getTime();
    time += timeIntermediate;
    setTime(time);
}

bool compareState(State a, State b){
    return  a.time < b.time;
}

// Event the triggers at every clock ticks
void RobotsUI::changeTimeBoxValue(Time newTime){
    if(getTime() >= timeline.states.back().time) timer->stop();

    State target = {.time = newTime};
    auto it = upper_bound(timeline.states.begin(), timeline.states.end(),target, compareState);
    if(it != timeline.states.begin()) --it;
    changeScore(newTime, *it);
    setState(calculateNextState(*it, newTime));
}

void RobotsUI::changeScore(Time newTime, State it){
    State target = {.time = newTime};
    double scoreRemaining = 0;
    double scoreRobot = 0;

    // Get the particles' size to calculate the scores
    for (int i = 0; i < it.particles.size(); ++i) scoreRemaining += M_PI * pow(it.particles.at(i).radius, 2);
    for (int i = 0; i < it.robots.size(); ++i) scoreRobot += it.robots.at(i).score;

    QString truncatedScoreRemaining;
    truncatedScoreRemaining.setNum(scoreRemaining, 'f', 0); // Allow to delete the decimal figures and the comma
    ui->maxScore->setText(truncatedScoreRemaining);

    QString truncatedScore;
    truncatedScore.setNum(scoreRobot, 'f', 0);
    ui->labelScore->setText(truncatedScore);
}

void RobotsUI::on_controlButton_clicked(){
    if(timer->isActive()){
        timer->stop();
        ui->controlButton->setText("|>");
    }
    else{//Ticks 24 times in 1 sec
        timer->start(1000./24);
        ui->controlButton->setText("||");
    }
}

void RobotsUI::on_beginningButton_clicked(){
    resetTimer();
    setState(timeline.states[0]);
}



