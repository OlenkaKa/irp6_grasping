/*!
 * \file ResultFileWriter.cpp
 * \author Aleksandra Karbarczyk
 */

#include "ResultFileWriter.h"

#include <chrono>
#include <fstream>

using namespace std;

void ResultFileWriter::init(const string &id, const string &dir_name) {
    time_t current_time_t = chrono::system_clock::to_time_t(chrono::system_clock::now());
    string current_time = string(ctime(&current_time_t));
    current_time = current_time.substr(0, current_time.length() - 1);

    file_name_ = dir_name + string("/") + id + string("-") + current_time + string(".csv");

    ofstream file;
    file.open(file_name_.c_str());
    file << "time,x,y,z,roll,pitch,yaw\n";
    file.close();
}

void ResultFileWriter::writePoseData(const PoseData &data) {
    ofstream file;
    file.open(file_name_.c_str(), ios::app);
    file.setf(ios::fixed, ios::floatfield);
    file << ros::Time::now().toSec() << ','
         << data.position.x << ',' << data.position.y << ',' << data.position.z << ','
         << data.orientation.x << ',' << data.orientation.y << ',' << data.orientation.z
         << '\n';
    file.close();
}
