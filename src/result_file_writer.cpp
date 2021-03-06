/*!
 * \file ResultFileWriter.cpp
 * \author Aleksandra Karbarczyk
 */

#include <irp6_grasping/result_file_writer.h>

#include <chrono>
#include <fstream>

using namespace irp6_grasping;
using namespace std;

ResultFileWriter::ResultFileWriter(const string &id, const string &dir_name)
{
  time_t current_time_t = chrono::system_clock::to_time_t(chrono::system_clock::now());
  string current_time = string(ctime(&current_time_t));
  current_time = current_time.substr(0, current_time.length() - 1);

  file_name_ = dir_name + string("/") + id + string("-") + current_time + string(".csv");

  ofstream file;
  file.open(file_name_.c_str());
  file << "time,x,y,z,roll,pitch,yaw,dx,dy,dz,droll,dpitch,dyaw\n";
  file.close();
}

void ResultFileWriter::writePoseData(const ros::Time &time_stamp, const PoseData &data) const
{
  ofstream file;
  file.open(file_name_.c_str(), ios::app);
  file.setf(ios::fixed, ios::floatfield);
  file << time_stamp.toSec() << ','
       << data.position.x << ',' << data.position.y << ',' << data.position.z << ','
       << data.orientation.x << ',' << data.orientation.y << ',' << data.orientation.z << ','
       << data.velocity.linear.x << ',' << data.velocity.linear.y << ',' << data.velocity.linear.z << ','
       << data.velocity.angular.x << ',' << data.velocity.angular.y << ',' << data.velocity.angular.z <<'\n';
  file.close();
}
