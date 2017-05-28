/*!
 * \file ResultFileWriter.h
 * \author Aleksandra Karbarczyk
 */

#ifndef IRP6_GRASPING_RESULT_FILE_WRITER_H
#define IRP6_GRASPING_RESULT_FILE_WRITER_H

#include "pose_kalman_filter.h"

namespace irp6_grasping
{
class ResultFileWriter
{
public:
  ResultFileWriter(const std::string &id, const std::string &dir_name);
  void writePoseData(const PoseData &data);

private:
  std::string file_name_;
};
}  // irp6_grasping

#endif  // IRP6_GRASPING_RESULT_FILE_WRITER_H
