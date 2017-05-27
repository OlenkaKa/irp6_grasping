/*!
 * \file ResultFileWriter.h
 * \author Aleksandra Karbarczyk
 */

#ifndef IRP6_GRASPING_RESULTFILEWRITER_H
#define IRP6_GRASPING_RESULTFILEWRITER_H

#include <string>
#include "pose_kalman_filter.h"

class ResultFileWriter {
public:
    void init(const std::string &id, const std::string &dir_name);
    void writePoseData(const PoseData &data);
private:
    std::string file_name_;
};

#endif //IRP6_GRASPING_RESULTFILEWRITER_H
