//
// Created by swh on 23-3-6.
//
#include <glog/logging.h>
#include "hdmap/entity/map_road.h"
#include "hdmap/entity/map_lane.h"
#include "hdmap/entity/map.h"
#include "hdmap/adapter/opendrive_adapter.h"
#include <iostream>
#include <exception>

int main(int argc,char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = false; // 是否将日志输出到stderr而非文件。
    FLAGS_alsologtostderr = false; //是否将日志输出到文件和stderr，如果：true，忽略FLAGS_stderrthreshold的限制，所有信息打印到终端。
    FLAGS_stderrthreshold = google::GLOG_WARNING; //输出到stderr的限值，默认为2（ERROR），默认ERORR以下的信息(INFO、WARNING)不打印到终端。
    FLAGS_minloglevel = 2;
    FLAGS_log_dir = "/home/ai/catkin_ws/src/prediction/log";

//    FLAGS_v = 0;
//    FLAGS_log_prefix = true; //设置日志前缀是否应该添加到每行输出。
//    FLAGS_logbufsecs = 0; //设置可以缓冲日志的最大秒数，0指实时输出。
//    FLAGS_max_log_size = 10; //设置最大日志文件大小（以MB为单位）。
//    FLAGS_stop_logging_if_full_disk = true; //设置是否在磁盘已满时避免日志记录到磁盘。

//    google::SetStderrLogging(google::GLOG_INFO);
//    google::SetStderrLogging(google::GLOG_WARNING);
//
    char* path = std::getenv("PREDICTION_PATH");
    std::string str_path(path);
    std::string map_file = str_path + "/src/hdmap/data/Town10HD_tmp3.xodr";
    std::shared_ptr<aptiv::hdmap::entity::Map> map_ptr(new aptiv::hdmap::entity::Map);
    try {
        aptiv::hdmap::adapter::OpendriveAdapter::LoadData(map_file,map_ptr);
    }catch (std::exception &exception){
        std::cout << exception.what() << std::endl;
    }

    return 0;
}