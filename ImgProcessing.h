#ifndef IMGPROCESSING_H
#define IMGPROCESSING_H
#include "opencv.hpp"
#include "QImage"

class ImgProcessing
{
public:
    static QImage Mat2QImage(const cv::Mat &img);

    static cv::Mat SplitChessBoard(const cv::Mat img);

    static int CalMatOTSU(cv::Mat &img);

    static std::vector<cv::Vec3f> GetCicles(const cv::Mat &img);
};


#endif // IMGPROCESSING_H
