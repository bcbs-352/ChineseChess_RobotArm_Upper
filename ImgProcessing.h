#ifndef IMGPROCESSING_H
#define IMGPROCESSING_H
#include "opencv.hpp"
#include "QImage"

class ImgProcessing
{
public:
    static QImage Mat2QImage(const cv::Mat &img);

    static cv::Mat SplitChessBoard(const cv::Mat img);

    static int CalcMatOTSU(cv::Mat &img);

    static std::vector<cv::Vec3f> GetCirclesPos(const cv::Mat &img);

    static std::vector<cv::Mat> GetCirclesImg(const cv::Mat &img,std::vector<cv::Vec3f> &circles);
};


#endif // IMGPROCESSING_H
