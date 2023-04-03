#ifndef IMGPROCESSING_H
#define IMGPROCESSING_H
#include "opencv.hpp"
#include <string>
#include "QImage"

using namespace cv;
using namespace std;

class ImgProcessing
{
public:
    static QImage Mat2QImage(const Mat &img);

    static int CalcMatOTSU(Mat &img);

    static Mat SplitChessBoard(const Mat &img);

    static vector<Vec3f> GetCirclesPos(const Mat &img);

    static vector<Mat> GetChessImg(const Mat &img, vector<Vec3f> &circles);

    static vector<Point2f> GetChessCoordinate(const Mat &img, vector<Vec3f> &circles, float Hratio=0.818, float Wratio=0.81);

    static vector<int> ClassifyChessImg(const vector<Mat> &chessImgs);

    static vector<vector<int>> GetGridInfo(const vector<Point2f> &coordinates, const vector<int> &chessCategories);

    static vector<vector<int>> MainFunc(const Mat &inImg);

    //static vector<string> chess_list;
};

//vector<string> ImgProcessing::chess_list = {"黑士", "红仕", "黑象", "红相", "黑炮", "红炮", "黑将", "红帅", "黑马", "红马", "黑卒", "红兵", "黑車", "红車"};


#endif // IMGPROCESSING_H
