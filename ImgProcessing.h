#ifndef IMGPROCESSING_H
#define IMGPROCESSING_H
#include "opencv.hpp"
#include <string>
#include "QImage"

using namespace cv;
using namespace std;

// Pos-图像(像素)坐标  Grid-(理想)棋盘着点  Coordinate-(理想)棋盘着点坐标  Location-真实空间坐标

class ImgProcessing
{
public:
    static QImage Mat2QImage(const Mat &img);

    static int CalcMatOTSU(Mat &img);

    static Mat SplitChessBoard(const Mat &img, vector<double>cannyParams);

    static vector<Vec3f> GetCirclesPos(const Mat &img, vector<double> &houghParams);

    static vector<Mat> GetChessImg(const Mat &img, vector<Vec3f> &circles);

    static vector<Point2f> GetChessCoordinate(const Mat &img, vector<Vec3f> &circles, float Hratio=0.818, float Wratio=0.81);

    static vector<int> ClassifyChessImg(const vector<Mat> &chessImgs);

    static vector<vector<int>> GetGridInfo(const vector<Point2f> &coordinates, const vector<int> &chessCategories);

    static vector<vector<int>> MainFunc(const Mat &inImg, vector<double> &cannyParams, vector<double> &houghParams);

    static QString GetFenStr(vector<vector<int>> gridInfo);

    static void CalsPos2Location(Vec2f leftUp, Vec2f rightUp, Vec2f rightDown);

    //static vector<string> chess_list;
    static Vec2f boardSize;
    static vector<vector<int>> gridInfo;
    static vector<Vec2f> chessPos;         // 检测到圆的圆心坐标对应的现实坐标
    static vector<vector<Vec2f>> gridPos, gridLocation, gridChessPos, gridChessLocation;  // 棋盘格点坐标对应的像素/现实坐标, 着点上若有棋子，则要获取着点上棋子现实坐标
};

//vector<string> ImgProcessing::chess_list = {"黑士", "红仕", "黑象", "红相", "黑炮", "红炮", "黑将", "红帅", "黑马", "红马", "黑卒", "红兵", "黑車", "红車"};


#endif // IMGPROCESSING_H
