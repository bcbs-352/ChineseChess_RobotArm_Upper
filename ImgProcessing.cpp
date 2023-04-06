#include "ImgProcessing.h"
#include "qdebug.h"

using namespace cv;
using namespace std;

const int H=32,W=32;

// Mat格式的图像转为QImage格式
QImage ImgProcessing::Mat2QImage(const Mat &img)
{
    switch(img.type())
    {
        case CV_8UC1:
        {
            QImage image(img.cols,img.rows,QImage::Format_Indexed8);
            image.setColorCount(256);
            for(int i=0;i<256;++i)
                image.setColor(i,qRgb(i,i,i));
            uchar*pSrc=img.data;
            for(int row=0;row<img.rows;++row)
            {
                uchar*pDest= image.scanLine(row);
                memcpy(pDest,pSrc,img.cols);
                pSrc+=img.step;
            }
            return image;
        }
        case CV_8UC3:
        {
            cvtColor(img,img,COLOR_BGR2RGB);
            const uchar*pSrc=(const uchar*)img.data;
            QImage image(pSrc,img.cols,img.rows,img.step,QImage::Format_RGB888);
            return image.copy();
        }
        case CV_8UC4:
        {
            const uchar*pSrc=(const uchar*)img.data;
            QImage image(pSrc,img.cols,img.rows,img.step,QImage::Format_ARGB32);
            return image.copy();
        }
    default:
        qDebug()<<"无法转换为QImage";
        return QImage();
    }
}

// 对Rotated矩形四个点排序(左上、右上、右下、左下)
void SortRotatedRectPoints(Point2f vetPoints[], RotatedRect rect)
{
    rect.points(vetPoints);
    Point2f curpoint;
    //按X轴排序
    for (int i = 0; i < 4; ++i) {
        for (int k = i + 1; k < 4; ++k) {
            if (vetPoints[i].x > vetPoints[k].x) {
                curpoint = vetPoints[i];
                vetPoints[i] = vetPoints[k];
                vetPoints[k] = curpoint;
            }
        }
    }
    //判断X坐标前两个定义左上左下角
    if (vetPoints[0].y > vetPoints[1].y) {
        curpoint = vetPoints[0];
        vetPoints[0] = vetPoints[1];
        vetPoints[1] = vetPoints[3];
        vetPoints[3] = curpoint;
    }
    else {
        curpoint = vetPoints[3];
        vetPoints[3] = vetPoints[1];
        vetPoints[1] = curpoint;
    }
    //判断X坐标后两个定义右上右下角
    if (vetPoints[1].y > vetPoints[2].y) {
        curpoint = vetPoints[1];
        vetPoints[1] = vetPoints[2];
        vetPoints[2] = curpoint;
    }
}

int CalPointsDistance(const Point2f&a,const Point2f&b)
{
    return sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
}

// 获取轮廓点列表中，最接近外接矩形角点的边界点
void GetContourCorner(Point2f nearPoints[],Point2f rectPoints[],vector<Point> contourPoints)
{
    vector<float> minDist(4,1e38);
    for (auto point : contourPoints) {
        for(size_t i=0;i<4;++i)
        {
            int distance=sqrt(pow((rectPoints[i].x-point.x),2)+pow((rectPoints[i].y-point.y),2));
            if(distance<minDist[i])
            {
                minDist[i]=distance;
                nearPoints[i]=point;
            }
        }
    }
}

void DrawHoughLine(Mat &img)
{
    vector<Vec4f> lines;
    HoughLinesP(img,lines,5,CV_PI/180,3,200,20);
    for(size_t i=0;i<lines.size();++i)
    {
        Vec4f l=lines[i];
        line(img,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(255,255,255),1,LINE_AA);
    }
    //Canny(img,img,80,127);
}

// 分割图像中棋盘区域    预处理、边缘检测、四边形轮廓拟合、透视变换   参考Vaccae
Mat ImgProcessing::SplitChessBoard(const Mat &inImg)
{
    Mat img,out;
    inImg.copyTo(img);
    inImg.copyTo(out);
    // 高斯滤波
    GaussianBlur(img,img,Size(5,5),0.5,0.5);
    //medianBlur(img,img,9);  // 中值滤波

    // 分为BGR三通道
    vector<Mat> channels;
    Mat B_img,G_img,R_img,gray_img,sumImg;
    split(img,channels);
    cvtColor(img,gray_img,COLOR_BGR2GRAY);

    //GetCicles(img);

    // <del>直方图均衡化</del> 大津法获取分割阈值，Canny算子边缘检测
    int threshold;
    //equalizeHist(channels[0],channels[0]);
    threshold=CalcMatOTSU(channels[0]);
    Canny(channels[0],B_img,threshold/2,saturate_cast<uchar>(1.3*threshold));
    threshold=CalcMatOTSU(channels[1]);
    Canny(channels[1],G_img,threshold/2,saturate_cast<uchar>(1.3*threshold));
    threshold=CalcMatOTSU(channels[2]);
    Canny(channels[2],R_img,threshold/2,saturate_cast<uchar>(1.3*threshold));

    threshold=CalcMatOTSU(gray_img);
    Canny(gray_img,gray_img,threshold/2,saturate_cast<uchar>(1.3*threshold));

    DrawHoughLine(B_img);DrawHoughLine(G_img);DrawHoughLine(R_img);

    // 三个通道或运算
    bitwise_or(B_img,G_img,sumImg);
    bitwise_or(R_img,sumImg,sumImg);
    //threshold=CalMatOTSU(gray_img);
    //threshold(sumImg,sumImg,threshold,255,THRESH_BINARY_INV);
    //sumImg=~sumImg;
    imshow("CannyGray",sumImg);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(sumImg,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_TC89_KCOS);

    int imgArea=img.size().area(),maxAreaIndex=0;
    float maxArea=0;

    // 遍历识别到的轮廓
    vector<vector<Point>> squares;
    for(size_t i=0;i<contours.size();++i)
    {
        vector<Point> approx;
        // 精度0.02 曲线闭合 道格拉斯-普克算法(Douglas-Peucker)
        approxPolyDP(Mat(contours[i]),approx,arcLength(Mat(contours[i]),true)*0.01,true);
        if(approx.size()!=4 || contourArea(Mat(approx)) < 4000 || !isContourConvex(Mat(approx)))
            continue;
        squares.push_back(approx);
        /*
        // 检测四个角是否接近直角
        double maxCos=0;
        for(int j=2;j<5;++j)
        {
            Point p1=approx[j%4],p2=approx[j-2],p0=approx[j-1];
            double x1=p1.x-p0.x, y1=p1.y-p0.y;
            double x2=p2.x-p0.x, y2=p2.y-p0.y;
            double cosAngle = fabs((x1*x2+y1*y2)/sqrt((x1*x1+y1*y1)*(x2*x2+y2*y2)+1e-10));
            if(cosAngle > maxCos)
                maxCos=cosAngle;
        }
        //qDebug()<<maxCos;
        if(maxCos<0.3)
            squares.push_back(approx);
        */

        // 获取面积最大四边形编号
        RotatedRect rotateRect=minAreaRect(contours[i]);
        if(rotateRect.size.area() > maxArea && rotateRect.size.area()>imgArea*0.1)
        {
            maxArea=rotateRect.size.area();
            maxAreaIndex=i;
        }

    }
    qDebug()<<contours.size();
    qDebug()<<maxAreaIndex<<"  "<<maxArea;

    //Mat imgContour = Mat::zeros(sumImg.size(),CV_8SC3);

    // 获取并绘制最小包围矩形
    RotatedRect boundingRect = minAreaRect(contours[maxAreaIndex]);
    Point2f vertices[4];
    SortRotatedRectPoints(vertices, boundingRect);
    for (int k = 0; k < 4; ++k)
    {
        line(out, vertices[k], vertices[(k + 1) % 4], Scalar(255, 0, 0),5);
    }

    // 获取轮廓点列表中距离外接矩形边角最近的4个点，并绘制该4点围成的多边形
    Point2f nearPoints[4];
    GetContourCorner(nearPoints,vertices,contours[maxAreaIndex]);
    for (int k = 0; k < 4; ++k)
    {
        line(out, nearPoints[k], nearPoints[(k + 1) % 4], Scalar(0, 0, 255),3);
    }

    // 计算目标矩形坐标点
    Point2f rectPoints[4];
    float width=CalPointsDistance(vertices[0],vertices[1]);
    float height=CalPointsDistance(vertices[1],vertices[2]);
    //rectPoints[0]=Point2f(nearPoints[0].x,nearPoints[0].y);
    rectPoints[0]=Point2f(vertices[0].x,vertices[0].y);
    rectPoints[1]=rectPoints[0]+Point2f(width,0);
    rectPoints[2]=rectPoints[1]+Point2f(0,height);
    rectPoints[3]=rectPoints[0]+Point2f(0,height);
    for (int k = 0; k < 4; ++k)
    {
        line(out, rectPoints[k], rectPoints[(k + 1) % 4], Scalar(0, 255, 255),3);
    }
    imshow("rect img",out);

    // 计算透视变换矩阵
    Mat transformImg;
    Mat transformMat=getPerspectiveTransform(nearPoints,rectPoints);
    warpPerspective(img,transformImg,transformMat,transformImg.size(),INTER_LINEAR);
    //imshow("perspect img",transformImg);

    // 裁切棋盘图像
    qDebug()<<"裁切区域:左上("<<rectPoints[0].x<<","<<rectPoints[0].y<<");右下("<<rectPoints[2].x<<","<<rectPoints[2].y<<")";
    Mat cutImg=transformImg(Rect(rectPoints[0]-Point2f(width*0.05,height*0.05),rectPoints[2]+Point2f(width*0.05,height*0.05)));
    //Mat cutImg=transformImg(Rect(rectPoints[0],rectPoints[2]));
    imshow("cut img",cutImg);

    /*
    // 绘制所有矩形
    for (size_t i = 0; i < contours.size(); i++)
    {
        const Point* p = &contours[i][0];

        int n = (int)contours[i].size();
        if (p->x > 3 && p->y > 3)
        {
            polylines(out, &p, &n, 1, true, Scalar(0, 0, 255), 3, LINE_AA);
            //polylines(out, squares, true, Scalar(0, 0, 255), 3, LINE_AA);
        }
    }
    */

    return cutImg;
}

// 最大类间方差法(大津OTSU法)求自适应分割阈值
int ImgProcessing::CalcMatOTSU(Mat &img)
{
    if(img.channels()>1)
        return 128;
    int rows=img.rows,cols=img.cols;

    // 灰度直方图
    float mathists[256]={0};
    for(int row=0;row<rows;++row)
    {
        for(int col=0;col<cols;++col)
        {
            int val=img.at<uchar>(row,col);
            mathists[val]++;
        }
    }
    // 每个灰度在图中占比
    float grayPro[256]={0};
    int matSize=rows*cols;
    for(int i=0;i<256;++i)
        grayPro[i] = (float)mathists[i]/(float)matSize;

    int retVal=-1;  // 最佳阈值
    int maxVal=0;   // 最大方差
    for(int i=0;i<256;++i)
    {
        // 0、1表示深浅两部分,w_tmp表示各部分概率和,u_表示各部分灰度均值
        float w0=0,w1=0,u0tmp=0,u1tmp=0,u0=0,u1=0,u=0,tmpVariance=0;
        for(int j=0;j<256;++j)
        {
            if(j<=i){
                w0+=grayPro[j];
                u0tmp+=grayPro[j]*j;
            }else{
                w1+=grayPro[j];
                u1tmp+=grayPro[j]*j;
            }
        }
        u0=u0tmp/w0;
        u1=u1tmp/w1;
        // 全图平均灰度
        u=u0tmp+u1tmp;
        // 当前灰度阈值下的类间方差
        tmpVariance = w0*pow((u0-u),2)+w1*pow((u1-u),2);
        if(tmpVariance>maxVal)
        {
            maxVal=tmpVariance;
            retVal=i;
        }
    }
    return retVal;
}

// 对图像进行霍夫圆检测，返回检测到的circles列表
vector<Vec3f> ImgProcessing::GetCirclesPos(const Mat &img)
{
    // 预处理:滤波(中值/高斯),转为灰度图,获取分割阈值
    vector<Vec3f> circles;
    Mat tmp,tmp_gray;
    img.copyTo(tmp);
    medianBlur(tmp,tmp,1);
    cvtColor(tmp,tmp_gray,COLOR_BGR2GRAY);

    int threshold=CalcMatOTSU(tmp_gray);

    double dp=2,minDist=20;
    double param=80; // 检测阶段圆心的累加器阈值。值越小，越可以检测到更多不存在的圆
    int minRadius=10,maxRadius=30;
    //do{
        circles.clear();
        HoughCircles(tmp_gray,circles,HOUGH_GRADIENT,dp,minDist,threshold,param,minRadius,maxRadius);
        qDebug()<<circles.size()<<"param:"<<param;
    //}while(circles.size()>32);

    for(size_t i=0;i<circles.size();++i)
    {
        circle(tmp,Point(circles[i][0],circles[i][1]),circles[i][2],Scalar(0,255,0),2,8);
        circle(tmp,Point(circles[i][0],circles[i][1]),10,Scalar(0,255,0),-1,8);
    }
    imshow("Hough Circle",tmp);

    return circles;
}

// 截取圆形区域图像，并resize
vector<Mat> ImgProcessing::GetChessImg(const Mat &inImg, vector<Vec3f> &circles)
{
    vector<Mat> circleImgs;
    for(size_t i=0;i<circles.size();++i)
    {
        Mat img;
        Mat mask=Mat::zeros(inImg.size(),CV_8UC3);
        circle(mask,Point(circles[i][0],circles[i][1]),circles[i][2],Scalar(255,255,255),-1);
        inImg.copyTo(img,mask);
        Point2f upleft(circles[i][0]-1.1*circles[i][2],circles[i][1]-1.1*circles[i][2]);
        Point2f lowright(circles[i][0]+1.1*circles[i][2],circles[i][1]+1.1*circles[i][2]);
        Mat cutImg=img(Rect(upleft,lowright));

        resize(cutImg,cutImg,Size(32,32),0,0,INTER_CUBIC);
        circleImgs.push_back(cutImg);

        // 保存图像
        //imshow("im:"+to_string(i),cutImg);
        imwrite("C:\\Users\\asus\\Desktop\\ChessDataset\\origin\\"+to_string(i)+".jpg",cutImg);
    }
    return circleImgs;
}

// 根据ratio确定棋盘格范围，根据圆心坐标，判断棋子所在的着点坐标(从左到右，从上到下)
vector<Point2f> ImgProcessing::GetChessCoordinate(const Mat &broadImg, vector<Vec3f> &circles, float Hratio, float Wratio)
{
    int height = broadImg.rows * Hratio, width = broadImg.cols * Wratio;
    int up = broadImg.rows * (1-Hratio)/2, left = broadImg.cols * (1-Wratio)/2;
    int gridHeight = height/9, gridWidth = width/8;

    // 绘制网格图
    Mat img = broadImg.clone();
    for(int i=0;i<10;++i)
    {
        // 横线
        line(img,Point(left,up+gridHeight*i),Point(left+width,up+gridHeight*i),Scalar(255,255,255),1,LINE_AA);
        // 纵线
        if(i<9)
            line(img,Point(left+gridWidth*i,up),Point(left+gridWidth*i,up+height),Scalar(255,255,255),1,LINE_AA);
    }
    imshow("grid img",img);

    vector<Point2f> coordinates;
    for(size_t i=0;i<circles.size();++i)
    {
        auto circle = circles[i];
        int x = (circle[0]-(left-gridWidth/2)) / gridWidth;
        int y = (circle[1]-(left-gridHeight/2)) / gridHeight;
        coordinates.push_back(Point2f(x,y));
    }
    return coordinates;
}

// 使用onnx模型对棋子图像分类，返回索引值
vector<int> ImgProcessing::ClassifyChessImg(const vector<Mat> &chessImgs)
{
    vector<string> chess_list = {"黑士", "红仕", "黑象", "红相", "黑炮", "红炮", "黑将", "红帅", "黑马", "红马", "黑卒", "红兵", "黑車", "红車"};

    dnn::Net net = dnn::readNetFromONNX("C:\\Users\\asus\\Desktop\\ChessDataset\\myModel_new.onnx");
    dnn::dnn4_v20220524::ClassificationModel cnn_model(net);
    cnn_model.setInputParams(1,Size(H,W),Scalar(0,0,0),true,false);


    net.setPreferableBackend(dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(dnn::DNN_TARGET_CUDA);
    Mat dst;
    vector<int> chessCategories;
    for(size_t i=0;i<chessImgs.size();++i)
    {
        Mat img = chessImgs[i];
        //imshow("imshow"+to_string(i),img);
        dst = chessImgs[i].clone();
        //resize(img,dst,Size(H,W),INTER_CUBIC);
        //dst = (dst/255.-0.5)/0.5;   // 归一化,对应transform.Normalize()
        auto blob = dnn::blobFromImage(dst, 2./255, Size(H, W), Scalar(127,127, 127), true, false);  // 转为四维Tensor类型[1,3,H,W]
        net.setInput(blob,"input");
        Mat output = net.forward("output");

        float maxVal = -10000.;
        int index = 0;
        for(int i=0;i<output.cols;++i)
        {
            //qDebug()<<output.at<float>(0,i);
            if(output.at<float>(0,i) > maxVal)
            {
                maxVal = output.at<float>(0,i);
                index = i;
            }
        }

        float confs;
        //cnn_model.classify(dst,index,confs);
        //qDebug()<<"当前棋子:" << QString::fromStdString(chess_list[index])<<"confs:"<<confs;

        chessCategories.push_back(index);
        //qDebug()<<QString::fromStdString(chess_list[index]);
    }

    return chessCategories;
}

// 根据棋子坐标和棋种索引,返回10行9列着点状态(-1为空, 0~13对应棋种)
// coordinates:坐标列表     chessCategories:棋种索引列表
vector<vector<int>> ImgProcessing::GetGridInfo(const vector<Point2f> &coordinates, const vector<int> &chessCategories)
{
    vector<string> chess_list = {"黑士", "红仕", "黑象", "红相", "黑炮", "红炮", "黑将", "红帅", "黑马", "红马", "黑卒", "红兵", "黑車", "红車"};
    vector<vector<int>> gridInfo(10,vector<int>(9,-1));
    if(coordinates.size()!=chessCategories.size())
        return gridInfo;
    for(size_t i=0;i<chessCategories.size();++i)
    {
        gridInfo[coordinates[i].x][coordinates[i].y]=chessCategories[i];
        qDebug()<<i<<"棋种:"<<QString::fromStdString(chess_list[chessCategories[i]])<<" 坐标("<<coordinates[i].x<<","<<coordinates[i].y<<")";
    }
    return gridInfo;
}

void WriteChess(const Mat &inImg, vector<Vec3f> &circlesPos, vector<int> &chessCategories)
{
    Mat dst = inImg.clone();
    vector<string> chess_list = {"shi", "shi", "xiang", "xiang", "pao", "pao", "jiang", "shuai", "ma", "ma", "zu", "bing", "che", "che"};
    for(size_t i=0;i<circlesPos.size();++i)
    {
        Scalar textColor = chessCategories[i]%2 == 0 ? Scalar(0,0,0) : Scalar(0,0,255);
        cv::putText(dst,chess_list[chessCategories[i]],Point2f(circlesPos[i][0]-32,circlesPos[i][1]),FONT_HERSHEY_SIMPLEX,1,textColor,2);
    }
    imshow("dst",dst);
}

// 主函数
vector<vector<int> > ImgProcessing::MainFunc(const Mat &inImg)
{
    // 分割棋盘区域(周围扩大5%)
    Mat cutImg = SplitChessBoard(inImg);
    // 霍夫曼圆检测
    vector<Vec3f> circlesPos = GetCirclesPos(cutImg);
    // 截取棋子图像
    vector<Mat> chessImgs = GetChessImg(cutImg,circlesPos);
    // 获取棋子对应的着点坐标
    vector<Point2f> coordinates = GetChessCoordinate(cutImg,circlesPos);
    // 对棋子分类,获得对应棋种索引
    vector<int> chessCategories = ClassifyChessImg(chessImgs);
    // 获取棋盘10x9个网格点状态
    vector<vector<int>> gridInfo = GetGridInfo(coordinates,chessCategories);

    WriteChess(cutImg,circlesPos,chessCategories);
    return gridInfo;
}

vector<int> tmp(const vector<Mat> chessImgs)
{
    vector<int> chessCategories; Mat dst;
    vector<string> chess_list = {"黑士", "红仕", "黑象", "红相", "黑炮", "红炮", "黑将",\
                                 "红帅", "黑马", "红马", "黑卒", "红兵", "黑車", "红車"};
    dnn::Net net = dnn::readNetFromONNX("C:\\Users\\asus\\Desktop\\ChessDataset\\myModel_tmp.onnx");
    net.setPreferableBackend(dnn::DNN_BACKEND_CUDA); net.setPreferableTarget(dnn::DNN_TARGET_CUDA);

    for(const Mat &img : chessImgs)
    {
        resize(img,dst,Size(H,W),INTER_CUBIC);
        auto blob = dnn::blobFromImage(dst, 2./255, Size(H, W), Scalar(127, 127, 127), true, false);
        net.setInput(blob,"input");
        Mat output = net.forward("output");

        float maxVal = -1e37; int index = -1;
        for(int i=0;i<output.cols;++i)
        {
            if(output.at<float>(0,i) > maxVal)
            {
                maxVal = output.at<float>(0,i);
                index = i;
            }
        }
        chessCategories.push_back(index);
    }
    return chessCategories;
}


