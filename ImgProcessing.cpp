#include "ImgProcessing.h"
#include "qdebug.h"


// Mat格式的图像转为QImage格式
QImage ImgProcessing::Mat2QImage(const cv::Mat &img)
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
            cv::cvtColor(img,img,cv::COLOR_BGR2RGB);
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
void SortRotatedRectPoints(cv::Point2f vetPoints[], cv::RotatedRect rect)
{
    rect.points(vetPoints);
    cv::Point2f curpoint;
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

int CalPointsDistance(const cv::Point2f&a,const cv::Point2f&b)
{
    return sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2));
}

// 获取轮廓点列表中，最接近外接矩形角点的边界点
void GetContourCorner(cv::Point2f nearPoints[],cv::Point2f rectPoints[],std::vector<cv::Point> contourPoints)
{
    std::vector<float> minDist(4,1e38);
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

void DrawHoughLine(cv::Mat &img,int threshold)
{
    std::vector<cv::Vec4f> lines;
    cv::HoughLinesP(img,lines,5,CV_PI/180,3,5,30);
    for(size_t i=0;i<lines.size();++i)
    {
        cv::Vec4f l=lines[i];
        cv::line(img,cv::Point(l[0],l[1]),cv::Point(l[2],l[3]),cv::Scalar(255,255,255),1,cv::LINE_AA);
    }
    //cv::Canny(img,img,80,127);
}

// 分割图像中棋盘区域    预处理、边缘检测、四边形轮廓拟合、透视变换   参考Vaccae
cv::Mat ImgProcessing::SplitChessBoard(const cv::Mat inImg)
{
    cv::Mat img,out;
    inImg.copyTo(img);
    inImg.copyTo(out);
    // 高斯滤波
    cv::GaussianBlur(img,img,cv::Size(5,5),0.5,0.5);
    //cv::medianBlur(img,img,9);  // 中值滤波

    // 分为BGR三通道
    std::vector<cv::Mat> channels;
    cv::Mat B_img,G_img,R_img,gray_img,sumImg;
    cv::split(img,channels);
    cv::cvtColor(img,gray_img,cv::COLOR_BGR2GRAY);

    //GetCicles(img);

    // <del>直方图均衡化</del> 大津法获取分割阈值，Canny算子边缘检测
    int threshold;
    //cv::equalizeHist(channels[0],channels[0]);
    threshold=CalcMatOTSU(channels[0]);
    cv::Canny(channels[0],B_img,threshold/2,cv::saturate_cast<uchar>(1.3*threshold));
    threshold=CalcMatOTSU(channels[1]);
    cv::Canny(channels[1],G_img,threshold/2,cv::saturate_cast<uchar>(1.3*threshold));
    threshold=CalcMatOTSU(channels[2]);
    cv::Canny(channels[2],R_img,threshold/2,cv::saturate_cast<uchar>(1.3*threshold));

    threshold=CalcMatOTSU(gray_img);
    cv::Canny(gray_img,gray_img,threshold/2,cv::saturate_cast<uchar>(1.3*threshold));

    DrawHoughLine(B_img,0);DrawHoughLine(G_img,0);DrawHoughLine(R_img,0);

    // 三个通道或运算
    cv::bitwise_or(B_img,G_img,sumImg);
    cv::bitwise_or(R_img,sumImg,sumImg);
    //threshold=CalMatOTSU(gray_img);
    //cv::threshold(sumImg,sumImg,threshold,255,cv::THRESH_BINARY_INV);
    //sumImg=~sumImg;
    cv::imshow("CannyGray",sumImg);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(sumImg,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_TC89_KCOS);

    int imgArea=img.size().area(),maxAreaIndex=0;
    float maxArea=0;

    // 遍历识别到的轮廓
    std::vector<std::vector<cv::Point>> squares;
    for(size_t i=0;i<contours.size();++i)
    {
        std::vector<cv::Point> approx;
        // 精度0.02 曲线闭合 道格拉斯-普克算法(Douglas-Peucker)
        cv::approxPolyDP(cv::Mat(contours[i]),approx,cv::arcLength(cv::Mat(contours[i]),true)*0.01,true);
        if(approx.size()!=4 || cv::contourArea(cv::Mat(approx)) < 4000 || !cv::isContourConvex(cv::Mat(approx)))
            continue;
        squares.push_back(approx);
        /*
        // 检测四个角是否接近直角
        double maxCos=0;
        for(int j=2;j<5;++j)
        {
            cv::Point p1=approx[j%4],p2=approx[j-2],p0=approx[j-1];
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
        cv::RotatedRect rotateRect=cv::minAreaRect(contours[i]);
        if(rotateRect.size.area() > maxArea && rotateRect.size.area()>imgArea*0.1)
        {
            maxArea=rotateRect.size.area();
            maxAreaIndex=i;
        }

    }
    qDebug()<<contours.size();
    qDebug()<<maxAreaIndex<<"  "<<maxArea;

    //cv::Mat imgContour = cv::Mat::zeros(sumImg.size(),CV_8SC3);

    // 获取并绘制最小包围矩形
    cv::RotatedRect boundingRect = cv::minAreaRect(contours[maxAreaIndex]);
    cv::Point2f vertices[4];
    SortRotatedRectPoints(vertices, boundingRect);
    for (int k = 0; k < 4; ++k)
    {
        line(out, vertices[k], vertices[(k + 1) % 4], cv::Scalar(255, 0, 0),5);
    }

    // 获取轮廓点列表中距离外接矩形边角最近的4个点，并绘制该4点围成的多边形
    cv::Point2f nearPoints[4];
    GetContourCorner(nearPoints,vertices,contours[maxAreaIndex]);
    for (int k = 0; k < 4; ++k)
    {
        line(out, nearPoints[k], nearPoints[(k + 1) % 4], cv::Scalar(0, 0, 255),1);
    }

    // 计算目标矩形坐标点
    cv::Point2f rectPoints[4];
    float width=CalPointsDistance(vertices[0],vertices[1]);
    float height=CalPointsDistance(vertices[1],vertices[2]);
    //rectPoints[0]=cv::Point2f(nearPoints[0].x,nearPoints[0].y);
    rectPoints[0]=cv::Point2f(vertices[0].x,vertices[0].y);
    rectPoints[1]=rectPoints[0]+cv::Point2f(width,0);
    rectPoints[2]=rectPoints[1]+cv::Point2f(0,height);
    rectPoints[3]=rectPoints[0]+cv::Point2f(0,height);
    for (int k = 0; k < 4; ++k)
    {
        line(out, rectPoints[k], rectPoints[(k + 1) % 4], cv::Scalar(0, 255, 255),3);
    }
    cv::imshow("rect img",out);

    // 计算透视变换矩阵
    cv::Mat transformImg;
    cv::Mat transformMat=cv::getPerspectiveTransform(nearPoints,rectPoints);
    cv::warpPerspective(img,transformImg,transformMat,transformImg.size(),cv::INTER_LINEAR);
    //imshow("perspect img",transformImg);

    // 裁切棋盘图像
    qDebug()<<"裁切区域:"<<rectPoints[0].x<<","<<rectPoints[0].y<<"  "<<rectPoints[2].x<<","<<rectPoints[2].y;
    //cv::Mat cutImg=transformImg(cv::Rect(rectPoints[0],rectPoints[2]));
    cv::Mat cutImg=transformImg(cv::Rect(rectPoints[0]-cv::Point2f(width*0.05,height*0.05),rectPoints[2]+cv::Point2f(width*0.05,height*0.05)));
    //cv::imshow("cut img",cutImg);

    std::vector<cv::Vec3f> circles = GetCirclesPos(cutImg);
    GetCirclesImg(cutImg,circles);

    /*
    // 绘制所有矩形
    for (size_t i = 0; i < contours.size(); i++)
    {
        const cv::Point* p = &contours[i][0];

        int n = (int)contours[i].size();
        if (p->x > 3 && p->y > 3)
        {
            polylines(out, &p, &n, 1, true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
            //polylines(out, squares, true, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
    }
    */

    //cv::imshow("imgContour",out);
    return out;
}

// 最大类间方差法(大津OTSU法)求自适应分割阈值
int ImgProcessing::CalcMatOTSU(cv::Mat &img)
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

// 对图像进行霍夫圆检测，返回检测到的circles
std::vector<cv::Vec3f> ImgProcessing::GetCirclesPos(const cv::Mat &img)
{
    // 预处理:滤波(中值/高斯),转为灰度图,获取分割阈值
    std::vector<cv::Vec3f> circles;
    cv::Mat tmp,tmp_gray;
    img.copyTo(tmp);
    cv::medianBlur(tmp,tmp,1);
    cv::cvtColor(tmp,tmp_gray,cv::COLOR_BGR2GRAY);

    int threshold=CalcMatOTSU(tmp_gray);

    double dp=2,minDist=20;
    double param=80; // 检测阶段圆心的累加器阈值。值越小，越可以检测到更多不存在的圆
    int minRadius=10,maxRadius=30;
    //do{
        circles.clear();
        cv::HoughCircles(tmp_gray,circles,cv::HOUGH_GRADIENT,dp,minDist,threshold,param,minRadius,maxRadius);
        qDebug()<<circles.size()<<"param:"<<param;
    //}while(circles.size()>32);

    for(size_t i=0;i<circles.size();++i)
    {
        cv::circle(tmp,cv::Point(circles[i][0],circles[i][1]),circles[i][2],cv::Scalar(0,255,0),2,8);
        //cv::circle(tmp,cv::Point(circles[i][0],circles[i][1]),10,cv::Scalar(0,255,0),-1,8);
    }
    cv::imshow("Hough Circle",tmp);

    return circles;
}

// 截取圆形区域图像，并resize
std::vector<cv::Mat> ImgProcessing::GetCirclesImg(const cv::Mat &inImg, std::vector<cv::Vec3f> &circles)
{
    std::vector<cv::Mat> circleImgs;
    for(size_t i=0;i<circles.size();++i)
    {
        cv::Mat img;
        cv::Mat mask=cv::Mat::zeros(inImg.size(),CV_8UC3);
        cv::circle(mask,cv::Point(circles[i][0],circles[i][1]),circles[i][2],cv::Scalar(255,255,255),-1);
        inImg.copyTo(img,mask);
        cv::Point2f upleft(circles[i][0]-1.1*circles[i][2],circles[i][1]-1.1*circles[i][2]);
        cv::Point2f lowright(circles[i][0]+1.1*circles[i][2],circles[i][1]+1.1*circles[i][2]);
        cv::Mat cutImg=img(cv::Rect(upleft,lowright));

        cv::resize(cutImg,cutImg,cv::Size(64,64),0,0,cv::INTER_CUBIC);
        circleImgs.push_back(cutImg);
        //cv::imshow("im:"+std::to_string(i),cutImg);

        //cv::imwrite("C:\\Users\\asus\\Desktop\\ChessDataset\\origin\\64x_"+std::to_string(i)+".jpeg",cutImg);
    }
    return circleImgs;
}



