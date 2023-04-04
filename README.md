# 象棋机器人QT上位机



### 更新记录

* 3月21日

  > 1. 完成串口通信基本功能，可以发送电机转动一定角度、持续转动等命令；
  > 2. 初步完成图像处理-棋盘分割部分，背景干净且与棋盘对比度大时才比较稳定，Unity的测试画面基本没问题；
  > 3. 下一步：串口接收并解析电机返回数据流；获取USB相机视频流和unity虚拟相机视频流；霍夫曼圆检测出单个棋子，定位并分类。

* 3月28日

  > 完成棋盘边缘检测、透视变换、裁剪、霍夫圆检测。
  >
  > 下一步：霍夫圆检测函数参数调整；裁剪棋子图像，制作训练集；与Unity通讯。

* 3月31日

  > 用棋盘占据画面中心大部分区域的720p图像测试，圆检测的效果还算可以。
  >
  > 裁剪出棋子圆形区域图像，周围为黑色。
  
* 4月3日

  > 基本完成棋子图像处理部分，主要内容：
  >
  > 1.整理ImgProcessing类成员函数，主要通过MainFunc()调用；
  >
  > 2.调用训练完成的onnx格式模型，对棋子识别分类；
  >
  > 3.最终输出棋盘10x9个理想着点的着棋情况。

  > 图像处理部分留下的坑：
  >
  > 1.棋盘检测效果一般，对真实图像处理能力差，可能还需要手动框选；
  >
  > 2.霍夫圆检测目前效果还行，看后面实际使用起来如何，最好能手动设置参数，实在不行换其他图像处理算法；
  >
  > 3.棋盘网格分割识别部分，要保证前面棋盘检测足够稳定才行，横纵向的比例目前还是手动设置，之后或许可以加入识别算法（类似围棋棋盘单目标定？根据棋子位置模糊确定网格？）。
  
* 4月4日

  > 完成Tcp客户端接收图像部分，但速度好像有点慢，不知道是因为push_back还是没开多线程
