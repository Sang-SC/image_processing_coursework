#include <opencv2/opencv.hpp>
using namespace cv;
int main()
{
    Mat img = imread("./Lena.jpg");
    imshow("img",img);
    waitKey(0);
    return 0;
}