#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // Open camera 4 using V4L2 backend
    cv::VideoCapture cap(4, cv::CAP_V4L2);

    if (!cap.isOpened())
    {
        std::cerr << "Cannot open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (true)
    {
        bool ret = cap.read(frame);

        if (!ret)
        {
            std::cerr << "Failed to grab frame" << std::endl;
            break;
        }

        cv::imshow("D435", frame);

        // Exit when 'q' is pressed
        if ((cv::waitKey(1) & 0xFF) == 'q')
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
