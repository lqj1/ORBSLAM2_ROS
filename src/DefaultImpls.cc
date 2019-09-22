

#ifdef ENABLE_GUI
#  include "IMapRenderer.h"
#endif

#include "IFrameSubscriber.h"
#include "IFrameDrawer.h"

using namespace ORB_SLAM2;


void IFrameSubscriber::Update(Tracking* tracking)
{
    // Do nothing
}

cv::Mat IFrameRenderer::DrawFrame()
{
    return cv::Mat {0, 0, CV_8UC3, cv::Scalar(0,0,0)};
}


