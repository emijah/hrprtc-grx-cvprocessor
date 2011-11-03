// -*- C++ -*-
/*!
 * @file  CvProcessor.cpp * @brief Sequence OutPort component * $Date$
 *
 * $Id$
 */
#include <opencv/highgui.h>
#include "CvProcessor.h"

// Module specification
// <rtc-template block="module_spec">
static const char* cvprocessor_spec[] =
{
    "implementation_id", "CvProcessor",
    "type_name",         "CvProcessor",
    "description",       "Sequence OutPort component",
    "version",           "1.0",
    "vendor",            "General Robotix,Inc.",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.H_max", "35",
    "conf.default.H_min", "15",
    "conf.default.S_max", "256",
    "conf.default.S_min", "100",
    "conf.default.V_max", "256",
    "conf.default.V_min", "150",
    "conf.default.HoughCircles_radius_max", "200",
    "conf.default.HoughCircles_radius_min",  "20",
    //"conf.default.HoughCircles_minDist",   "200",
    "conf.default.HoughCircles_Param1",    "100",
    "conf.default.HoughCircles_Param2",    "40",
    //
    "conf.orange.H_max", "25",
    "conf.orange.H_min", "15",
    "conf.orange.S_max", "256",
    "conf.orange.S_min", "100",
    "conf.orange.V_max", "256",
    "conf.orange.V_min", "150",
    "conf.orange.HoughCircles_radius_max", "200",
    "conf.orange.HoughCircles_radius_min",  "20",
    //"conf.orange.HoughCircles_minDist",   "200",
    "conf.orange.HoughCircles_Param1",    "100",
    "conf.orange.HoughCircles_Param2",    "40",
    //
    "conf.blue.H_max", "130",
    "conf.blue.H_min", "90",
    "conf.blue.S_max", "256",
    "conf.blue.S_min", "100",
    "conf.blue.V_max", "256",
    "conf.blue.V_min", "100",
    "conf.blue.HoughCircles_radius_max", "200",
    "conf.blue.HoughCircles_radius_min",  "20",
    //"conf.blue.HoughCircles_minDist",   "200",
    "conf.blue.HoughCircles_Param1",    "100",
    "conf.blue.HoughCircles_Param2",    "40",
    //
    "conf.white.H_max", "256",
    "conf.white.H_min", "0",
    "conf.white.S_max", "50",
    "conf.white.S_min", "0",
    "conf.white.V_max", "255",
    "conf.white.V_min", "170",
    "conf.white.HoughCircles_radius_max", "200",
    "conf.white.HoughCircles_radius_min",  "20",
    //"conf.white.HoughCircles_minDist",   "200",
    "conf.white.HoughCircles_Param1",    "100",
    "conf.white.HoughCircles_Param2",    "40",
    ""
};
// </rtc-template>

CvProcessor::CvProcessor(RTC::Manager* manager)
// <rtc-template block="initializer">
    : RTC::DataFlowComponentBase(manager),
    m_MultiCameraImageIn("MultiCameraImage", m_MultiCameraImage),
    m_MultiCameraImageOut("MultiCameraImage", m_MultiCameraImage),
    m_CvProcessorServicePort("CvProcessorService")

// </rtc-template>
{
    m_service0.setComponent(this);
}

CvProcessor::~CvProcessor()
{
}


RTC::ReturnCode_t CvProcessor::onInitialize()
{
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("MultiCameraImage", m_MultiCameraImageIn);

    // Set OutPort buffer
    addOutPort("MultiCameraImage", m_MultiCameraImageOut);

    // Set service provider to Ports

    // Set service consumers to Ports

    // Set CORBA Service Ports

    // </rtc-template>

    // <rtc-template block="bind_config">
    // Bind variables and configuration variable

    // </rtc-template>
    m_CvProcessorServicePort.registerProvider("service0", "CvProcessorService", m_service0);
    addPort(m_CvProcessorServicePort);

    m_frame = 0;
    m_hsv_frame = 0;
    m_thresholded = 0;
    m_storage = cvCreateMemStorage(0);

    bindParameter("H_min", m_H_min, "0");
    bindParameter("S_min", m_S_min, "0");
    bindParameter("V_min", m_V_min, "0");

    bindParameter("H_max", m_H_max, "255");
    bindParameter("S_max", m_S_max, "255");
    bindParameter("V_max", m_V_max, "255");

    bindParameter("HoughCircles_radius_min", m_CircleRadius_min,  "20");
    bindParameter("HoughCircles_radius_max", m_CircleRadius_max, "200");
    //bindParameter("HoughCircles_minDist",    m_distance_min,     "200");
    bindParameter("HoughCircles_Param1",     m_param1,           "100");
    bindParameter("HoughCircles_Param2",     m_param2,            "40");

    cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("dst", CV_WINDOW_AUTOSIZE);

    return RTC::RTC_OK;
}


/*
   RTC::ReturnCode_t CvProcessor::onFinalize()
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onStartup(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onShutdown(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onActivated(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */

RTC::ReturnCode_t CvProcessor::onDeactivated(RTC::UniqueId ec_id)
{
    cvDestroyWindow("src");
    cvDestroyWindow("dst");
    cvReleaseImage(&m_frame);
    cvReleaseImage(&m_hsv_frame);
    cvReleaseImage(&m_thresholded);
    return RTC::RTC_OK;
}


RTC::ReturnCode_t CvProcessor::onExecute(RTC::UniqueId ec_id)
{
    if (m_MultiCameraImageIn.isNew()) {
        m_MultiCameraImageIn.read();
        if (m_MultiCameraImage.error_code != 0) {
            return RTC::RTC_OK;
        }
    }
    return RTC::RTC_OK;
}

void CvProcessor::HoughCircles()
{
    CvScalar hsv_min = cvScalar(m_H_min, m_S_min, m_V_min, 0);
    CvScalar hsv_max = cvScalar(m_H_max, m_S_max, m_V_max, 0);

    for (int i=0; i<m_MultiCameraImage.data.image_seq.length(); i++) {
        Img::ImageData& idat = m_MultiCameraImage.data.image_seq[i].image;
        if (m_frame == 0) {
            m_frame       = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 3);
            m_hsv_frame   = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 3);
            m_thresholded = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 1);
        }
        for (int row=0; row<idat.height; ++row) {
            for (int col=0; col<idat.width; ++col) {
                for (int k=0; k<3; ++k) {
                    ((unsigned char *)(m_frame->imageData))[row * m_frame->widthStep + col*3 + k]
                        = idat.raw_data[(row * idat.width + col)*3+k];
                }
            }
        }
        cvCvtColor(m_frame, m_hsv_frame, CV_BGR2HSV);
        cvInRangeS(m_hsv_frame, hsv_min, hsv_max, m_thresholded);

        m_circles = cvHoughCircles(m_thresholded, m_storage, CV_HOUGH_GRADIENT, 2, m_thresholded->height/4, m_param1, m_param2, m_CircleRadius_min, m_CircleRadius_max);
        for (int j=0; j<m_circles->total; j++) {
            float* p = (float*)cvGetSeqElem( m_circles, j );
            cvCircle( m_frame, cvPoint(cvRound(p[0]),cvRound(p[1])), 3,             CV_RGB(0,255,0), -1, 8, 0 );
            cvCircle( m_frame, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0),  3, 8, 0 );
        }
        cvShowImage("src", m_frame);
        cvShowImage("dst", m_thresholded);
        char c = cvWaitKey(10);
    }
}

void CvProcessor::HoughLinesP()
{
    CvScalar hsv_min = cvScalar(m_H_min, m_S_min, m_V_min, 0);
    CvScalar hsv_max = cvScalar(m_H_max, m_S_max, m_V_max, 0);

    for (int i=0; i<m_MultiCameraImage.data.image_seq.length(); i++) {
        Img::ImageData& idat = m_MultiCameraImage.data.image_seq[i].image;
        if (m_frame == 0) {
            m_frame       = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 3);
            m_hsv_frame   = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 3);
            m_thresholded = cvCreateImage(cvSize(idat.width, idat.height), IPL_DEPTH_8U, 1);
        }
        for (int row=0; row<idat.height; ++row) {
            for (int col=0; col<idat.width; ++col) {
                for (int k=0; k<3; ++k) {
                    ((unsigned char *)(m_frame->imageData))[row * m_frame->widthStep + col*3 + k]
                        = idat.raw_data[(row * idat.width + col)*3+k];
                }
            }
        }
        cvCvtColor(m_frame, m_hsv_frame, CV_BGR2HSV);
        cvInRangeS(m_hsv_frame, hsv_min, hsv_max, m_thresholded);
        cv::Mat mat(m_thresholded); 
        cv::HoughLinesP(mat, m_lines, 3.14/180, 80, 30, 10);
        for (int j=0; j<m_lines.size(); j++) {
	    std::cout << m_lines[j][0] << std::endl;
	    cv::line(mat, cvPoint(m_lines[j][0], m_lines[j][1]), cvPoint(m_lines[j][2], m_lines[j][3]), cv::Scalar(0, 0, 255), 3, 8);
        }
        cvShowImage("src", m_frame);
        cvShowImage("dst", m_thresholded);
        char c = cvWaitKey(10);
    }
}
/*
   RTC::ReturnCode_t CvProcessor::onAborting(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onError(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onReset(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onStateUpdate(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */
/*
   RTC::ReturnCode_t CvProcessor::onRateChanged(RTC::UniqueId ec_id)
   {
   return RTC::RTC_OK;
   }
 */


extern "C"
{

void CvProcessorInit(RTC::Manager* manager)
{
    coil::Properties profile(cvprocessor_spec);
    manager->registerFactory(profile,
                             RTC::Create<CvProcessor>,
                             RTC::Delete<CvProcessor>);
}

};



