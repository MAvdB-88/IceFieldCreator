// MIT License

// Copyright (c) 2020 Marnix van den Berg <m.a.vdberg88@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "GUI.h"
#include "BodyVec.h"
#include "Body.h"
#include "Shape.h"
#include "DesktopResolution.h"
#include "Contact.h"
#include "ErrorLogger.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

enum class Color { white, green };


namespace
{

static bool g_acceptEvent(true); //Used to make sure that mouse moves do not override click events.

static void trackBarCallBack(int, void*) {}

void mouseCallback(int event, int x, int y, int flags, void* param)
{
    GUI::CVMouseInfo* mInfo = (GUI::CVMouseInfo*)param;

    if (event == cv::EVENT_LBUTTONUP) //Always accept release.
    {
        g_acceptEvent = false;
        mInfo->event = event;
        mInfo->x = x;
        mInfo->y = y;
    }
    if (event == cv::EVENT_LBUTTONDOWN && g_acceptEvent) //can't override mouse release.
    {
        g_acceptEvent = false;
        mInfo->event = event;
        mInfo->x = x;
        mInfo->y = y;
    }
    if (event == cv::EVENT_MOUSEMOVE && g_acceptEvent) //is overwritten by left press or release.
    {
        mInfo->event = event;
        mInfo->x = x;
        mInfo->y = y;
    }
};

}


GUI::GUI():
    m_scaling(0.0),
    m_windowHeight(0),
    m_windowWidth(0),
    m_centeringDistH(0),
    m_centeringDistV(0),
    m_sliderPosMargin(1000),
    m_sliderMaxMargin(1000),
    m_sliderPosDisp(500),
    m_sliderMaxDisp(1000)
{}


GUI::~GUI() = default;


bool GUI::init(Vector2 domainSize)
{
    if (domainSize.x() < DBL_EPSILON || domainSize.y() < DBL_EPSILON)
    {
        ErrLog::log("Error: Domain size cannot be 0.");
        return false;
    }

    int hPix(0), vPix(0);
    desktopRes::getDesktopResolution(&hPix, &vPix);
    vPix -= 100; //To make sure bottom of window is not behind task bar.

    Vector2 size = domainSize*1.01; //Show a bit of the boundary bodies.
    m_offset = 0.5 * size;  //By default domain is from -0.5*size to 0.5*size. cvMat starts at 0.

    //Check if the height or the with of the desktop is governing for initial window size.
    double desktopRatio = double(hPix) / double(vPix);
    double sizeRatio = size.x() / size.y();

    if (desktopRatio > sizeRatio)
    {
        m_windowHeight = vPix;
        m_scaling = m_windowHeight / size.y();
        m_centeringDistV = 0;
        m_windowWidth = vPix * int(sizeRatio);
        m_centeringDistH = (hPix - m_windowWidth) / 2;
    }
    else
    {
        m_scaling = hPix / size.y();
        m_windowWidth = hPix;
        m_windowHeight = hPix / int(sizeRatio);
        m_centeringDistV = (vPix - m_windowHeight) / 2;
    }


    //Create and center the window:
    cv::namedWindow("Ice Field Creator", cv::WINDOW_NORMAL);
    cv::resizeWindow("Ice Field Creator", m_windowHeight, m_windowWidth);
    cv::moveWindow("Ice Field Creator", m_centeringDistH, m_centeringDistV);

    //Set mouse callback and create trackbars:
    cv::setMouseCallback("Ice Field Creator", mouseCallback, &m_cvMouseInfo);
    cv::createTrackbar("maxDisp", "Ice Field Creator", &m_sliderPosDisp, m_sliderMaxDisp, trackBarCallBack);
    cv::createTrackbar("colMargin", "Ice Field Creator", &m_sliderPosMargin, m_sliderMaxMargin, trackBarCallBack);

    return true;
}


UserInput GUI::draw(const BodyVec& bodies, const std::vector<Contact>& contacts, double maxPenetration, bool penBelowLimit) //, const std::vector<Contact>& contacts
{   
    //Init image matrix and set background color to light blue;
    m_image = std::make_unique<cv::Mat> (cv::Mat::zeros(m_windowHeight, m_windowWidth, CV_8UC3));
    m_image->setTo(cv::Scalar(255, 200, 0));

    drawBodies(bodies);
    drawContacts(contacts);

    //Write maximum penentration to window. Red is above limit set in input file, green if below.
    std::string str = "Max penetration: " + std::to_string(maxPenetration);
    cv::String cvstr(str.c_str());

    cv::Scalar color;
    if (penBelowLimit) color = cv::Scalar(0, 255, 0); //green
    else color = cv::Scalar(0, 0, 255); //red

    cv::putText(*m_image, cvstr, cv::Point(30, 50),
        cv::FONT_HERSHEY_SIMPLEX, 1, color, 2, cv::LINE_AA);

    g_acceptEvent = true; //Accept new mouse click events (see mouseCallBack)

    //Plot scene:
    if (cv::getWindowProperty("Ice Field Creator", cv::WND_PROP_VISIBLE))
    {
        cv::imshow("Ice Field Creator", *m_image);
        cv::waitKey(1);
    }

    return simDomainUserInput();
}

//Creates userInput struct for output to main simulation.
UserInput GUI::simDomainUserInput()
{
    UserInput userInput;
    UserInputMouse& mouseInput = userInput.mouseInput;

    //Scale mouse coordinates from pixels to domain, translate to openCV independent enums.
    if (m_cvMouseInfo.event == cv::EVENT_LBUTTONDOWN)
    {
        mouseInput.mouseFlag = MouseEvent::leftDown;

        double xw = double(m_cvMouseInfo.x) / m_scaling - m_offset.x();
        double yw = double(m_cvMouseInfo.y) / m_scaling - m_offset.y();

        mouseInput.mouseLocation = Vector2(xw, yw);
    }
    if (m_cvMouseInfo.event == cv::EVENT_LBUTTONUP)
    {
        mouseInput.mouseFlag = MouseEvent::leftUp;

        double xw = double(m_cvMouseInfo.x) / m_scaling - m_offset.x();
        double yw = double(m_cvMouseInfo.y) / m_scaling - m_offset.y();

        mouseInput.mouseLocation = Vector2(xw, yw);
    }
    if (m_cvMouseInfo.event == cv::EVENT_MOUSEMOVE)
    {
        mouseInput.mouseFlag = MouseEvent::move;

        double xw = double(m_cvMouseInfo.x) / m_scaling - m_offset.x();
        double yw = double(m_cvMouseInfo.y) / m_scaling - m_offset.y();

        mouseInput.mouseLocation = Vector2(xw, yw);
    }

    userInput.collisionMarginRatio = (double)m_sliderPosMargin /m_sliderMaxMargin;
    userInput.maxDisplacementRatio = (double)m_sliderPosDisp / m_sliderMaxDisp;

    if (!cv::getWindowProperty("Ice Field Creator", cv::WND_PROP_VISIBLE))
    {
        userInput.windowClose = true;
    }

    return userInput;
}


void GUI::drawBodies(const BodyVec& bodies)
{
    for (int i = 0; i < bodies.size(); i++)
    {
        const Body& body = bodies[i];

        Color color;
        if (body.isStatic()) color = Color::green;
        else color = Color::white;

        std::vector<Vector2> globalPoints = body.globalPoints();
        drawPoly(globalPoints,&color);
    }
}

void GUI::drawContacts(const std::vector<Contact>& contacts)
{
    for (int i = 0; i < contacts.size(); i++)
    {
        drawContact(contacts[i]);
    }
}


void GUI::drawContact(const Contact& contact)
{
    Vector2 normal = contact.normal();
    Vector2 contactPoint = contact.contactPoint();
    double halfPenetration = contact.penetration()/2.0;

    //Plot contact point (where contact impulses are applied)
    {
        Vector2 p = contactPoint + m_offset;
        cv::Point cvP(int(m_scaling * p.x()), int(m_scaling * p.y()));
        cv::circle(*m_image, cvP, 3, (255, 0, 0)); //blue
    }

    //Plot contact normal, with length equal to penetration depth:
    {
        std::vector<cv::Point> normLine(2);
        {
            Vector2 p = contactPoint - normal* halfPenetration;
            p += m_offset;
            cv::Point cvP(int(m_scaling * p.x()), int(m_scaling * p.y()));
            normLine[0] = cvP;
        }
        {
            Vector2 p = contactPoint + normal* halfPenetration;
            p += m_offset;
            cv::Point cvP(int(m_scaling * p.x()), int(m_scaling * p.y()));
            normLine[1] = cvP;
        }
        cv::polylines(*m_image, normLine, true, cv::Scalar(255, 0, 0)); //blue
    }

}


void GUI::drawPoly(const std::vector<Vector2>& poly, Color* color)
{
    std::vector<cv::Point> cvPointsVec;

    for (int i = 0; i < poly.size(); i++)
    {
        Vector2 p = poly[i] + m_offset;
        cv::Point cvP(int(m_scaling*p.x()), int(m_scaling * p.y()));
        cvPointsVec.push_back(cvP);
    }

    int lineType = cv::LINE_8;

    const cv::Point* ppt[1] = { &cvPointsVec[0] };
    size_t npt[] = { cvPointsVec.size() };

    cv::Scalar col;
    if (*color == Color::white) col = cv::Scalar(255, 255, 255);
    else col = cv::Scalar(0, 255, 0); //green

    fillConvexPoly(*m_image,
        cvPointsVec,
        col,
        lineType);

    cv::polylines(*m_image, cvPointsVec, true, cv::Scalar(0, 0, 0)); //black polygon outlines
}
