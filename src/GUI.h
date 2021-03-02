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

#pragma once

#include <vector>
#include <memory>

#include "Vector2.h"
#include "UserInput.h" //userInput is returned from the GUI to the main simulation

class BodyVec;
class Contact;
enum class Color;

namespace cv { class Mat; }


//A very basic GUI based on openCV highGUI. 


class GUI
{
public:
    GUI();
    ~GUI(); //Needed because of forward declaration of cv::Mat

    UserInput draw(const BodyVec& bodies, const std::vector<Contact>& contacts, double maxPenetration, bool penBelowLimit);
    bool init(Vector2 domainSize);

    //Used in mouseCallBack
    struct CVMouseInfo
    {
        CVMouseInfo() : event(-1), x(-1), y(-1) {}
        int event; int x; int y;
    };

private:

    UserInput simDomainUserInput();

    void drawBodies(const BodyVec& bodies);
    void drawContacts(const std::vector<Contact>& contacts);

    void drawPoly(const std::vector<Vector2>& poly, Color* color);
    void drawContact(const Contact& contact);

    double  m_scaling; //Scales to screen resolution.
    Vector2 m_offset; //Transforms from domain coordinates screen pixel location.

    std::unique_ptr<cv::Mat> m_image; //Image pixel matrix

    //Variables adjusted by mouse and sliders:
    CVMouseInfo m_cvMouseInfo;

    int m_sliderMaxMargin;
    int m_sliderPosMargin;
    
    int m_sliderPosDisp;
    int m_sliderMaxDisp;

    //Window size and position on screen
    int m_windowHeight;
    int m_windowWidth;
    int m_centeringDistH;
    int m_centeringDistV;

};