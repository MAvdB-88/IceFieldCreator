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

#include "Vector2.h"
#include "InputData.h" //defines settings.

class BodyVec;
class GUI;
class CollisionDetector;
struct UserInputMouse;

class IceFieldCreator
{
public:
    IceFieldCreator();
    ~IceFieldCreator();

    bool createIceField(const InputData& inputData);

    const BodyVec& bodyVec() const { return *m_bodyVec; }

private:

    bool initGui(Vector2 domainSize);
    bool initBoundary(Vector2 domainSize);
    bool initBodies(const InputData& inputData);

    void solveOverlaps();


    void   handleMouseEvents(const UserInputMouse& mouseInput);
    double handleColMarginInput(double marginRatio, double prevMarginRatio);

    std::shared_ptr<BodyVec>           m_bodyVec; //shared with collisionDetector
    std::unique_ptr<GUI>               m_gui;
    std::unique_ptr<CollisionDetector> m_collisionDetector;

    Settings                           m_settings;
    int                                m_heldBody; //Body currently being held by the used (by a pressed left mouse button)
};