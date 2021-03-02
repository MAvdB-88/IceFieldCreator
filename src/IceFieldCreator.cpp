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

//For definition of M_PI:
#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>

#include "Contact.h"
#include "BodyVec.h"
#include "ErrorLogger.h"
#include "IceFieldCreator.h"
#include "InputData.h"
#include "CollisionDetector.h"
#include "Solver.h"
#include "Body.h"
#include "UserInput.h"
#include "GUI.h"

IceFieldCreator::IceFieldCreator() :
    m_heldBody(-1),
    m_bodyVec{ std::make_shared<BodyVec>() }
{}


IceFieldCreator::~IceFieldCreator() = default;


bool IceFieldCreator::createIceField(const InputData& inputData)
{
    m_settings = inputData.settings;

    if (!initBoundary(inputData.domainSize)) return false;

    if (!initBodies(inputData)) return false;

    if (!initGui(inputData.domainSize)) return false;

    m_collisionDetector = std::make_unique<CollisionDetector>(m_bodyVec);

    solveOverlaps();

    return true;
}


bool IceFieldCreator::initGui(Vector2 domainSize)
{
    m_gui = std::make_unique<GUI>();
    if (!m_gui->init(domainSize)) return false;
    return true;
}


//TODO: change to boundary polygon, and make specialized boundary-shape collision algorithm.
bool IceFieldCreator::initBoundary(Vector2 domainSize)
{
    if (domainSize.x() < DBL_EPSILON || domainSize.y() < DBL_EPSILON)
    {
        ErrLog::log("Error: Domain size cannot be 0.");
        return false;
    }

    double halfSizeX = 0.5 * domainSize.x();
    double halfSizeY = 0.5 * domainSize.y();

    double halfWallThickness = domainSize.x();
    double wallThickness = domainSize.y();

    //Initialise static bodies to form the boundary:
    {
        std::vector<Vector2> wallPointsY(4);
        wallPointsY[0] = Vector2(-halfSizeX, -halfWallThickness);
        wallPointsY[1] = Vector2(halfSizeX, -halfWallThickness);
        wallPointsY[2] = Vector2(halfSizeX, halfWallThickness);
        wallPointsY[3] = Vector2(-halfSizeX, halfWallThickness);

        {
            Shape wallShapeY;
            wallShapeY.init(wallPointsY); //Never fails in current usage (initializing rectangular shape) so no need to check the return bolean.
            wallShapeY.setDefaultNormalDirection(Vector2(0.0, 1.0));
            wallShapeY.setCollisionMargin(m_settings.maxCollisionMargin);

            Vector2 position(0.0, -halfSizeY - halfWallThickness);
            Transform trans(position, 0.0);
            double density = 0.0;
            Body wallBodyY(trans, std::move(wallShapeY), density);
            m_bodyVec->addBody(std::move(wallBodyY));
        }

        {
            Shape wallShapeY;
            wallShapeY.init(wallPointsY); //Never fails in current usage (initializing rectangular shape) so no need to check the return bolean.
            wallShapeY.setDefaultNormalDirection(Vector2(0.0, -1.0));

            Vector2 position(0.0, halfSizeY + halfWallThickness);
            Transform trans(position, 0.0);
            double density = 0.0;
            Body wallBodyY(trans, std::move(wallShapeY), density);
            m_bodyVec->addBody(std::move(wallBodyY));
        }
    }
    {
        std::vector<Vector2> wallPointsX(4);
        wallPointsX[0] = Vector2(-halfWallThickness, -halfSizeY - wallThickness);
        wallPointsX[1] = Vector2(halfWallThickness, -halfSizeY - wallThickness);
        wallPointsX[2] = Vector2(halfWallThickness, halfSizeY + wallThickness);
        wallPointsX[3] = Vector2(-halfWallThickness, halfSizeY + wallThickness);

        {
            Shape wallShapeX;
            wallShapeX.init(wallPointsX);
            wallShapeX.setDefaultNormalDirection(Vector2(1.0, 0.0));
            wallShapeX.setCollisionMargin(m_settings.maxCollisionMargin);

            Vector2 position(-halfSizeX - halfWallThickness, 0.0);
            Transform trans(position, 0.0);
            double density = 0.0;
            Body wallBodyX(trans, std::move(wallShapeX), density);
            m_bodyVec->addBody(std::move(wallBodyX));
        }

        {
            Shape wallShapeX;
            wallShapeX.init(wallPointsX);
            wallShapeX.setDefaultNormalDirection(Vector2(-1.0, 0.0));
            wallShapeX.setCollisionMargin(m_settings.maxCollisionMargin);

            Vector2 position(halfSizeX + halfWallThickness, 0.0);
            Transform trans(position, 0.0);
            double density = 0.0;
            Body wallBodyX(trans, std::move(wallShapeX), density);
            m_bodyVec->addBody(std::move(wallBodyX));
        }
    }

    return true;
}





bool IceFieldCreator::initBodies(const InputData& inputData)
{
    const std::vector<std::vector<Vector2>>& bodyPointsVec = inputData.bodyPointsVec;
    const Settings& settings = inputData.settings;

    double collisionMargin = settings.maxCollisionMargin;

    Vector2 size = inputData.domainSize;
    Vector2 min = -0.5 * size;

    double concentration = inputData.concentration;
    double neededArea = size.x() * size.y() * concentration / 100.0;

    int nPolygons = int(bodyPointsVec.size());
    double addedArea(0.0);

    while (addedArea < neededArea) //Add floes until we reach the desired concentration
    {
        int randomShapeIdx = int((double)nPolygons * (double)rand() / RAND_MAX - 0.01);
        const std::vector<Vector2>& polygon = bodyPointsVec[randomShapeIdx];

        Shape shape;
        if (!shape.init(polygon))
        {
            ErrLog::log("Error: Failed to create shape from polygon vector at index: " + std::to_string(randomShapeIdx));
            return false;
        }
        shape.setCollisionMargin(collisionMargin);

        double orientation = (double)rand() / RAND_MAX * 2.0 * M_PI - M_PI;
        double xCor = min.x() + (double)rand() / RAND_MAX * size.x();
        double yCor = min.y() + (double)rand() / RAND_MAX * size.y();

        Transform trans(Vector2(xCor, yCor), orientation);

        addedArea += shape.calcArea();

        Body body(trans, std::move(shape));
        m_bodyVec->addBody(std::move(body));
    }

    return true;
}


void IceFieldCreator::solveOverlaps()
{

    SolverSettings solverSettings;

    solverSettings.maxIterations = m_settings.maxSolverIterations;
    solverSettings.maxPenetrationError = m_settings.maxResidualPenetration / 2.0;
    solverSettings.maxDisplacement = HUGE_VAL;

    double collisionMarginRatio(1.0);

    Solver solver;

    double maxPenetration(HUGE_VAL);
    while (true)
    {
        //Each penetration iteration, the overlaps at the contacts detected in that iteration step are resolved.
        //Overlaps are resolved implicitly, considering the complete network of overlapping bodies.
        //Multiple iteration steps are needed for the following reasons:
        //- Solving the overlaps will lead to new overlaps at contacts that were not present in the previous step. 
        //- The rotational displacement is linearized in the impulse solver, leading to numerical errors for larger rotations. 
        //  These need to be corrected in further iteration steps.
        //- The impulse solver runs with a fixed maximum number of iterations and may stop before a solution is found.
        //- Body displacements and rotations have an upper limit, which may prevent complete resolution of all overlaps.

        std::vector<Contact> contacts = m_collisionDetector->getContacts(&maxPenetration);

        //Visualize current state and retrieve used input:
        bool penBelowLimit(false);
        if (maxPenetration < m_settings.maxResidualPenetration) penBelowLimit = true;
        UserInput userInput = m_gui->draw(*m_bodyVec, contacts, maxPenetration, penBelowLimit);

        //Handle mouse input:
        handleMouseEvents(userInput.mouseInput);

        //Detect and apply changes to the collisionMargins:
        collisionMarginRatio = handleColMarginInput(userInput.collisionMarginRatio, collisionMarginRatio);

        //Apply max displacement settings:
        solverSettings.maxDisplacement = userInput.maxDisplacementRatio * maxPenetration;
        solverSettings.maxRotation = 0.2 * userInput.maxDisplacementRatio;

        if (userInput.windowClose)
        {
            break; //Stop on windowClose
        }

        //Solve overlaps at current contacts and update body positions:
        solver.solveOverlaps(m_bodyVec.get(), contacts, solverSettings);
    }
}


void   IceFieldCreator::handleMouseEvents(const UserInputMouse& mouseInput)
{
    if (mouseInput.mouseFlag == MouseEvent::leftDown)
    {
        int bodyIdx = m_collisionDetector->bodyAtPos(mouseInput.mouseLocation);
        if (bodyIdx >= 0)
        {
            Body& body = m_bodyVec->at(bodyIdx);
            body.hold();
            m_heldBody = bodyIdx;
        }
    }
    if (mouseInput.mouseFlag == MouseEvent::leftUp)
    {
        if (m_heldBody >= 0)
        {
            m_bodyVec->at(m_heldBody).release();
            m_heldBody = -1;
        }
    }
    if (mouseInput.mouseFlag == MouseEvent::move)
    {
        if (m_heldBody >= 0)
        {
            m_bodyVec->at(m_heldBody).setPosition(mouseInput.mouseLocation);
        }
    }
}


double   IceFieldCreator::handleColMarginInput(double marginRatio, double prevMarginRatio)
{
    if (std::abs(marginRatio - prevMarginRatio) > DBL_EPSILON)
    {
        double margin = m_settings.maxCollisionMargin * marginRatio;

        for (int i = 0; i < m_bodyVec->size(); i++)
        {
            Body& body = m_bodyVec->at(i);
            body.setMargin(margin);
        }
    }

    return marginRatio;
}


