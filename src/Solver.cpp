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

#include "Solver.h"
#include "Body.h"
#include "SolverConstraint.h"
#include "BodyVec.h"
#include "Contact.h"

//Solver imspired by the btSequentialImpulseSolver from Bullet Physics.
//The same solver can be used for dynamic multi-body simulations or for 
//penetration correction, as is done in this code. It solves the contact
//impulses needed to change the body velocities such that they recover from
//penetration in a unit time step. 

//Body masses, impulses and velocities should be seen as "virtual" 
//values used to recover from penetration. After updating the body positions, 
//all velocities are set to zero for the next iteration step. 
//Terminology is kept as it is because the intention is to extend this code to 
//a simple physics physics engine.

//Solvers in Bullet Physics and Box2d create local compact "solverBody" vectors.
//Since the body vector is already quite compact this is not done in this solver.

Solver::Solver() {}


Solver::~Solver() = default;


void Solver::solveOverlaps(BodyVec* bodies, const std::vector<Contact>& contacts, const SolverSettings& solverSettings)
{
    m_solverSettings = solverSettings;

    initConstraints(*bodies, contacts);
    solve(bodies);
    transformBodies(bodies);

    m_solverConstraints.resize(0);
}


void Solver::initConstraints(const BodyVec& bodies, const std::vector<Contact>& contacts)
{

    m_solverSettings.maxImpulseError = HUGE_VAL;

    for (int i = 0; i < contacts.size(); i++)
    {
        const Contact& contact = contacts[i];

        int bodyIdx0 = contact.bodyIdx0();
        const Body& body0 = bodies[bodyIdx0];

        int bodyIdx1 = contact.bodyIdx1();
        const Body& body1 = bodies[bodyIdx1];

        Vector2 contactPoint = contact.contactPoint();
        Vector2 contactNormal = contact.normal();

        Vector2 relPos0 = contactPoint - body0.transform().position();
        Vector2 relPos1 = contactPoint - body1.transform().position();

        double torqueAxis0 = relPos0.cross(contactNormal);
        double torqueAxis1 = -relPos1.cross(contactNormal);

        double invInertia0 = body0.invInertia();
        double invInertia1 = body1.invInertia();
        
        double invMass0 = body0.invMass();
        double invMass1 = body1.invMass();

        SolverConstraint solverConstraint;

        solverConstraint.bodyIdx0 = bodyIdx0;
        solverConstraint.bodyIdx1 = bodyIdx1;
        solverConstraint.normal0 = contactNormal;
        solverConstraint.normal1 = -contactNormal;
        solverConstraint.relPos0CrossNormal = torqueAxis0;
        solverConstraint.relPos1CrossNormal = torqueAxis1;
        solverConstraint.angularComponent0 = invInertia0 * torqueAxis0;
        solverConstraint.angularComponent1 = invInertia1 * torqueAxis1;
        solverConstraint.linearComponent0 = invMass0 * contactNormal;
        solverConstraint.linearComponent1 = invMass1 * -contactNormal;

        double inverseEffectiveMass0 = solverConstraint.linearComponent0.length()
            + solverConstraint.angularComponent0 * solverConstraint.relPos0CrossNormal;

        double inverseEffectiveMass1 = solverConstraint.linearComponent1.length()
            + solverConstraint.angularComponent1 * solverConstraint.relPos1CrossNormal;

        double inverseEffectiveMass = inverseEffectiveMass0 + inverseEffectiveMass1;

        if (inverseEffectiveMass < DBL_EPSILON) continue; //static-static collision.

        solverConstraint.effectiveMass = 1.0 / inverseEffectiveMass;
        solverConstraint.rhs = contact.penetration()*solverConstraint.effectiveMass;
        solverConstraint.impulse = 0.0;

        double impulseError = m_solverSettings.maxPenetrationError * solverConstraint.effectiveMass;
        if (impulseError < m_solverSettings.maxImpulseError) m_solverSettings.maxImpulseError = impulseError;

        m_solverConstraints.push_back(solverConstraint);
    }

}

void Solver::solve(BodyVec* bodies)
{
    double error(HUGE_VAL);
    double maxError = m_solverSettings.maxImpulseError;
    int nIterations(0);
    int maxIterations = m_solverSettings.maxIterations;

    int numConstraints = int(m_solverConstraints.size());

    BodyVec& bodiesRef = *bodies;

    while (error > maxError && nIterations < maxIterations)
    {
        ++nIterations;
        error = 0.0;

        for (int i = 0; i < numConstraints; i++)
        {
            SolverConstraint& constraint = m_solverConstraints[i];
            Body& body0 = bodiesRef[constraint.bodyIdx0];
            Body& body1 = bodiesRef[constraint.bodyIdx1];

            double deltaImpulse = constraint.rhs;

            const double deltaVel0Dotn = constraint.normal0.dot(body0.linearVelocityChangeImpulse()) 
                + constraint.relPos0CrossNormal*body0.angularVelocityChangeImpulse();
            const double deltaVel1Dotn = constraint.normal1.dot(body1.linearVelocityChangeImpulse()) 
                + constraint.relPos1CrossNormal*body1.angularVelocityChangeImpulse();

            deltaImpulse -= deltaVel0Dotn * constraint.effectiveMass;
            deltaImpulse -= deltaVel1Dotn * constraint.effectiveMass;

            //Normal constraint impulses may not be negative.
            double newImpulse = constraint.impulse + deltaImpulse;
            if (newImpulse < 0.0)
            {
                deltaImpulse = 0.0 - constraint.impulse;
                constraint.impulse = 0.0;
            }
            else
            {
                constraint.impulse = newImpulse;
            }

            if (deltaImpulse > error) error = deltaImpulse;

            body0.applyImpulse(constraint.linearComponent0, constraint.angularComponent0, deltaImpulse);
            body1.applyImpulse(constraint.linearComponent1, constraint.angularComponent1, deltaImpulse);
        }
    }
}


void Solver::transformBodies(BodyVec* bodies)
{
    BodyVec& bodiesRef = *bodies;

    for (int i = 0; i < bodiesRef.size(); i++)
    {
        Body& body = bodiesRef[i];
        body.updatePosition(m_solverSettings.maxDisplacement, m_solverSettings.maxRotation);
    }

}