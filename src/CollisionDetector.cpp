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

#include "CollisionDetector.h"
#include "Body.h"
#include "Contact.h"
#include "BodyVec.h"
#include "ErrorLogger.h"

#include <algorithm>    // std::sort


CollisionDetector::CollisionDetector(std::shared_ptr<BodyVec> bodyVec):
    m_bodyVec(bodyVec)
{
    //The collisionDetector uses the body's position in the bodies vector as index value.
    //This works fine as long as no bodies are removed. 
    //Even body removal would not be a major issue with the current implementation, 
    //but handling contact persistance would require a more advanced method.

    for (int i = 0; i < m_bodyVec->size(); i++)
    {
        const Body& body = m_bodyVec->at(i);
        AABB aabb = body.globalAABBWithMargin();

        //Margins should be added by the body. No margins added in tree!
        m_dynamicAABBTree.insertParticle(i, aabb);
    }
}


std::vector<Contact> CollisionDetector::getContacts(double* maxPenetration)
{
    std::set<IndexPair> potentialContacts = findPotentialContacts(); //broadPhase
    return processContacts(potentialContacts, maxPenetration); //narrowPhase
}


namespace 
{
bool sortHighLow(int i, int j) { return (i > j); }
}


int CollisionDetector::bodyAtPos(Vector2 position) const
{
    AABB aabb(position, position);
    std::vector<int> potentialBodies = m_dynamicAABBTree.query(aabb);

    //Take the body that is last rendered, which is the one with the highest position index
    std::sort(potentialBodies.begin(), potentialBodies.end(), sortHighLow);

    int idxIt(-1), idxOverlap(-1);
    for (int i = 0; i < potentialBodies.size(); i++)
    {
        idxIt = potentialBodies[i];
        const Body& body = m_bodyVec->at(idxIt);
        if (body.overlaps(position))
        {
            idxOverlap = idxIt;
            break;
        }
    }

    return idxOverlap;
}


std::set<IndexPair> CollisionDetector::findPotentialContacts()
{

    //Update the aabb tree:
    for (int i = 0; i < m_bodyVec->size(); i++)
    {
        const Body& body = m_bodyVec->at(i);
        AABB aabb = body.globalAABBWithMargin();

        //Margins should be added by the body. No margins added in tree!
        m_dynamicAABBTree.updateParticle(i, aabb);
    }


    std::set<IndexPair> colPairs; //Using an ordered set ensures determinism.

    //Querying the AABB tree is by far the most time consuming part of the current code.
    //TODO: improve efficiency by fattening the AABBs and only querying the three if a body moves
    //Out of its fattened AABB, similar to Box2D.

    //find body pairs:
    for (int i = 0; i < m_bodyVec->size(); i++)
    {
        std::vector<int> potentialContacts = m_dynamicAABBTree.query(i);

        for (int j = 0; j < potentialContacts.size(); j++)
        {
            int idx = potentialContacts[j];
            colPairs.insert(utils::makePair(i, idx));
        }
    }

    return colPairs;
}


std::vector<Contact> CollisionDetector::processContacts(const std::set<IndexPair>& indexPairs,
    double* maxPenetration)
{
    if (maxPenetration) *maxPenetration = 0.0;

    std::vector<Contact> contacts;

    //Process pairs:
    for (auto it = indexPairs.begin(); it != indexPairs.end(); it++)
    {
        int idx0 = it->first;
        int idx1 = it->second;

        const Body& body0 = m_bodyVec->at(idx0);
        const Body& body1 = m_bodyVec->at(idx1);

        if (body0.isStatic() && body1.isStatic()) continue; //Don't add static-static contacts.

        std::vector<Contact> contactsBodyPair = body0.contactProperties(body1);

        for (int i = 0; i < contactsBodyPair.size(); i++)
        {
            Contact& contact = contactsBodyPair[i];
            contact.setBodyIdx0(idx0);
            contact.setBodyIdx1(idx1);

            if (maxPenetration)
            {
                if (contact.penetration() > *maxPenetration) *maxPenetration = contact.penetration();
            }
        }

        contacts.insert(std::end(contacts), std::begin(contactsBodyPair), std::end(contactsBodyPair));

    }

    return contacts;
}