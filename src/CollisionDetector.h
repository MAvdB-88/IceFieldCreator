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
#include <set>
#include <memory>

#include "DynamicAabbTree.h"
#include "Utils.h" //Defines IndexPair

class BodyVec;
class Contact;

class CollisionDetector
{
public:

    CollisionDetector(std::shared_ptr<BodyVec> bodyVec);
    std::vector<Contact> getContacts(double* maxPenetration = NULL);

    int bodyAtPos(Vector2 position);

private:

    //Broadphase
    std::set<IndexPair> findPotentialContacts();

    //Narrowphase
    std::vector<Contact> CollisionDetector::processContacts(const std::set<IndexPair>& indexPairs,
        double* maxPenetration = NULL); 

    //Dynamic AABB tree
    DynamicAabbTree m_dynamicAABBTree;
    std::shared_ptr<BodyVec> m_bodyVec;
};