// MIT License

// Copyright(c) 2019 Erin Catto http ://www.box2d.org
// --Modified based on box2d source code--
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
#include <unordered_map>

#include "AABB.h"

struct Node
{
    AABB aabb; 	   /// The axis-aligned bounding box.
    int parent;    /// Index of the parent node.
    int next; 	   /// Index of the next node.
    int left;      /// Index of the left-hand child.
    int right;	   /// Index of the right-hand child.
    int height;    /// Height of the node. This is 0 for a leaf and -1 for a free node.
    int particle;  /// The index of the particle that the node contains (leaf nodes only).

    bool isLeaf() const;    //True if leaf node
};


class DynamicAabbTree
{
public:

    DynamicAabbTree(int nParticles = 16);

    bool insertParticle(int id, const AABB& aabb);
    bool removeParticle(int id);
    bool updateParticle(int id, const AABB& aabb);

    std::vector<int> query(int id) const;
    std::vector<int> query(const AABB& aabb) const;

    const AABB& getAABB(int id);

    int getNumParticles() const {return m_particleMap.size(); }
    int nodeCount() const { return m_nodeCount; }

    int getHeight() const;

private:

    std::vector<int> query(int id, const AABB& aabb) const;

    int   allocateNode();
    void  freeNode(int);
    void  insertLeaf(int);
    void  removeLeaf(int);
    int   balance(int);

    std::vector<Node> m_nodes; // The dynamic tree.

    int m_root; // The index of the root node.
    int m_nodeCount; // The current number of nodes in the tree.
    int m_nodeCapacity; // The current node capacity.
    int m_freeList; // The position of node at the top of the free list.

    Vector2 m_boxSize; // The size of the system in each dimension.

    std::unordered_map<int, int> m_particleMap; // A map between particle and node indices.
};
