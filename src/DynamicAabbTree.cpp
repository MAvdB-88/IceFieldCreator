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

#include "DynamicAabbTree.h"
#include "ErrorLogger.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
  // Null node flag.
  const int g_nullNode = 0xffffffff;
}

bool Node::isLeaf() const
{
    return (left ==  g_nullNode);
}

DynamicAabbTree::DynamicAabbTree(int nParticles):
    m_boxSize(0.0, 0.0)
{
    // Initialise the tree.
    m_root =  g_nullNode;
    m_nodeCount = 0;
    m_nodeCapacity = nParticles;
    m_nodes.resize(m_nodeCapacity);

    // Build a linked list for the list of free m_nodes.
    for (int i = 0; i < m_nodeCapacity - 1; i++)
    {
        m_nodes[i].next = i + 1;
        m_nodes[i].height = -1;
    }
    m_nodes[m_nodeCapacity - size_t(1)].next =  g_nullNode;
    m_nodes[m_nodeCapacity - size_t(1)].height = -1;

    // Assign the index of the first free node.
    m_freeList = 0;
}


int DynamicAabbTree::allocateNode()
{
    // Exand the node pool as needed.
    if (m_freeList ==  g_nullNode)
    {
        assert(m_nodeCount == m_nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        m_nodeCapacity *= 2;
        m_nodes.resize(m_nodeCapacity);

        // Build a linked list for the list of free m_nodes.
        for (int i = m_nodeCount; i < m_nodeCapacity - 1; i++)
        {
            m_nodes[i].next = i + 1;
            m_nodes[i].height = -1;
        }
        m_nodes[m_nodeCapacity - 1].next =  g_nullNode;
        m_nodes[m_nodeCapacity - 1].height = -1;

        // Assign the index of the first free node.
        m_freeList = m_nodeCount;
    }

    // Peel a node off the free list.
    int node = m_freeList;
    m_freeList = m_nodes[node].next;
    m_nodes[node].parent =  g_nullNode;
    m_nodes[node].left =  g_nullNode;
    m_nodes[node].right =  g_nullNode;
    m_nodes[node].height = 0;
    m_nodeCount++;

    return node;
}

void DynamicAabbTree::freeNode(int node)
{
    m_nodes[node].next = m_freeList;
    m_nodes[node].height = -1;
    m_freeList = node;
    m_nodeCount--;
}



bool DynamicAabbTree::insertParticle(int particle, const AABB& aabb)
{
    // Make sure the particle doesn't already exist.
    if (m_particleMap.count(particle) != 0)
    {
        ErrLog::log("Error: Particle already exists in DynamicAabbTree!");
        return false;
    }

    // Allocate a new node for the particle.
    int node = allocateNode();

    m_nodes[node].aabb = aabb;
    m_nodes[node].height = 0;     // Zero the height.

    insertLeaf(node);     // Insert a new leaf into the tree.

    // Add the new particle to the map.
    m_particleMap.insert(std::unordered_map<int, int>::value_type(particle, node));

    m_nodes[node].particle = particle;     // Store the particle index.

    return true;
}


bool DynamicAabbTree::removeParticle(int particle)
{
    auto it = m_particleMap.find(particle);
    if (it == m_particleMap.end())
    {
        ErrLog::log("ERROR: Cannot remove particle, invalid particle index!");
        return false;
    }

    int node = it->second;     // Extract the node index.
    m_particleMap.erase(it);     // Erase the particle from the map.

    removeLeaf(node);
    freeNode(node);

    return true;
}


bool DynamicAabbTree::updateParticle(int particle, const AABB& aabb)
{
    // Find the particle.
    auto it = m_particleMap.find(particle);

    // The particle doesn't exist.
    if (it == m_particleMap.end())
    {
        ErrLog::log("ERROR: DynamicAABBTree invalid particle index on update!" + std::to_string(particle));
        return false;
    }

    // Extract the node index.
    int node = it->second;

    // Remove the current leaf.
    removeLeaf(node);

    // Assign the new AABB.
    m_nodes[node].aabb = aabb;

    // Insert a new leaf node.
    insertLeaf(node);

    return true;
}


std::vector<int> DynamicAabbTree::query(int particle) const
{
    auto mapIt = m_particleMap.find(particle);
    if (mapIt == m_particleMap.end())
    {
        ErrLog::log("ERROR: DynamicAABBTree invalid particle index on querty! Index:" + std::to_string(particle));
        return std::vector<int>(); //return empty vector
    }

    return query(particle, m_nodes[mapIt->second].aabb);
}


std::vector<int> DynamicAabbTree::query(int particle, const AABB& aabb) const
{
    std::vector<int> stack;
    stack.reserve(256);
    stack.push_back(m_root);

    std::vector<int> particles;

    while (stack.size() > 0)
    {
        int node = stack.back();
        stack.pop_back();

        // Copy the AABB.
        AABB nodeAABB = m_nodes[node].aabb;

        if (node ==  g_nullNode) continue;

        // Test for overlap between the AABBs.
        if (aabb.overlaps(nodeAABB))
        {
            // Check that we're at a leaf node.
            if (m_nodes[node].isLeaf())
            {
                // Can't interact with itself.
                if (m_nodes[node].particle != particle)
                    particles.push_back(m_nodes[node].particle);
            }
            else
            {
                stack.push_back(m_nodes[node].left);
                stack.push_back(m_nodes[node].right);
            }
        }
    }

    return particles;
}


std::vector<int> DynamicAabbTree::query(const AABB& aabb) const
{
    // Make sure the tree isn't empty.
    if (m_particleMap.size() == 0)
    {
        return std::vector<int>();
    }

    // Test overlap of AABB against all particles.
    return query(std::numeric_limits<int>::max(), aabb);
}


bool DynamicAabbTree::getAABB(int particle, AABB* aabb) const
{
    auto mapIt = m_particleMap.find(particle);
    if (mapIt == m_particleMap.end())
    {
        ErrLog::log("ERROR: DynamicAABBTree invalid particle index on getAABB! Index:" + std::to_string(particle));
        return false; //return empty vector
    }

    *aabb = m_nodes[mapIt->second].aabb;
    return true;
}


void DynamicAabbTree::insertLeaf(int leaf)
{
    if (m_root ==  g_nullNode)
    {
        m_root = leaf;
        m_nodes[m_root].parent =  g_nullNode;
        return;
    }

    // Find the best sibling for the node.

    AABB leafAABB = m_nodes[leaf].aabb;
    int index = m_root;

    while (!m_nodes[index].isLeaf())
    {
        // Extract the children of the node.
        int left = m_nodes[index].left;
        int right = m_nodes[index].right;

        double surfaceArea = m_nodes[index].aabb.perimiter();

        AABB combinedAABB;
        combinedAABB.merge(m_nodes[index].aabb, leafAABB);
        double combinedSurfaceArea = combinedAABB.perimiter();

        // Cost of creating a new parent for this node and the new leaf.
        double cost = 2.0 * combinedSurfaceArea;

        // Minimum cost of pushing the leaf further down the tree.
        double inheritanceCost = 2.0 * (combinedSurfaceArea - surfaceArea);

        // Cost of descending to the left.
        double costLeft;
        if (m_nodes[left].isLeaf())
        {
            AABB aabb;
            aabb.merge(leafAABB, m_nodes[left].aabb);
            costLeft = aabb.perimiter() + inheritanceCost;
        }
        else
        {
            AABB aabb;
            aabb.merge(leafAABB, m_nodes[left].aabb);
            double oldArea = m_nodes[left].aabb.perimiter();
            double newArea = aabb.perimiter();
            costLeft = (newArea - oldArea) + inheritanceCost;
        }

        // Cost of descending to the right.
        double costRight;
        if (m_nodes[right].isLeaf())
        {
            AABB aabb;
            aabb.merge(leafAABB, m_nodes[right].aabb);
            costRight = aabb.perimiter() + inheritanceCost;
        }
        else
        {
            AABB aabb;
            aabb.merge(leafAABB, m_nodes[right].aabb);
            double oldArea = m_nodes[right].aabb.perimiter();
            double newArea = aabb.perimiter();
            costRight = (newArea - oldArea) + inheritanceCost;
        }

        // Descend according to the minimum cost.
        if ((cost < costLeft) && (cost < costRight)) break;

        // Descend.
        if (costLeft < costRight) index = left;
        else                      index = right;
    }

    int sibling = index;

    // Create a new parent.
    int oldParent = m_nodes[sibling].parent;
    int newParent = allocateNode();
    m_nodes[newParent].parent = oldParent;
    m_nodes[newParent].aabb.merge(leafAABB, m_nodes[sibling].aabb);
    m_nodes[newParent].height = m_nodes[sibling].height + 1;

    // The sibling was not the root.
    if (oldParent !=  g_nullNode)
    {
        if (m_nodes[oldParent].left == sibling) m_nodes[oldParent].left = newParent;
        else                                  m_nodes[oldParent].right = newParent;

        m_nodes[newParent].left = sibling;
        m_nodes[newParent].right = leaf;
        m_nodes[sibling].parent = newParent;
        m_nodes[leaf].parent = newParent;
    }
    // The sibling was the root.
    else
    {
        m_nodes[newParent].left = sibling;
        m_nodes[newParent].right = leaf;
        m_nodes[sibling].parent = newParent;
        m_nodes[leaf].parent = newParent;
        m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs.
    index = m_nodes[leaf].parent;
    while (index !=  g_nullNode)
    {
        index = balance(index);

        int left = m_nodes[index].left;
        int right = m_nodes[index].right;

        m_nodes[index].height = 1 + std::max(m_nodes[left].height, m_nodes[right].height);
        m_nodes[index].aabb.merge(m_nodes[left].aabb, m_nodes[right].aabb);

        index = m_nodes[index].parent;
    }
}


void DynamicAabbTree::removeLeaf(int leaf)
{
    if (leaf == m_root)
    {
        m_root =  g_nullNode;
        return;
    }

    int parent = m_nodes[leaf].parent;
    int grandParent = m_nodes[parent].parent;
    int sibling;

    if (m_nodes[parent].left == leaf) sibling = m_nodes[parent].right;
    else                            sibling = m_nodes[parent].left;

    // Destroy the parent and connect the sibling to the grandparent.
    if (grandParent !=  g_nullNode)
    {
        if (m_nodes[grandParent].left == parent) m_nodes[grandParent].left = sibling;
        else                                   m_nodes[grandParent].right = sibling;

        m_nodes[sibling].parent = grandParent;
        freeNode(parent);

        // Adjust ancestor bounds.
        int index = grandParent;
        while (index !=  g_nullNode)
        {
            index = balance(index);

            int left = m_nodes[index].left;
            int right = m_nodes[index].right;

            m_nodes[index].aabb.merge(m_nodes[left].aabb, m_nodes[right].aabb);
            m_nodes[index].height = 1 + std::max(m_nodes[left].height, m_nodes[right].height);

            index = m_nodes[index].parent;
        }
    }
    else
    {
        m_root = sibling;
        m_nodes[sibling].parent =  g_nullNode;
        freeNode(parent);
    }
}


int DynamicAabbTree::balance(int node)
{
    assert(node !=  g_nullNode);

    if (m_nodes[node].isLeaf() || (m_nodes[node].height < 2))         return node;

    int left = m_nodes[node].left;
    int right = m_nodes[node].right;

    int currentBalance = m_nodes[right].height - m_nodes[left].height;

    // Rotate right branch up.
    if (currentBalance > 1)
    {
        int rightLeft = m_nodes[right].left;
        int rightRight = m_nodes[right].right;

        // Swap node and its right-hand child.
        m_nodes[right].left = node;
        m_nodes[right].parent = m_nodes[node].parent;
        m_nodes[node].parent = right;

        // The node's old parent should now point to its right-hand child.
        if (m_nodes[right].parent !=  g_nullNode)
        {
            if (m_nodes[m_nodes[right].parent].left == node) m_nodes[m_nodes[right].parent].left = right;
            else
            {
                m_nodes[m_nodes[right].parent].right = right;
            }
        }
        else m_root = right;

        // Rotate.
        if (m_nodes[rightLeft].height > m_nodes[rightRight].height)
        {
            m_nodes[right].right = rightLeft;
            m_nodes[node].right = rightRight;
            m_nodes[rightRight].parent = node;
            m_nodes[node].aabb.merge(m_nodes[left].aabb, m_nodes[rightRight].aabb);
            m_nodes[right].aabb.merge(m_nodes[node].aabb, m_nodes[rightLeft].aabb);

            m_nodes[node].height = 1 + std::max(m_nodes[left].height, m_nodes[rightRight].height);
            m_nodes[right].height = 1 + std::max(m_nodes[node].height, m_nodes[rightLeft].height);
        }
        else
        {
            m_nodes[right].right = rightRight;
            m_nodes[node].right = rightLeft;
            m_nodes[rightLeft].parent = node;
            m_nodes[node].aabb.merge(m_nodes[left].aabb, m_nodes[rightLeft].aabb);
            m_nodes[right].aabb.merge(m_nodes[node].aabb, m_nodes[rightRight].aabb);

            m_nodes[node].height = 1 + std::max(m_nodes[left].height, m_nodes[rightLeft].height);
            m_nodes[right].height = 1 + std::max(m_nodes[node].height, m_nodes[rightRight].height);
        }

        return right;
    }

    // Rotate left branch up.
    if (currentBalance < -1)
    {
        int leftLeft = m_nodes[left].left;
        int leftRight = m_nodes[left].right;

        // Swap node and its left-hand child.
        m_nodes[left].left = node;
        m_nodes[left].parent = m_nodes[node].parent;
        m_nodes[node].parent = left;

        // The node's old parent should now point to its left-hand child.
        if (m_nodes[left].parent !=  g_nullNode)
        {
            if (m_nodes[m_nodes[left].parent].left == node) m_nodes[m_nodes[left].parent].left = left;
            else
            {
                m_nodes[m_nodes[left].parent].right = left;
            }
        }
        else m_root = left;

        // Rotate.
        if (m_nodes[leftLeft].height > m_nodes[leftRight].height)
        {
            m_nodes[left].right = leftLeft;
            m_nodes[node].left = leftRight;
            m_nodes[leftRight].parent = node;
            m_nodes[node].aabb.merge(m_nodes[right].aabb, m_nodes[leftRight].aabb);
            m_nodes[left].aabb.merge(m_nodes[node].aabb, m_nodes[leftLeft].aabb);

            m_nodes[node].height = 1 + std::max(m_nodes[right].height, m_nodes[leftRight].height);
            m_nodes[left].height = 1 + std::max(m_nodes[node].height, m_nodes[leftLeft].height);
        }
        else
        {
            m_nodes[left].right = leftRight;
            m_nodes[node].left = leftLeft;
            m_nodes[leftLeft].parent = node;
            m_nodes[node].aabb.merge(m_nodes[right].aabb, m_nodes[leftLeft].aabb);
            m_nodes[left].aabb.merge(m_nodes[node].aabb, m_nodes[leftRight].aabb);

            m_nodes[node].height = 1 + std::max(m_nodes[right].height, m_nodes[leftLeft].height);
            m_nodes[left].height = 1 + std::max(m_nodes[node].height, m_nodes[leftRight].height);
        }

        return left;
    }

    return node;
}


int DynamicAabbTree::getHeight() const
{
    if (m_root ==  g_nullNode) return 0;
    return m_nodes[m_root].height;
}