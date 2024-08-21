/*
Minetest
Copyright (C) 2024 Andrey, Andrey2470T <andreyt2203@gmail.com>
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#pragma once

#include "irrlichttypes_extrabloated.h"
#include <array>
#include <functional>
#include "mapblock.h"
#include "mapblock_mesh.h"
#include "mesh_storage.h"

// The grid-aligned octree size is 8x8x8 mapblocks
#define OCTREE_SIZE 8

struct MapDrawControl;
class Client;

// Octree for accelerating the frustum culling
class OctreeNode
{
public:
    OctreeNode(core::aabbox3d<s16> bounding_box, s16 size)
		: m_bounding_box(bounding_box), m_size(size)
	{
		for (u16 i = 0; i < 8; i++)
			m_child_nodes[i] = nullptr;
	}

    virtual ~OctreeNode()
	{
		for (auto node : m_child_nodes)
			if (node)
				delete node;

        if (storage)
            delete storage;
	}

    DISABLE_CLASS_COPY(OctreeNode)

    void splitNode();

	// Calls some callback for this node and in case true continue traversing through its children
    void traverseNode(std::function<bool(const OctreeNode *)> func) const
	{
		bool result = func(this);

		if (result)
			for (auto next_node : m_child_nodes)
                if (next_node)
					next_node->traverseNode(func);
	}

    bool traverseNodeForAdd(MapBlock *mp);

    bool traverseNodeForDelete(v3s16 pos);

    s16 getSize() const { return m_size; }
    s16 getVolume() const { return m_size * m_size * m_size; }

    v3f getCenter() const // in nodes
    {
        v3f min_pos_f = intToFloat(m_bounding_box.MinEdge * MAP_BLOCKSIZE, BS);
        v3f max_pos_f = intToFloat(m_bounding_box.MaxEdge * MAP_BLOCKSIZE + (MAP_BLOCKSIZE - 1), BS);

        return aabb3f(min_pos_f, max_pos_f).getCenter();
    }

    f32 getRadius() const // in nodes
    {
        v3f min_pos_f = intToFloat(m_bounding_box.MinEdge * MAP_BLOCKSIZE, BS);
        v3f max_pos_f = intToFloat(m_bounding_box.MaxEdge * MAP_BLOCKSIZE + (MAP_BLOCKSIZE - 1), BS);

        return ((max_pos_f - min_pos_f) / 2.0f).getLength();
    }

    MapblockMeshCollector *getMapblockMesh() const {
        if (!m_mapblock.second || !m_mapblock.second->mesh)
            return nullptr;

        return m_mapblock.second->mesh->getMesh();
    }

    bool resetMapblockTimer() const
    {
		if (!m_mapblock.second)
			return false;

		m_mapblock.second->resetUsageTimer();

		return true;
	}

    bool need_rebuild = false;

    MeshStorage *storage = nullptr;
private:
    std::array<OctreeNode *, 8> m_child_nodes;

    std::pair<v3s16, MapBlock *> m_mapblock;

	core::aabbox3d<s16> m_bounding_box; // in blocks
    s16 m_size;                         // in blocks
};

class Octree
{
public:
    Octree() = default;

    ~Octree()
	{
		clearTree();
	}

    DISABLE_CLASS_COPY(Octree)

	// Creates the root node and splits it recursively into 8 octants
	// until the box size of the children is not equal to the mapblock one
    void buildTree(v3s16 corner_pos_blocks, s16 size, MapDrawControl &control);

	// Checks if the octree was built
	bool isBuilt() const { return m_root_node != nullptr; }

	// Traverses through the tree recursively with some callback
    void traverseTree(std::function<bool(const OctreeNode *)> func) const
	{
		if (!isBuilt())
			return;

		m_root_node->traverseNode(func);
	}

    void traverseTreeForAdd(MapBlock *mp)
    {
        if (!isBuilt())
            return;

        m_root_node->traverseNodeForAdd(mp);
    }

    void traverseTreeForDelete(v3s16 pos)
    {
        if (!isBuilt())
            return;

        m_root_node->traverseNodeForDelete(pos);
    }

	// Clears all children recursively
	void clearTree()
	{
		if (isBuilt())
			delete m_root_node;
	}

    void markNeedsRebuild()
    {
        if (isBuilt())
            m_root_node->need_rebuild = true;
    }

	std::map<v3s16, MapBlock *> mapblocks;

private:
    OctreeNode *m_root_node;
};
