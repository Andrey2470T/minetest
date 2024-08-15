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
#include <vector>
#include <unordered_map>
#include <functional>
#include "mapblock.h"
#include "mapsector.h"

class MapDrawControl;

// Octree for accelerated frustum culling
class OctreeNode
{
public:
	OctreeNode(std::vector<MapBlock *> mapblocks, core::aabbox3d<s16> box)
		: m_mapblocks(mapblocks), m_bounding_box(box)
	{
		for (u32 i = 0; i < 8; i++)
			m_child_nodes[i] = nullptr;
	}

	~OctreeNode()
	{
		// If this is not a leaf node, recursively delete its childs
		for (auto node : m_child_nodes)
			if (node)
				delete node;
	}

	void splitNode();

	// Calls some callback for this node and in case true continue traversing through its children
	void traverseNode(std::function<bool(const OctreeNode *)> func)
	{
		bool result = func(this);

		if (result)
			for (auto &next_node : m_child_nodes)
				if (next_node)
					next_node->traverseNode(func);
	}

	const std::vector<MapBlock *> &getMapBlocks() const
	{
		return m_mapblocks;
	}

	core::aabbox3d<s16> getBox() const
	{
		return m_bounding_box;
	}

	u32 getFullCountOfMapBlocks() const
	{
		u32 mapblocks_count = 0;

		mapblocks_count += m_mapblocks.size();

		for (auto &node : m_child_nodes)
			if (node)
				mapblocks_count += node->getFullCountOfMapBlocks();

		return mapblocks_count;
	}
private:
	std::array<OctreeNode *, 8> m_child_nodes;

	std::vector<MapBlock *> m_mapblocks;

	core::aabbox3d<s16> m_bounding_box; // in nodes
};

class Octree
{
public:
	Octree() = default;

	void buildTree(std::unordered_map<v2s16, MapSector *> &sectors, v3s16 cam_pos_nodes, MapDrawControl &control);

	void traverseTree(std::function<bool(const OctreeNode *)> func)
	{
		if (!m_root_node)
			return;

		m_root_node->traverseNode(func);
	}

	bool isBuilt() { return m_root_node != nullptr; }

	void clearTree()
	{
		if (m_root_node)
			delete m_root_node;
	}

private:
	OctreeNode *m_root_node = nullptr;
};
