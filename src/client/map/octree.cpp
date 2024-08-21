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

#include "octree.h"
#include <cassert>
#include <cmath>

void OctreeNode::splitNode()
{
	//infostream << "splitNode() mapblocks count: " << m_mapblocks.size() << std::endl;

	if (m_size <= 1)
		return;

	s16 cur_size = m_size >> 1;

	std::array<core::aabbox3d<s16>, 8> oct_boxes;

	oct_boxes[0] = core::aabbox3d<s16>(
		m_bounding_box.MinEdge, m_bounding_box.MinEdge + (cur_size - 1));
	oct_boxes[1] = core::aabbox3d<s16>(
		oct_boxes[0].MinEdge + v3s16(0, 0, cur_size), oct_boxes[0].MaxEdge + v3s16(0, 0, cur_size));
	oct_boxes[2] = core::aabbox3d<s16>(
		oct_boxes[1].MinEdge + v3s16(cur_size, 0, 0), oct_boxes[1].MaxEdge + v3s16(cur_size, 0, 0));
	oct_boxes[3] = core::aabbox3d<s16>(
		oct_boxes[0].MinEdge + v3s16(cur_size, 0, 0), oct_boxes[0].MaxEdge + v3s16(cur_size, 0, 0));
	oct_boxes[4] = core::aabbox3d<s16>(
		oct_boxes[0].MinEdge + v3s16(0, cur_size, 0), oct_boxes[0].MaxEdge + v3s16(0, cur_size, 0));
	oct_boxes[5] = core::aabbox3d<s16>(
		oct_boxes[1].MinEdge + v3s16(0, cur_size, 0), oct_boxes[1].MaxEdge + v3s16(0, cur_size, 0));
	oct_boxes[6] = core::aabbox3d<s16>(
		oct_boxes[2].MinEdge + v3s16(0, cur_size, 0), oct_boxes[2].MaxEdge + v3s16(0, cur_size, 0));
	oct_boxes[7] = core::aabbox3d<s16>(
		oct_boxes[3].MinEdge + v3s16(0, cur_size, 0), oct_boxes[3].MaxEdge + v3s16(0, cur_size, 0));

	for (u16 i = 0; i < 8; i++) {
		//infostream << "splitNode() new octree node i = " << i << std::endl;
        m_child_nodes[i] = new OctreeNode(oct_boxes[i], cur_size);
        m_child_nodes[i]->splitNode();
	}
}

bool OctreeNode::traverseNodeForAdd(MapBlock *mp)
{
    core::aabbox3d<s16> box_nodes(
        m_bounding_box.MinEdge * MAP_BLOCKSIZE,
        m_bounding_box.MaxEdge * MAP_BLOCKSIZE + (MAP_BLOCKSIZE - 1)
    );
    auto box = mp->getBox();

    if (!box.isFullInside(box_nodes))
        return false;

    need_rebuild = true;

    for (auto next_node : m_child_nodes)
        if (next_node)
            if (next_node->traverseNodeForAdd(mp))
                return false;

    m_mapblock = std::make_pair(mp->getPos(), mp);

    return true;
}

bool OctreeNode::traverseNodeForDelete(v3s16 pos)
{
    if (m_mapblock.first == pos) {
        m_mapblock = std::make_pair(v3s16(0, 0, 0), nullptr);
        return true;
    }

    need_rebuild = true;

    for (auto next_node : m_child_nodes)
        if (next_node)
            if (next_node->traverseNodeForDelete(pos))
                break;

    return false;
}


void Octree::buildTree(v3s16 corner_pos_blocks, s16 size, MapDrawControl &control)
{
	assert(std::pow(2, std::log2(size)) == size); // the size should be the power of two

	// Clears all childs formed in the previous tree building
	clearTree();

	//g_profiler->avg("MapBlock meshes in octree [#]", blocks_in_octree);
	//g_profiler->avg("MapBlocks loaded [#]", blocks_loaded);

    m_root_node = new OctreeNode(
		core::aabbox3d<s16>(corner_pos_blocks, corner_pos_blocks + (size - 1)), size);

	// Starts recursively splitting the nodes until leafs don`t have one mapblock
    m_root_node->splitNode();
}
