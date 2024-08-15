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
#include "clientmap.h"

void OctreeNode::splitNode()
{
	//infostream << "splitNode() mapblocks count: " << m_mapblocks.size() << std::endl;
	if (m_mapblocks.size() <= 1)
		return;

	v3s16 dims = m_bounding_box.MaxEdge - m_bounding_box.MinEdge + v3s16(1, 1, 1);

	if (dims.X <= MAP_BLOCKSIZE || dims.Y <= MAP_BLOCKSIZE || dims.Z <= MAP_BLOCKSIZE)
		return;

	v3s16 centre = m_bounding_box.MinEdge + dims / 2;

	std::array<core::aabbox3d<s16>, 8> oct_boxes;

	oct_boxes[0] = core::aabbox3d<s16>(m_bounding_box.MinEdge, centre - v3s16(1, 1, 1));
	oct_boxes[1] = core::aabbox3d<s16>(
		v3s16(oct_boxes[0].MinEdge.X, oct_boxes[0].MinEdge.Y, centre.Z),
		v3s16(oct_boxes[0].MaxEdge.X, oct_boxes[0].MaxEdge.Y, m_bounding_box.MaxEdge.Z)
	);
	oct_boxes[2] = core::aabbox3d<s16>(
		v3s16(centre.X, oct_boxes[1].MinEdge.Y, oct_boxes[1].MinEdge.Z),
		v3s16(m_bounding_box.MaxEdge.X, oct_boxes[1].MaxEdge.Y, oct_boxes[1].MaxEdge.Z)
	);
	oct_boxes[3] = core::aabbox3d<s16>(
		v3s16(centre.X, oct_boxes[0].MinEdge.Y, oct_boxes[0].MinEdge.Z),
		v3s16(m_bounding_box.MaxEdge.X, oct_boxes[0].MaxEdge.Y, oct_boxes[0].MaxEdge.Z)
	);
	oct_boxes[4] = core::aabbox3d<s16>(
		v3s16(oct_boxes[0].MinEdge.X, centre.Y, oct_boxes[0].MinEdge.Z),
		v3s16(oct_boxes[0].MaxEdge.X, m_bounding_box.MaxEdge.Y, oct_boxes[0].MaxEdge.Z)
	);
	oct_boxes[5] = core::aabbox3d<s16>(
		v3s16(oct_boxes[4].MinEdge.X, oct_boxes[4].MinEdge.Y, centre.Z),
		v3s16(oct_boxes[4].MaxEdge.X, oct_boxes[4].MaxEdge.Y, m_bounding_box.MaxEdge.Z)
	);
	oct_boxes[6] = core::aabbox3d<s16>(
		v3s16(centre.X, oct_boxes[5].MinEdge.Y, oct_boxes[5].MinEdge.Z),
		v3s16(m_bounding_box.MaxEdge.X, oct_boxes[5].MaxEdge.Y, oct_boxes[5].MaxEdge.Z)
	);
	oct_boxes[7] = core::aabbox3d<s16>(
		v3s16(centre.X, oct_boxes[4].MinEdge.Y, oct_boxes[4].MinEdge.Z),
		v3s16(m_bounding_box.MaxEdge.X, oct_boxes[4].MaxEdge.Y, oct_boxes[4].MaxEdge.Z)
	);

	std::array<std::vector<MapBlock *>, 8> oct_lists;
	std::vector<u32> del_mapblocks_indices;

	for (s32 k = m_mapblocks.size()-1; k >= 0; k--) {
		core::aabbox3d<s16> box = m_mapblocks[k]->getBox();
		for (u16 i = 0; i < 8; i++)
			if (box.isFullInside(oct_boxes[i])) {
				oct_lists[i].push_back(m_mapblocks[k]);
				m_mapblocks.erase(m_mapblocks.begin() + k);
			}
	}

	for (u16 i = 0; i < 8; i++) {
		//infostream << "splitNode() new octree node i = " << i << std::endl;
		m_child_nodes[i] = new OctreeNode(oct_lists[i], oct_boxes[i]);
		m_child_nodes[i]->splitNode();
	}
}


void Octree::buildTree(std::unordered_map<v2s16, MapSector *> &sectors, v3s16 cam_pos_nodes, MapDrawControl &control)
{
	// Clears all childs formed in the previous tree building
	clearTree();

	v3s16 blocks_min{0, 0, 0};
	v3s16 blocks_max{0, 0, 0};

	// Number of blocks currently loaded by the client
	u32 blocks_loaded = 0;
	// Number of blocks with mesh caught by the tree
	u32 blocks_in_octree = 0;

	/*
	 * Calculates the min and max edges of the bounding box in nodes
	 */

	// If the limited viewing range is enabled, just gets two edge positions relatively to the camera one
	if (!control.range_all) {
		v3s16 box_nodes_d = v3s16(1, 1, 1) * control.wanted_range;

		blocks_min = getContainerPos(cam_pos_nodes - box_nodes_d, MAP_BLOCKSIZE);
		blocks_max = getContainerPos(cam_pos_nodes + box_nodes_d, MAP_BLOCKSIZE);
	}

	/*
	 * Collects all mapblocks within the calculated area
	 */
	std::vector<MapBlock *> mapblocks;

	for (auto &sector : sectors) {
		blocks_loaded += sector.second->size();

		MapBlockVect blocks_vect;
		sector.second->getBlocks(blocks_vect);

		for (auto &block : blocks_vect) {
			if (!block)
				continue;

			v3s16 block_pos = block->getPos();

			if (control.range_all) {
				blocks_min.X = std::min(block_pos.X, blocks_min.X);
				blocks_min.Y = std::min(block_pos.Y, blocks_min.Y);
				blocks_min.Z = std::min(block_pos.Z, blocks_min.Z);

				blocks_max.X = std::max(block_pos.X, blocks_max.X);
				blocks_max.Y = std::max(block_pos.Y, blocks_max.Y);
				blocks_max.Z = std::max(block_pos.Z, blocks_max.Z);
			}

			if (block->mesh) {
				if (!control.range_all) {
					if (blocks_min <= block_pos && block_pos <= blocks_max) {
						mapblocks.push_back(block);
						blocks_in_octree++;
					}
				}
				else {
					mapblocks.push_back(block);
					blocks_in_octree++;
				}

			}
		}
	}

	//infostream << "buildTree() blocks_loaded = " << blocks_loaded << std::endl;
	//infostream << "buildTree() blocks_in_octree = " << blocks_in_octree << std::endl;
	g_profiler->avg("MapBlock meshes in octree [#]", blocks_in_octree);
	g_profiler->avg("MapBlocks loaded [#]", blocks_loaded);

	core::aabbox3d<s16> box(
		blocks_min * MAP_BLOCKSIZE, blocks_max * MAP_BLOCKSIZE + v3s16(1, 1, 1) * (MAP_BLOCKSIZE-1));

	m_root_node = new OctreeNode(mapblocks, box);

	// Starts recursively splitting the nodes until leafs don`t have one mapblock
	m_root_node->splitNode();
}
