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
#include <vector>
#include <unordered_map>
#include <map>
#include <list>
#include <mutex>
#include "client/mesh_collector.h"
#include "util/basic_macros.h"
#include <IVertexBuffer.h>
#include <atomic>

class Client;
struct TileSpec;
struct MeshTriangle;

using StorageBuffers = std::list<std::pair<video::SMaterial, std::list<scene::IVertexBuffer *>>>;

// Abstraction containing mesh sets of the same material
struct MeshLayer
{
	// Material of the layer
	video::SMaterial material;
	// Baked solid mesh sets for rebuilding vbos
	std::list<MeshPart> merged_mesh;

	MeshLayer() = default;

	explicit MeshLayer(const video::SMaterial &mat) : material(mat)
	{
		merged_mesh.push_back(MeshPart());
	}
};

// Cubic level consisting of other cubic sublevels
// The upper level contains 8x8x8 mapblocks, the lower one is 4x4x4, then 2x2x2 and the mapblock itself
// Each level has an own merged meshes scattered through layers
// MeshLevel is also an octree node containing other MeshLevels as octants
class MeshStorage
{
public:
	std::atomic<bool> storage_updated = false;

    MeshStorage(Client *client)
		: m_client(client) {}

    ~MeshStorage();

    DISABLE_CLASS_COPY(MeshStorage) // copying of the layers is prohibited

	void mergeNewLayers(const std::list<MeshLayer *> &new_layers);

    void rebuildSolidBuffers(video::IVideoDriver *driver);

	void renderBuffers(video::IVideoDriver *driver, bool wireframe, u32 &drawcall_count);
   // void rebuildTransparentVBOs(video::IVideoDriver *driver,
	//	std::vector<std::pair<video::SMaterial, scene::IVertexBuffer *>> &vbos);

	void updateLighting(video::SColorf &day_color);

private:
	Client *m_client;

	std::atomic<bool> needs_rebuild_tvbos = false;

	std::list<MeshLayer *> m_layers;
	StorageBuffers m_buffers;

	// Baked transparent mesh sets for rebuilding vbos
	//std::list<std::pair<video::SMaterial, MeshPart>> m_transparent_mesh;

	// Many mesh generation threads, update clientmap and also main one may access to the layers simultaneously
    //std::mutex m_layers_mutex;
	//std::mutex m_tmesh_mutex;
};
