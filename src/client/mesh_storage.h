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
#include <shared_mutex>
#include "client/mesh_collector.h"
#include "util/basic_macros.h"
#include "IVertexBuffer.h"
#include <atomic>

class Client;
struct TileSpec;
struct MeshTriangle;

// Abstraction containing buffers of the same material
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

	// Add vertex and index arrays from some mesh part in this layer
    //void mergeArrays(u32 layer_index, const MeshPart &mesh, MeshInfo &mesh_info);
   // size_t removeArrays(u32 arrays_index);

	//bool isEmpty() { return m_arrays.empty(); }

	//void prepareSolidMesh(const std::vector<u32> &arrays);

    //void rebuildSolidVBOs(video::IVideoDriver *driver, std::vector<scene::IVertexBuffer *> &output);

	//void fetchVertexTriple(u32 arrays_index, const std::array<u32, 3> &indices,
     //   std::array<video::S3DVertex, 3> &vertices);

	//void updateLighting(video::SColorf &day_color);

    //void render(video::IVideoDriver *driver, bool wireframe, u32 &drawcall_count);
};

// Abstraction saving all layers of the scene (mapblocks or wieldmesh)
class MeshStorage
{
	Client *m_client;

	std::atomic<bool> needs_rebuild_tvbos = false;

	std::list<MeshLayer *> m_layers;

	// Baked transparent mesh sets for rebuilding vbos
	std::list<std::pair<video::SMaterial, MeshPart>> m_transparent_mesh;

	// Many mesh generation threads, update clientmap and also main one may access to the layers simultaneously
	std::mutex m_layers_mutex;
	std::mutex m_tmesh_mutex;

public:
	std::atomic<bool> layers_updated = false;

	MeshStorage(Client *client);

	DISABLE_CLASS_COPY(MeshStorage) // copying of the layers is prohibited

	// Add a bunch of new arrays from the collector
    //void addArrays(const MapblockMeshCollector &collector);

	// Delete all buffers and if necessary materials which the mesh part owns from the storage
	//size_t deleteArrays(const MeshRef &mesh);

	//void prepareSolidMeshes(const std::vector<MeshRef> &mesh_parts);
	//void prepareTransparentMeshes(const std::map<v3f, MeshTriangle, TriangleComparer> &triangles);

	void mergeNewLayers(const std::list<MeshLayer *> &new_layers);

    void rebuildSolidVBOs(video::IVideoDriver *driver,
        std::list<std::pair<video::SMaterial, std::list<scene::IVertexBuffer *>>> &vbos);
   // void rebuildTransparentVBOs(video::IVideoDriver *driver,
	//	std::vector<std::pair<video::SMaterial, scene::IVertexBuffer *>> &vbos);

	void updateLighting(video::SColorf &day_color);

   // void renderSolidVBOs(video::IVideoDriver *driver,
	//	bool wireframe, u32 &drawcall_count);
	//void renderTransparentVBOs(video::IVideoDriver *driver,
	//	bool wireframe, u32 &drawcall_count);
};
