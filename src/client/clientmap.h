/*
Minetest
Copyright (C) 2010-2013 celeron55, Perttu Ahola <celeron55@gmail.com>

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
#include "map.h"
#include "camera.h"
#include <map>
#include "profiler.h"
#include "mapsector.h"
#include "mapblock.h"
#include "mapblock_mesh.h"
#include <functional>
#include <atomic>
#include "threading/thread.h"
#include <chrono>
#include "porting.h"
#include "log.h"
#include "IVertexBuffer.h"

class MeshStorage;
class Client;
class ITextureSource;

struct MapDrawControl
{
	// Wanted drawing range
	f32 wanted_range = 0.0f;
	// Overrides limits by drawing everything
	bool range_all = false;
	// Allow rendering out of bounds
	bool allow_noclip = false;
	// show a wire frame for debugging
	bool show_wireframe = false;
};

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

class UpdateClientMapThread;

/*
	ClientMap

	This is the only map class that is able to render itself on screen.
*/

class ClientMap : public Map, public scene::ISceneNode
{
public:
	ClientMap(
			Client *client,
			RenderingEngine *rendering_engine,
			MapDrawControl &control,
			s32 id
	);

	bool maySaveBlocks() override
	{
		return false;
	}

	void updateCamera(v3f pos, v3f dir, f32 fov, v3s16 offset, video::SColor light_color);

	/*
		Forcefully get a sector from somewhere
	*/
	MapSector * emergeSector(v2s16 p) override;

	MapBlock * emergeBlock(v3s16 p, bool create_blank=true) override;
	/*
		ISceneNode methods
	*/

	virtual void OnRegisterSceneNode() override;

	virtual void render() override
	{
		video::IVideoDriver* driver = SceneManager->getVideoDriver();
		rebuildVBOs(driver);

		driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);
		renderMap(driver, SceneManager->getSceneNodeRenderPass());
	}

	virtual const aabb3f &getBoundingBox() const override
	{
		return m_box;
	}

	void rebuildOctree();
	//void getBlocksInViewRange(v3s16 cam_pos_nodes,
	//	v3s16 *p_blocks_min, v3s16 *p_blocks_max, float range=-1.0f);
	void frustumCull();
	// @brief Calculate statistics about the map and keep the blocks alive
	void updateDrawBuffers();
    void updateLighting();
	void touchMapBlocks();
	void rebuildVBOs(video::IVideoDriver* driver);
	void updateDrawListShadow(v3f shadow_light_pos, v3f shadow_light_dir, float radius, float length);
	// Returns true if draw list needs updating before drawing the next frame.
	//bool needsUpdateDrawList() { return m_needs_update_drawlist; }
	void renderMap(video::IVideoDriver* driver, s32 pass);

	void renderMapShadows(video::IVideoDriver *driver,
			const video::SMaterial &material, s32 pass, int frame, int total_frames);

	int getBackgroundBrightness(float max_d, u32 daylight_factor,
			int oldvalue, bool *sunlight_seen_result);

	void renderPostFx(CameraMode cam_mode);

	// For debug printing
	void PrintInfo(std::ostream &out) override;

	const MapDrawControl & getControl() const { return m_control; }
	f32 getWantedRange() const { return m_control.wanted_range; }
	f32 getCameraFov() const { return m_camera_fov; }

    MeshStorage &getMeshStorage() { return m_mesh_storage; }

protected:
	// use drop() instead
	virtual ~ClientMap();

	void reportMetrics(u64 save_time_us, u32 saved_blocks, u32 all_blocks) override;
private:
	//bool isMeshOccluded(MapBlock *mesh_block, u16 mesh_size, v3s16 cam_pos_nodes);

	// update the vertex order in transparent mesh buffers
	//void updateTransparentMeshBuffers();


	// Orders blocks by distance to the camera
	class MapBlockComparer
	{
	public:
		MapBlockComparer(const v3s16 &camera_bpos) : m_camera_bpos(camera_bpos) {}

		bool operator() (const v3s16 &left, const v3s16 &right) const
		{
			auto distance_left = left.getDistanceFromSQ(m_camera_bpos);
			auto distance_right = right.getDistanceFromSQ(m_camera_bpos);
			return distance_left > distance_right || (distance_left == distance_right && left > right);
		}

	private:
		v3s16 m_camera_bpos;
	};

	// reference to a mesh buffer used when rendering the map.
	/*struct DrawBuffer {
		v3s16 m_pos;
		union {
			scene::IMeshBuffer *m_buffer;
			const PartialMeshBuffer *m_partial_buffer;
		};
		bool m_reuse_material:1;
		bool m_use_partial_buffer:1;

		DrawBuffer(v3s16 pos, scene::IMeshBuffer *buffer, bool reuse_material) :
			m_pos(pos), m_buffer(buffer), m_reuse_material(reuse_material), m_use_partial_buffer(false)
		{}

		DrawBuffer(v3s16 pos, const PartialMeshBuffer *buffer) :
			m_pos(pos), m_partial_buffer(buffer), m_reuse_material(false), m_use_partial_buffer(true)
		{}

		scene::IMeshBuffer* getBuffer();
		void draw(video::IVideoDriver* driver);
	};*/

	Client *m_client;
	RenderingEngine *m_rendering_engine;

	aabb3f m_box = aabb3f(-BS * 1000000, -BS * 1000000, -BS * 1000000,
		BS * 1000000, BS * 1000000, BS * 1000000);

	MapDrawControl &m_control;

	v3f m_camera_position = v3f(0,0,0);
	v3f m_camera_direction = v3f(0,0,1);
	f32 m_camera_fov = M_PI;
	v3f m_camera_direction_change = v3f(0,0,0);
	v3s16 m_camera_offset;
	video::SColor m_camera_light_color = video::SColor(0xFFFFFFFF);

	f32 m_last_wanted_range = 0.0f;
	bool m_was_range_all = false;

	//std::map<v3s16, MapBlock*> m_drawlist_shadow;

	std::atomic<bool> m_needs_rebuild_octree;
	std::atomic<bool> m_needs_frustum_cull_blocks;
	std::atomic<bool> m_needs_update_transparent_meshes;

	UpdateClientMapThread *m_clientmap_thread = nullptr;

	f32 m_last_daynight_ratio;

	Octree m_octree;
	std::map<v3s16, MapBlock *, MapBlockComparer> m_sorted_mapblocks;

	MeshStorage m_mesh_storage;

	std::vector<std::pair<video::SMaterial, std::vector<scene::IVertexBuffer *>>> m_solid_vbos;
	std::vector<std::pair<video::SMaterial, scene::IVertexBuffer *>> m_transparent_vbos;


	//std::unordered_map<u32, std::vector<u32>> m_visible_buffers;

	bool m_enable_shaders;
	//u16 m_cache_transparency_sorting_distance;

	//bool m_loops_occlusion_culler;
	//bool m_enable_raytraced_culling;
};

// ClientMap Thread executing in each 50 ms:
  /* rebuilding of the octree
   * frustum culling
   * regenerating VBOs
   * updating vertex lighting and mapblocks timers
   */
class UpdateClientMapThread final: public Thread
{
public:
    UpdateClientMapThread(ClientMap *map)
        : Thread("Update Clientmap"), m_map(map) {}

	void *run()
	{
		BEGIN_DEBUG_EXCEPTION_HANDLER

		while (!stopRequested()) {
			//infostream << "rebuildOctree()" << std::endl;
			m_map->rebuildOctree();
			//infostream << "frustumCull()" << std::endl;
			m_map->frustumCull();
			//infostream << "updateLightAndTimers()" << std::endl;
            m_map->updateLighting();
			//infostream << "updateDrawBuffers()" << std::endl;
			m_map->updateDrawBuffers();
			//infostream << "touchMapBlocks()" << std::endl;
			m_map->touchMapBlocks();
			//infostream << "UpdateClientMapThread sleep 50 ms..." << std::endl;

			sleep_ms(50);
			//infostream << "UpdateClientMapThread continue" << std::endl;
		}

		END_DEBUG_EXCEPTION_HANDLER

		return nullptr;
	}

private:
	ClientMap *m_map;
};
