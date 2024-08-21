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

#include "clientmap.h"
#include "client/client.h"
#include "client/mesh.h"
#include <IMaterialRenderer.h>
#include <matrix4.h>
#include "nodedef.h"
#include "settings.h"
#include "util/basic_macros.h"
#include "client/renderingengine.h"
#include "client/light_colors.h"
#include "threading/mutex_auto_lock.h"
#include "mesh_storage.h"
#include "log.h"

#include <queue>

// ClientMap

ClientMap::ClientMap(
		Client *client,
		RenderingEngine *rendering_engine,
		MapDrawControl &control,
		s32 id
):
	Map(client),
	scene::ISceneNode(rendering_engine->get_scene_manager()->getRootSceneNode(),
		rendering_engine->get_scene_manager(), id),
	m_client(client),
	m_rendering_engine(rendering_engine),
	m_control(control),
    m_levels(PositionComparer(v3f()))//,
	//m_sorted_mapblocks(MapBlockComparer(v3s16(0,0,0))),
    //m_mesh_storage(client)
{
	/*
	 * @Liso: Sadly C++ doesn't have introspection, so the only way we have to know
	 * the class is whith a name ;) Name property cames from ISceneNode base class.
	 */
	Name = "ClientMap";
	m_box = aabb3f(-BS*1000000,-BS*1000000,-BS*1000000,
			BS*1000000,BS*1000000,BS*1000000);

	/* TODO: Add a callback function so these can be updated when a setting
	 *       changes.  At this point in time it doesn't matter (e.g. /set
	 *       is documented to change server settings only)
	 *
	 * TODO: Local caching of settings is not optimal and should at some stage
	 *       be updated to use a global settings object for getting thse values
	 *       (as opposed to the this local caching). This can be addressed in
	 *       a later release.
	 */
	m_enable_shaders = g_settings->getBool("enable_shaders");

    m_clientmap_thread = new UpdateClientMapThread(this);
	m_clientmap_thread->start();
	//m_cache_transparency_sorting_distance = g_settings->getU16("transparency_sorting_distance");
	//m_loops_occlusion_culler = g_settings->get("occlusion_culler") == "loops";
	//g_settings->registerChangedCallback("occlusion_culler", on_settings_changed, this);
	//m_enable_raytraced_culling = g_settings->getBool("enable_raytraced_culling");
	//g_settings->registerChangedCallback("enable_raytraced_culling", on_settings_changed, this);
}

/*void ClientMap::onSettingChanged(const std::string &name)
{
	if (name == "occlusion_culler")
		m_loops_occlusion_culler = g_settings->get("occlusion_culler") == "loops";
	if (name == "enable_raytraced_culling")
		m_enable_raytraced_culling = g_settings->getBool("enable_raytraced_culling");
}*/

ClientMap::~ClientMap()
{
	m_clientmap_thread->stop();
	m_clientmap_thread->wait();

	delete m_clientmap_thread;

    /*for (auto &vbos_layer : m_solid_vbos)
        for (auto buffer : vbos_layer.second)
            if (buffer)
                delete buffer;

    for (auto &vbo_layer : m_transparent_vbos)
        if (vbo_layer.second)
            delete vbo_layer.second;*/
	//g_settings->deregisterChangedCallback("occlusion_culler", on_settings_changed, this);
	//g_settings->deregisterChangedCallback("enable_raytraced_culling", on_settings_changed, this);
}

void ClientMap::updateCamera(v3f pos, v3f dir, f32 fov, v3s16 offset, video::SColor light_color)
{
	//TimeTaker updatecamera_time("Update camera", nullptr, PRECISION_MICRO);
	m_camera_direction_change += dir - m_camera_direction;

	v3s16 previous_node = floatToInt(m_camera_position, BS) + m_camera_offset;
	v3s16 previous_block = getContainerPos(previous_node, MAP_BLOCKSIZE);

	m_camera_position = pos;
	m_camera_direction = dir;
	m_camera_fov = fov;
	m_camera_offset = offset;
	m_camera_light_color = light_color;

	v3s16 current_node = floatToInt(m_camera_position, BS) + m_camera_offset;
	v3s16 current_block = getContainerPos(current_node, MAP_BLOCKSIZE);

	bool cam_block_changed = previous_block != current_block;
	bool cam_dir_changed = m_camera_direction_change.getLengthSQ() >= 0.04f;
	bool wanted_range_changed = m_control.wanted_range != m_last_wanted_range && !m_control.range_all;
	bool range_all_changed = m_control.range_all != m_was_range_all;

	//infostream << "updateCamera() 1" << std::endl;
	if (cam_block_changed || cam_dir_changed || wanted_range_changed || range_all_changed) {
		//infostream << "updateCamera() 2" << std::endl;
		//infostream << "rebuilding octree" << std::endl;
		//m_needs_rebuild_octree = true;
		m_needs_frustum_cull_blocks = true;
		m_needs_update_transparent_meshes = true;
		//infostream << "updateCamera() 3" << std::endl;

		if (cam_dir_changed)
			m_camera_direction_change = v3f(0.0f);
		if (wanted_range_changed)
			m_last_wanted_range = m_control.wanted_range;

		if (range_all_changed)
			m_was_range_all = m_control.range_all;
	}

	// reorder transparent meshes when camera crosses node boundary
	//if (previous_node != current_node || m_needs_frustum_cull_blocks) {
		//infostream << "updateCamera() 6" << std::endl;
	//	m_needs_update_transparent_meshes = true;
		//infostream << "updateCamera() 7" << std::endl;
	//}

	//updatecamera_time.stop(true);
}

MapSector * ClientMap::emergeSector(v2s16 p2d)
{
	// Check that it doesn't exist already
	MapSector *sector = getSectorNoGenerate(p2d);

	// Create it if it does not exist yet
	if (!sector) {
		sector = new MapSector(this, p2d, m_gamedef);
		m_sectors[p2d] = sector;
	}

	return sector;
}

MapBlock * ClientMap::emergeBlock(v3s16 p, bool create_blank)
{
	MapSector *sector = emergeSector(v2s16(p.X, p.Z));

	MapBlock *block = sector->getBlockNoCreateNoEx(p.Y);

	if (!block && create_blank) {
		block = sector->createBlankBlock(p.Y);

		m_new_mapblocks.push_back(block);
		m_needs_frustum_cull_blocks = true;
	}

	return block;
}

void ClientMap::pushDeletedBlocks(const std::vector<v3s16> &deleted_blocks)
{
	for (auto &del_block_p : deleted_blocks)
		m_delete_mapblocks.push_back(del_block_p);

	m_needs_frustum_cull_blocks = true;
}

void ClientMap::OnRegisterSceneNode()
{
	if (IsVisible)
	{
		SceneManager->registerNodeForRendering(this, scene::ESNRP_SOLID);
		SceneManager->registerNodeForRendering(this, scene::ESNRP_TRANSPARENT);
	}

	ISceneNode::OnRegisterSceneNode();
	// It's not needed to register this node to the shadow renderer
	// we have other way to find it
}

void ClientMap::updateOctrees()
{
	//infostream << "updateOctrees() m_new_mapblocks count: " << m_new_mapblocks.size() << std::endl;
	TimeTaker update_octrees_time("Update octrees", nullptr, PRECISION_MICRO);
	ScopeProfiler sp(g_profiler, "CM::updateOctrees()", SPT_AVG);

	for (; !m_new_mapblocks.empty(); m_new_mapblocks.pop_front()) {
		MapBlock *cur_mp = m_new_mapblocks.front();

		v3s16 new_mp_pos = cur_mp->getPos();
		//infostream << "updateOctrees() new mapblock pos: " << new_mp_pos.X << ", " << new_mp_pos.Y << ", " << new_mp_pos.Z << std::endl;
		v3s16 oct_pos = getContainerPos(cur_mp->getPos(), OCTREE_SIZE);
		//infostream << "updateOctrees() octree pos for this mapblock: " << oct_pos.X << ", " << oct_pos.Y << ", " << oct_pos.Z << std::endl;
		auto oct_it = m_octrees.find(oct_pos);

		Octree *oct = nullptr;

		if (oct_it == m_octrees.end()) {
			oct = new Octree();
            oct->buildTree(oct_pos * OCTREE_SIZE, OCTREE_SIZE, m_control);
            m_octrees.emplace(oct_pos, oct);
		}
		else
			oct = oct_it->second;

        oct->traverseTreeForAdd(cur_mp);
	}

	//infostream << "updateOctrees() m_delete_mapblocks count: " << m_new_mapblocks.size() << std::endl;
	for (; !m_delete_mapblocks.empty(); m_delete_mapblocks.pop_front()) {
		v3s16 cur_mp_pos = m_delete_mapblocks.front();

		auto oct_it = m_octrees.find(getContainerPos(cur_mp_pos, OCTREE_SIZE));

		if (oct_it != m_octrees.end())
            oct_it->second->traverseTreeForDelete(cur_mp_pos);
	}

	update_octrees_time.stop(false);
}

void ClientMap::frustumCull()
{
	if (!m_needs_frustum_cull_blocks)
		return;

	infostream << "m_octree count: " << m_octrees.size() << std::endl;
	ScopeProfiler sp(g_profiler, "CM::frustumCull()", SPT_AVG);

    m_levels = std::map<v3f, OctreeNode *, PositionComparer>(m_camera_position);

	// Number of blocks frustum culled
	u32 blocks_frustum_culled = 0;

	if (!m_client->getCamera()) {
        return;
    }

    auto frustum_planes = m_client->getCamera()->getFrustumCullPlanes();

	TimeTaker frustum_culling_time("Frustum culling", nullptr, PRECISION_MICRO);

	u32 octnodes_count = 0;
	auto frustum_cull = [&] (const OctreeNode *node) {
		octnodes_count++;

		infostream << "frustumCull() current traversing node size: " << node->getSize() << std::endl;
        v3f center = node->getCenter();
        v3f pos_camspace = center - intToFloat(m_camera_offset, BS);

        f32 radius = node->getRadius();

        bool fully_inside_frustum = true;

        s16 volume = node->getVolume();
		for (auto &plane : frustum_planes) {
            f32 dist = plane.getDistanceTo(pos_camspace);
            // If true, the node is fully outside the frustum
            if (dist >= radius) {
                blocks_frustum_culled += volume;
                return false;
            }
            // If true, the node is partially inside the frustum
            else if (std::fabs(dist) < radius) {
                fully_inside_frustum = false;
                break;
            }
		}

        s16 size = node->getSize();
        if (fully_inside_frustum || size == 1) {
            m_levels.emplace(center, const_cast<OctreeNode *>(node));
            return false;
        }

        return true;
	};

    for (auto &octree : m_octrees) {
        octree.second->traverseTree(frustum_cull);
		infostream << "frustumCull() count of traversed nodes of the octree: " << octnodes_count << std::endl;
		octnodes_count = 0;
	}

    frustum_culling_time.stop(false);

    g_profiler->avg("MapBlocks frustum culled [#]", blocks_frustum_culled);
}

void ClientMap::updateStorages()
{
    if (!m_needs_frustum_cull_blocks)
        return;

    m_needs_frustum_cull_blocks = false;
	m_storages_updated = true;

    std::list<std::pair<OctreeNode *, std::list<MeshLayer *>>> octnodes_layers;
    std::list<MeshLayer *> octnode_layers;

    auto merge_octnode_mesh = [&octnode_layers] (const OctreeNode *node) {
        auto mesh = node->getMapblockMesh();

        if (mesh) { // only octree leaves have mapblocks
            for (std::pair<video::SMaterial, MeshPart> mp_layer : mesh->layers) {
                auto layer_it = std::find_if(octnode_layers.begin(), octnode_layers.end(),
                    [&mp_layer] (MeshLayer *layer)
                    {
                        return layer->material == mp_layer.first;
                    }
                );

                if (layer_it == octnode_layers.end()) {
                    octnode_layers.push_back(new MeshLayer(mp_layer.first));
                    layer_it = std::prev(octnode_layers.end());
                }

                auto &last_part = (*layer_it)->merged_mesh.back();

                auto &new_mesh = mp_layer.second;

                if (last_part.vertices.size() + new_mesh.vertices.size() > 1e6) {
                    (*layer_it)->merged_mesh.push_back(MeshPart());
                    last_part = (*layer_it)->merged_mesh.back();
                }

                u32 vertex_count = last_part.vertices.size();

                last_part.vertices.insert(last_part.vertices.end(), new_mesh.vertices.begin(), new_mesh.vertices.end());

                for (auto &ind : new_mesh.indices)
                    ind += vertex_count;

                last_part.indices.insert(last_part.indices.end(), new_mesh.indices.begin(), new_mesh.indices.end());
            }

            return false;
        }

        return true;
    };

    for (auto &octnode : m_levels) {
        OctreeNode *oct_n = octnode.second;

        if (!oct_n->storage)
            oct_n->storage = new MeshStorage(m_client);

        if (oct_n->need_rebuild) {
            oct_n->need_rebuild = false;
            oct_n->traverseNode(merge_octnode_mesh);
        }

        octnodes_layers.emplace_back(oct_n, octnode_layers);

        octnode_layers.clear();
    }

    MutexAutoLock storages_lock(m_storages_mutex);

    m_render_storages.clear();

    for (auto &node_layers : octnodes_layers) {
        if (!node_layers.second.empty())
            node_layers.first->storage->mergeNewLayers(node_layers.second);
        m_render_storages.push_back(node_layers.first->storage);
    }
}

void ClientMap::rebuildBuffers(video::IVideoDriver *driver)
{
	if (!m_storages_updated)
		return;

	m_storages_updated = false;

    MutexAutoLock storage_lock(m_storages_mutex);

    for (auto storage : m_render_storages)
        storage->rebuildSolidBuffers(driver);
}

/*void ClientMap::rebuildOctree()
{
	if (!m_needs_rebuild_octree)
		return;

	TimeTaker octree_build_time("Building octree", nullptr, PRECISION_MICRO);
	ScopeProfiler sp(g_profiler, "CM::rebuildOctree()", SPT_AVG);

	m_octree.buildTree(m_sectors, floatToInt(m_camera_position, BS), m_control);

	m_needs_rebuild_octree = false;
	octree_build_time.stop(false);
}*/

/*void ClientMap::frustumCull()
{
	if (!m_needs_frustum_cull_blocks)
		return;

	ScopeProfiler sp(g_profiler, "CM::frustumCull()", SPT_AVG);

	m_needs_frustum_cull_blocks = false;

	v3s16 camera_blocks_pos = getContainerPos(floatToInt(m_camera_position, BS), MAP_BLOCKSIZE);
	m_sorted_mapblocks = std::map<v3s16, MapBlock *, MapBlockComparer>(camera_blocks_pos);
	// Number of blocks occlusion culled
	//u32 blocks_occlusion_culled = 0;
	// Number of blocks frustum culled
	u32 blocks_frustum_culled = 0;

    if (!m_client->getCamera()) {
        return;
    }

	auto is_frustum_culled = m_client->getCamera()->getFrustumCuller();

	TimeTaker frustum_culling_time("Frustum culling", nullptr, PRECISION_MICRO);

	auto frustum_cull = [&] (const OctreeNode *node) {
		auto box = node->getBox();
		box.MaxEdge += v3s16(1, 1, 1);

		v3f box_center = intToFloat(box.getCenter(), BS);
		f32 box_radius = intToFloat((box.MaxEdge - box.MinEdge) / 2, BS).getLength();

		if (is_frustum_culled(box_center, box_radius)) {
			blocks_frustum_culled += node->getFullCountOfMapBlocks();

			return false;
		}

		auto blocks = node->getMapBlocks();

		for (auto block : blocks) {
			if (!block->mesh) {
				blocks_frustum_culled++;
				continue;
			}
			v3s16 block_pos_nodes = block->getPosRelative();

			v3f mesh_sphere_center = intToFloat(block_pos_nodes, BS)
					+ block->mesh->getBoundingSphereCenter();
			f32 mesh_sphere_radius = block->mesh->getBoundingRadius();

			if (is_frustum_culled(mesh_sphere_center,
					mesh_sphere_radius)) {
				blocks_frustum_culled++;
				continue;
			}

			m_sorted_mapblocks[block->getPos()] = block;
		}

		return true;
	};

	m_octree.traverseTree(frustum_cull);

	frustum_culling_time.stop(false);

	infostream << "Visible mapblocks count: " << m_sorted_mapblocks.size() << std::endl;
	//infostream << "frustumCull() blocks_frustum_culled = " << blocks_frustum_culled << std::endl;
	g_profiler->avg("MapBlocks frustum culled [#]", blocks_frustum_culled);
}*/

/*void ClientMap::updateDrawBuffers()
{
	//infostream << "updateDrawBuffers() 1" << std::endl;
	//infostream << "updateDrawBuffers() visible meshes count: " << m_sorted_mapblocks.size() << std::endl;
	if (m_sorted_mapblocks.empty() || !m_needs_update_transparent_meshes)
		return;

	//infostream << "updateDrawBuffers() 2" << std::endl;
	ScopeProfiler sp(g_profiler, "CM::updateDrawBuffers()", SPT_AVG);

	TimeTaker updatebuffers_time("Update buffers", nullptr, PRECISION_MICRO);

	u32 sorted_blocks = 0;

	std::list<MeshLayer *> new_layers;

	//infostream << "updateDrawBuffers() 3" << std::endl;
	for (auto &block_p : m_sorted_mapblocks) {
		MapBlock *block = block_p.second;
		if (!block->mesh)
			continue;

		auto block_mesh = block->mesh->getMesh();

		for (std::pair<video::SMaterial, MeshPart> block_layer : block_mesh->layers) {
			auto layer_it = std::find_if(new_layers.begin(), new_layers.end(),
				[&block_layer] (MeshLayer *layer)
				{
					return layer->material == block_layer.first;
				});

			if (layer_it == new_layers.end()) {
				new_layers.push_back(new MeshLayer(block_layer.first));
				layer_it = std::prev(new_layers.end());
			}

			std::list<MeshPart> &mesh_list = (*layer_it)->merged_mesh;
			MeshPart &last_part = mesh_list.back();
			MeshPart &new_part = block_layer.second;

			if (last_part.vertices.size() + new_part.vertices.size() > U32_MAX) {
				mesh_list.emplace_back();
				last_part = mesh_list.back();
			}

			u32 vertex_count = last_part.vertices.size();

			last_part.vertices.insert(last_part.vertices.end(), new_part.vertices.begin(), new_part.vertices.end());

			for (auto &index : new_part.indices)
				index += vertex_count;

			last_part.indices.insert(last_part.indices.end(), new_part.indices.begin(), new_part.indices.end());
		}
	}

	m_mesh_storage.mergeNewLayers(new_layers);

	m_needs_update_transparent_meshes = false;

	updatebuffers_time.stop(false);

	g_profiler->avg("CM::Transparent Buffers - Sorted", sorted_blocks);
}*/

void ClientMap::updateLighting()
{
	ScopeProfiler sp(g_profiler, "CM::updateLighting()", SPT_AVG);

	//TimeTaker updatelight_time("Update light", nullptr, PRECISION_MICRO);

	u32 daynight_ratio = m_client->getEnv().getDayNightRatio();

	if (m_enable_shaders || (daynight_ratio == m_last_daynight_ratio))
		return;

	video::SColorf day_color;
	get_sunlight_color(&day_color, daynight_ratio);

	m_storages_updated = true;

	MutexAutoLock storage_lock(m_storages_mutex);

    for (auto storage : m_render_storages)
		storage->updateLighting(day_color);

	m_last_daynight_ratio = daynight_ratio;

	//updatelight_time.stop(false);
}

void ClientMap::touchMapBlocks()
{
	if (m_levels.empty())
		return;

	auto reset_timer = [] (const OctreeNode *node)
	{
		bool reset_result = node->resetMapblockTimer();

		if (reset_result)
			return false;

		return true;
	};

	for (auto &octnode : m_levels)
		octnode.second->traverseNode(reset_timer);
}

/*void ClientMap::rebuildVBOs(video::IVideoDriver* driver)
{
	//TimeTaker rebuild_vbos_time("Rebuilding VBOs", nullptr, PRECISION_MICRO);
	m_mesh_storage.rebuildSolidVBOs(driver, m_solid_vbos);
	//m_mesh_storage.rebuildTransparentVBOs(driver, m_transparent_vbos);
	//rebuild_vbos_time.stop(false);
}*/

void ClientMap::renderMap(video::IVideoDriver* driver, s32 pass)
{
	std::string prefix;
	if (pass == scene::ESNRP_SOLID)
		prefix = "renderMap(SOLID): ";
	else
		prefix = "renderMap(TRANSPARENT): ";

	u32 drawcall_count = 0;

	TimeTaker draw("Drawing mesh buffers", nullptr, PRECISION_MICRO);

	core::matrix4 m; // Model matrix
	v3f offset = intToFloat(m_camera_offset, BS);
	m.setTranslation(-offset);

	driver->setTransform(video::ETS_WORLD, m);

	if (pass == scene::ESNRP_SOLID)
        for (auto storage : m_render_storages)
            storage->renderBuffers(driver, m_control.show_wireframe, drawcall_count);

	//infostream << "renderMap() solid materials count: " << materials_count << std::endl;
	//infostream << "renderMap() drawcall_count = " << drawcall_count << std::endl;
	//infostream << "renderMap() count of mesh buffers drawn: " << draw_order.size() << std::endl;

	g_profiler->avg(prefix + "draw meshes [ms]", draw.stop(true));
	g_profiler->avg(prefix + "drawcalls [#]", drawcall_count);
}

/*void ClientMap::updateDrawBuffers()
{
	ScopeProfiler sp(g_profiler, "CM::updateDrawList()", SPT_AVG);

	m_needs_update_drawlist = false;

	for (auto &i : m_drawlist) {
		MapBlock *block = i.second;
		block->refDrop();
	}
	m_drawlist.clear();

	for (auto &block : m_keeplist) {
		block->refDrop();
	}
	m_keeplist.clear();

	const v3s16 cam_pos_nodes = floatToInt(m_camera_position, BS);

	v3s16 p_blocks_min;
	v3s16 p_blocks_max;
	getBlocksInViewRange(cam_pos_nodes, &p_blocks_min, &p_blocks_max);

	// Number of blocks occlusion culled
	u32 blocks_occlusion_culled = 0;
	// Number of blocks frustum culled
	u32 blocks_frustum_culled = 0;

	MeshGrid mesh_grid = m_client->getMeshGrid();

	// No occlusion culling when free_move is on and camera is inside ground
	// No occlusion culling for chunk sizes of 4 and above
	//   because the current occlusion culling test is highly inefficient at these sizes
	bool occlusion_culling_enabled = mesh_grid.cell_size < 4;
	if (m_control.allow_noclip) {
		MapNode n = getNode(cam_pos_nodes);
		if (n.getContent() == CONTENT_IGNORE || m_nodedef->get(n).solidness == 2)
			occlusion_culling_enabled = false;
	}

	const v3s16 camera_block = getContainerPos(cam_pos_nodes, MAP_BLOCKSIZE);
	m_drawlist = std::map<v3s16, MapBlock*, MapBlockComparer>(MapBlockComparer(camera_block));

	auto is_frustum_culled = m_client->getCamera()->getFrustumCuller();

	// Uncomment to debug occluded blocks in the wireframe mode
	// TODO: Include this as a flag for an extended debugging setting
	// if (occlusion_culling_enabled && m_control.show_wireframe)
	// 	occlusion_culling_enabled = porting::getTimeS() & 1;

	// Set of mesh holding blocks
	std::set<v3s16> shortlist;*/

	/*
	 When range_all is enabled, enumerate all blocks visible in the
	 frustum and display them.
	 */
	/*if (m_control.range_all || m_loops_occlusion_culler) {
		// Number of blocks currently loaded by the client
		u32 blocks_loaded = 0;
		// Number of blocks with mesh in rendering range
		u32 blocks_in_range_with_mesh = 0;

		MapBlockVect sectorblocks;

		for (auto &sector_it : m_sectors) {
			const MapSector *sector = sector_it.second;
			v2s16 sp = sector->getPos();

			blocks_loaded += sector->size();
			if (!m_control.range_all) {
				if (sp.X < p_blocks_min.X || sp.X > p_blocks_max.X ||
						sp.Y < p_blocks_min.Z || sp.Y > p_blocks_max.Z)
					continue;
			}

			// Loop through blocks in sector
			for (const auto &entry : sector->getBlocks()) {
				MapBlock *block = entry.second.get();
				MapBlockMesh *mesh = block->mesh;

				// Calculate the coordinates for range and frustum culling
				v3f mesh_sphere_center;
				f32 mesh_sphere_radius;

				v3s16 block_pos_nodes = block->getPosRelative();

				if (mesh) {
					mesh_sphere_center = intToFloat(block_pos_nodes, BS)
							+ mesh->getBoundingSphereCenter();
					mesh_sphere_radius = mesh->getBoundingRadius();
				} else {
					mesh_sphere_center = intToFloat(block_pos_nodes, BS)
							+ v3f((MAP_BLOCKSIZE * 0.5f - 0.5f) * BS);
					mesh_sphere_radius = 0.0f;
				}

				// First, perform a simple distance check.
				if (!m_control.range_all &&
					mesh_sphere_center.getDistanceFrom(m_camera_position) >
						m_control.wanted_range * BS + mesh_sphere_radius)
					continue; // Out of range, skip.

				// Keep the block alive as long as it is in range.
				block->resetUsageTimer();
				blocks_in_range_with_mesh++;

				// Frustum culling
				// Only do coarse culling here, to account for fast camera movement.
				// This is needed because this function is not called every frame.
				float frustum_cull_extra_radius = 300.0f;
				if (is_frustum_culled(mesh_sphere_center,
						mesh_sphere_radius)) {
					blocks_frustum_culled++;
					continue;
				}

				// Raytraced occlusion culling - send rays from the camera to the block's corners
				if (!m_control.range_all && occlusion_culling_enabled && m_enable_raytraced_culling &&
						mesh &&
						isMeshOccluded(block, mesh_grid.cell_size, cam_pos_nodes)) {
					blocks_occlusion_culled++;
					continue;
				}

				if (mesh_grid.cell_size > 1) {
					// Block meshes are stored in the corner block of a chunk
					// (where all coordinate are divisible by the chunk size)
					// Add them to the de-dup set.
					shortlist.emplace(mesh_grid.getMeshPos(block->getPos()));
					// All other blocks we can grab and add to the keeplist right away.
					m_keeplist.push_back(block);
					block->refGrab();
				} else if (mesh) {
					// without mesh chunking we can add the block to the drawlist
					block->refGrab();
					m_drawlist.emplace(block->getPos(), block);
				}
			}
		}

		g_profiler->avg("MapBlock meshes in range [#]", blocks_in_range_with_mesh);
		g_profiler->avg("MapBlocks loaded [#]", blocks_loaded);
	} else {
		// Blocks visited by the algorithm
		u32 blocks_visited = 0;
		// Block sides that were not traversed
		u32 sides_skipped = 0;

		std::queue<v3s16> blocks_to_consider;

		v3s16 camera_mesh = mesh_grid.getMeshPos(camera_block);
		v3s16 camera_cell = mesh_grid.getCellPos(camera_block);

		// Bits per block:
		// [ visited | 0 | 0 | 0 | 0 | Z visible | Y visible | X visible ]
		MapBlockFlags meshes_seen(mesh_grid.getCellPos(p_blocks_min), mesh_grid.getCellPos(p_blocks_max) + 1);

		// Start breadth-first search with the block the camera is in
		blocks_to_consider.push(camera_mesh);
		meshes_seen.getChunk(camera_cell).getBits(camera_cell) = 0x07; // mark all sides as visible

		// Recursively walk the space and pick mapblocks for drawing
		while (!blocks_to_consider.empty()) {

			v3s16 block_coord = blocks_to_consider.front();
			blocks_to_consider.pop();

			v3s16 cell_coord = mesh_grid.getCellPos(block_coord);
			auto &flags = meshes_seen.getChunk(cell_coord).getBits(cell_coord);

			// Only visit each block once (it may have been queued up to three times)
			if ((flags & 0x80) == 0x80)
				continue;
			flags |= 0x80;

			blocks_visited++;

			// Get the sector, block and mesh
			MapSector *sector = this->getSectorNoGenerate(v2s16(block_coord.X, block_coord.Z));

			MapBlock *block = sector ? sector->getBlockNoCreateNoEx(block_coord.Y) : nullptr;

			MapBlockMesh *mesh = block ? block->mesh : nullptr;

			// Calculate the coordinates for range and frustum culling
			v3f mesh_sphere_center;
			f32 mesh_sphere_radius;

			v3s16 block_pos_nodes = block_coord * MAP_BLOCKSIZE;

			if (mesh) {
				mesh_sphere_center = intToFloat(block_pos_nodes, BS)
						+ mesh->getBoundingSphereCenter();
				mesh_sphere_radius = mesh->getBoundingRadius();
			} else {
				mesh_sphere_center = intToFloat(block_pos_nodes, BS) + v3f((mesh_grid.cell_size * MAP_BLOCKSIZE * 0.5f - 0.5f) * BS);
				mesh_sphere_radius = 0.87f * mesh_grid.cell_size * MAP_BLOCKSIZE * BS;
			}

			// First, perform a simple distance check.
			if (!m_control.range_all &&
				mesh_sphere_center.getDistanceFrom(intToFloat(cam_pos_nodes, BS)) >
					m_control.wanted_range * BS + mesh_sphere_radius)
				continue; // Out of range, skip.

			// Frustum culling
			// Only do coarse culling here, to account for fast camera movement.
			// This is needed because this function is not called every frame.
			float frustum_cull_extra_radius = 300.0f;
			if (is_frustum_culled(mesh_sphere_center,
					mesh_sphere_radius)) {
				blocks_frustum_culled++;
				continue;
			}

			// Calculate the vector from the camera block to the current block
			// We use it to determine through which sides of the current block we can continue the search
			v3s16 look = block_coord - camera_mesh;

			// Occluded near sides will further occlude the far sides
			u8 visible_outer_sides = flags & 0x07;

			// Raytraced occlusion culling - send rays from the camera to the block's corners
			if (occlusion_culling_enabled && m_enable_raytraced_culling &&
					block && mesh &&
					visible_outer_sides != 0x07 && isMeshOccluded(block, mesh_grid.cell_size, cam_pos_nodes)) {
				blocks_occlusion_culled++;
				continue;
			}

			if (mesh_grid.cell_size > 1) {
				// Block meshes are stored in the corner block of a chunk
				// (where all coordinate are divisible by the chunk size)
				// Add them to the de-dup set.
				shortlist.emplace(block_coord.X, block_coord.Y, block_coord.Z);
				// All other blocks we can grab and add to the keeplist right away.
				if (block) {
					m_keeplist.push_back(block);
					block->refGrab();
				}
			} else if (mesh) {
				// without mesh chunking we can add the block to the drawlist
				block->refGrab();
				m_drawlist.emplace(block_coord, block);
			}

			// Decide which sides to traverse next or to block away

			// First, find the near sides that would occlude the far sides
			// * A near side can itself be occluded by a nearby block (the test above ^^)
			// * A near side can be visible but fully opaque by itself (e.g. ground at the 0 level)

			// mesh solid sides are +Z-Z+Y-Y+X-X
			// if we are inside the block's coordinates on an axis,
			// treat these sides as opaque, as they should not allow to reach the far sides
			u8 block_inner_sides = (look.X == 0 ? 3 : 0) |
				(look.Y == 0 ? 12 : 0) |
				(look.Z == 0 ? 48 : 0);

			// get the mask for the sides that are relevant based on the direction
			u8 near_inner_sides = (look.X > 0 ? 1 : 2) |
					(look.Y > 0 ? 4 : 8) |
					(look.Z > 0 ? 16 : 32);

			// This bitset is +Z-Z+Y-Y+X-X (See MapBlockMesh), and axis is XYZ.
			// Get he block's transparent sides
			u8 transparent_sides = (occlusion_culling_enabled && block) ? ~block->solid_sides : 0x3F;

			// compress block transparent sides to ZYX mask of see-through axes
			u8 near_transparency =  (block_inner_sides == 0x3F) ? near_inner_sides : (transparent_sides & near_inner_sides);

			// when we are inside the camera block, do not block any sides
			if (block_inner_sides == 0x3F)
				block_inner_sides = 0;

			near_transparency &= ~block_inner_sides & 0x3F;

			near_transparency |= (near_transparency >> 1);
			near_transparency = (near_transparency & 1) |
					((near_transparency >> 1) & 2) |
					((near_transparency >> 2) & 4);

			// combine with known visible sides that matter
			near_transparency &= visible_outer_sides;

			// The rule for any far side to be visible:
			// * Any of the adjacent near sides is transparent (different axes)
			// * The opposite near side (same axis) is transparent, if it is the dominant axis of the look vector

			// Calculate vector from camera to mapblock center. Because we only need relation between
			// coordinates we scale by 2 to avoid precision loss.
			v3s16 precise_look = 2 * (block_pos_nodes - cam_pos_nodes) + mesh_grid.cell_size * MAP_BLOCKSIZE - 1;

			// dominant axis flag
			u8 dominant_axis = (abs(precise_look.X) > abs(precise_look.Y) && abs(precise_look.X) > abs(precise_look.Z)) |
						((abs(precise_look.Y) > abs(precise_look.Z) && abs(precise_look.Y) > abs(precise_look.X)) << 1) |
						((abs(precise_look.Z) > abs(precise_look.X) && abs(precise_look.Z) > abs(precise_look.Y)) << 2);

			// Queue next blocks for processing:
			// - Examine "far" sides of the current blocks, i.e. never move towards the camera
			// - Only traverse the sides that are not occluded
			// - Only traverse the sides that are not opaque
			// When queueing, mark the relevant side on the next block as 'visible'
			for (s16 axis = 0; axis < 3; axis++) {

				// Select a bit from transparent_sides for the side
				u8 far_side_mask = 1 << (2 * axis);

				// axis flag
				u8 my_side = 1 << axis;
				u8 adjacent_sides = my_side ^ 0x07;

				auto traverse_far_side = [&](s8 next_pos_offset) {
					// far side is visible if adjacent near sides are transparent, or if opposite side on dominant axis is transparent
					bool side_visible = ((near_transparency & adjacent_sides) | (near_transparency & my_side & dominant_axis)) != 0;
					side_visible = side_visible && ((far_side_mask & transparent_sides) != 0);

					v3s16 next_pos = block_coord;
					next_pos[axis] += next_pos_offset;

					v3s16 next_cell = mesh_grid.getCellPos(next_pos);

					// If a side is a see-through, mark the next block's side as visible, and queue
					if (side_visible) {
						auto &next_flags = meshes_seen.getChunk(next_cell).getBits(next_cell);
						next_flags |= my_side;
						blocks_to_consider.push(next_pos);
					}
					else {
						sides_skipped++;
					}
				};


				// Test the '-' direction of the axis
				if (look[axis] <= 0 && block_coord[axis] > p_blocks_min[axis])
					traverse_far_side(-mesh_grid.cell_size);

				// Test the '+' direction of the axis
				far_side_mask <<= 1;

				if (look[axis] >= 0 && block_coord[axis] < p_blocks_max[axis])
					traverse_far_side(+mesh_grid.cell_size);
			}
		}
		g_profiler->avg("MapBlocks sides skipped [#]", sides_skipped);
		g_profiler->avg("MapBlocks examined [#]", blocks_visited);
	}
	g_profiler->avg("MapBlocks shortlist [#]", shortlist.size());

	f32 sorting_distance_sq = std::pow(m_cache_transparency_sorting_distance * BS, 2.0f);

	assert(m_drawlist.empty() || shortlist.empty());
	for (auto pos : shortlist) {
		MapBlock *block = getBlockNoCreateNoEx(pos);
		if (block) {
			block->refGrab();
			m_drawlist.emplace(pos, block);
		}
	}

	g_profiler->avg("MapBlocks occlusion culled [#]", blocks_occlusion_culled);
	g_profiler->avg("MapBlocks frustum culled [#]", blocks_frustum_culled);
	g_profiler->avg("MapBlocks drawn [#]", m_drawlist.size());
}*/


/*void ClientMap::renderMap(video::IVideoDriver* driver, s32 pass)
{
	bool is_transparent_pass = pass == scene::ESNRP_TRANSPARENT;

	std::string prefix;
	if (pass == scene::ESNRP_SOLID)
		prefix = "renderMap(SOLID): ";
	else
		prefix = "renderMap(TRANSPARENT): ";*/

	/*
		This is called two times per frame, reset on the non-transparent one
	*/
	//if (pass == scene::ESNRP_SOLID)
	//	m_last_drawn_sectors.clear();

	/*
		Get animation parameters
	*/
	/*const float animation_time = m_client->getAnimationTime();
	const int crack = m_client->getCrackLevel();
	const u32 daynight_ratio = m_client->getEnv().getDayNightRatio();

	const v3f camera_position = m_camera_position;*/

	/*
		Get all blocks and draw all visible ones
	*/

	//u32 vertex_count = 0;
	//u32 drawcall_count = 0;

	// For limiting number of mesh animations per frame
	//u32 mesh_animate_count = 0;
	//u32 mesh_animate_count_far = 0;

	/*
		Update transparent meshes
	*/
	//if (is_transparent_pass)
	//	updateTransparentMeshBuffers();

	/*
		Draw the selected MapBlocks
	*/

	/*MeshBufListMaps grouped_buffers;
	std::vector<DrawDescriptor> draw_order;
	video::SMaterial previous_material;

	auto is_frustum_culled = m_client->getCamera()->getFrustumCuller();

	const MeshGrid mesh_grid = m_client->getMeshGrid();

	for (auto &i : m_drawlist) {
		v3s16 block_pos = i.first;
		MapBlock *block = i.second;
		MapBlockMesh *block_mesh = block->mesh;

		// If the mesh of the block happened to get deleted, ignore it
		if (!block_mesh)
			continue;

		// Do exact frustum culling
		// (The one in updateDrawList is only coarse.)
		//v3f mesh_sphere_center = intToFloat(block->getPosRelative(), BS)
		//		+ block_mesh->getBoundingSphereCenter();
		//f32 mesh_sphere_radius = block_mesh->getBoundingRadius();
		//if (is_frustum_culled(mesh_sphere_center, mesh_sphere_radius))
		//	continue;

		v3f block_pos_r = intToFloat(block->getPosRelative() + MAP_BLOCKSIZE / 2, BS);

		float d = camera_position.getDistanceFrom(block_pos_r);
		d = MYMAX(0,d - BLOCK_MAX_RADIUS);

		// Mesh animation
		if (pass == scene::ESNRP_SOLID) {
			// Pretty random but this should work somewhat nicely
			bool faraway = d >= BS * 50;
			if (block_mesh->isAnimationForced() || !faraway ||
					mesh_animate_count < (m_control.range_all ? 200 : 50)) {

				bool animated = block_mesh->animate(faraway, animation_time,
					crack, daynight_ratio);
				if (animated)
					mesh_animate_count++;
			} else {
				block_mesh->decreaseAnimationForceTimer();
			}
		}*/

		/*
			Get the meshbuffers of the block
		*/
		/*if (is_transparent_pass) {
			// In transparent pass, the mesh will give us
			// the partial buffers in the correct order
			for (auto &buffer : block_mesh->getTransparentBuffers())
				draw_order.emplace_back(block_pos, &buffer);
		}
		else {
			// otherwise, group buffers across meshes
			// using MeshBufListMaps
			for (int layer = 0; layer < MAX_TILE_LAYERS; layer++) {
				scene::IMesh *mesh = block_mesh->getMesh(layer);
				assert(mesh);

				u32 c = mesh->getMeshBufferCount();
				for (u32 i = 0; i < c; i++) {
					scene::IMeshBuffer *buf = mesh->getMeshBuffer(i);

					video::SMaterial& material = buf->getMaterial();
					video::IMaterialRenderer* rnd =
							driver->getMaterialRenderer(material.MaterialType);
					bool transparent = (rnd && rnd->isTransparent());
					if (!transparent) {
						if (buf->getVertexCount() == 0)
							errorstream << "Block [" << analyze_block(block)
									<< "] contains an empty meshbuf" << std::endl;

						grouped_buffers.add(buf, block_pos, layer);
					}
				}
			}
		}
	}

	//infostream << "renderMap() count of transparent meshbuffers: " << draw_order.size() << std::endl;
	//infostream << "renderMap() count of mapblock meshes: " << mapblock_count << std::endl;

	// Capture draw order for all solid meshes
	for (auto &map : grouped_buffers.maps) {
		for (auto &list : map) {
			// iterate in reverse to draw closest blocks first
			for (auto it = list.second.rbegin(); it != list.second.rend(); ++it) {
				draw_order.emplace_back(it->first, it->second, it != list.second.rbegin());
			}
		}
	}

	//infostream << "renderMap() common count of meshbuffers: " << draw_order.size() << std::endl;

	TimeTaker draw("Drawing mesh buffers");

	core::matrix4 m; // Model matrix
	v3f offset = intToFloat(m_camera_offset, BS);
	u32 material_swaps = 0;

	// Render all mesh buffers in order
	drawcall_count += draw_order.size();

	for (auto &descriptor : draw_order) {
		scene::IMeshBuffer *buf = descriptor.getBuffer();

		if (!descriptor.m_reuse_material) {
			auto &material = buf->getMaterial();

			// Apply filter settings
			material.forEachTexture([this] (auto &tex) {
				setMaterialFilters(tex, m_cache_bilinear_filter, m_cache_trilinear_filter,
						m_cache_anistropic_filter);
			});
			material.Wireframe = m_control.show_wireframe;

			// pass the shadow map texture to the buffer texture
			ShadowRenderer *shadow = m_rendering_engine->get_shadow_renderer();
			if (shadow && shadow->is_active()) {
				auto &layer = material.TextureLayers[ShadowRenderer::TEXTURE_LAYER_SHADOW];
				layer.Texture = shadow->get_texture();
				layer.TextureWrapU = video::E_TEXTURE_CLAMP::ETC_CLAMP_TO_EDGE;
				layer.TextureWrapV = video::E_TEXTURE_CLAMP::ETC_CLAMP_TO_EDGE;
				// Do not enable filter on shadow texture to avoid visual artifacts
				// with colored shadows.
				// Filtering is done in shader code anyway
				layer.MinFilter = video::ETMINF_NEAREST_MIPMAP_NEAREST;
				layer.MagFilter = video::ETMAGF_NEAREST;
				layer.AnisotropicFilter = 0;
			}
			//set_material_c++;

			driver->setMaterial(material);
			++material_swaps;
			material.TextureLayers[ShadowRenderer::TEXTURE_LAYER_SHADOW].Texture = nullptr;
		}

		v3f block_wpos = intToFloat(mesh_grid.getMeshPos(descriptor.m_pos) * MAP_BLOCKSIZE, BS);
		m.setTranslation(block_wpos - offset);

		driver->setTransform(video::ETS_WORLD, m);
		descriptor.draw(driver);
		vertex_count += buf->getIndexCount();
	}

	//infostream << "renderMap() count of mesh buffers drawn: " << draw_order.size() << std::endl;

	g_profiler->avg(prefix + "draw meshes [ms]", draw.stop(true));

	// Log only on solid pass because values are the same
	if (pass == scene::ESNRP_SOLID) {
		g_profiler->avg("renderMap(): animated meshes [#]", mesh_animate_count);
	}

	if (pass == scene::ESNRP_TRANSPARENT) {
		g_profiler->avg("renderMap(): transparent buffers [#]", draw_order.size());
	}

	g_profiler->avg(prefix + "vertices drawn [#]", vertex_count);
	g_profiler->avg(prefix + "drawcalls [#]", drawcall_count);
	g_profiler->avg(prefix + "material swaps [#]", material_swaps);
}*/

static bool getVisibleBrightness(Map *map, const v3f &p0, v3f dir, float step,
	float step_multiplier, float start_distance, float end_distance,
	const NodeDefManager *ndef, u32 daylight_factor, float sunlight_min_d,
	int *result, bool *sunlight_seen)
{
	int brightness_sum = 0;
	int brightness_count = 0;
	float distance = start_distance;
	dir.normalize();
	v3f pf = p0;
	pf += dir * distance;
	int noncount = 0;
	bool nonlight_seen = false;
	bool allow_allowing_non_sunlight_propagates = false;
	bool allow_non_sunlight_propagates = false;
	// Check content nearly at camera position
	{
		v3s16 p = floatToInt(p0 /*+ dir * 3*BS*/, BS);
		MapNode n = map->getNode(p);
		if(ndef->getLightingFlags(n).has_light &&
				!ndef->getLightingFlags(n).sunlight_propagates)
			allow_allowing_non_sunlight_propagates = true;
	}
	// If would start at CONTENT_IGNORE, start closer
	{
		v3s16 p = floatToInt(pf, BS);
		MapNode n = map->getNode(p);
		if(n.getContent() == CONTENT_IGNORE){
			float newd = 2*BS;
			pf = p0 + dir * 2*newd;
			distance = newd;
			sunlight_min_d = 0;
		}
	}
	for (int i=0; distance < end_distance; i++) {
		pf += dir * step;
		distance += step;
		step *= step_multiplier;

		v3s16 p = floatToInt(pf, BS);
		MapNode n = map->getNode(p);
		ContentLightingFlags f = ndef->getLightingFlags(n);
		if (allow_allowing_non_sunlight_propagates && i == 0 &&
				f.has_light && !f.sunlight_propagates) {
			allow_non_sunlight_propagates = true;
		}

		if (!f.has_light || (!f.sunlight_propagates && !allow_non_sunlight_propagates)){
			nonlight_seen = true;
			noncount++;
			if(noncount >= 4)
				break;
			continue;
		}

		if (distance >= sunlight_min_d && !*sunlight_seen && !nonlight_seen)
			if (n.getLight(LIGHTBANK_DAY, f) == LIGHT_SUN)
				*sunlight_seen = true;
		noncount = 0;
		brightness_sum += decode_light(n.getLightBlend(daylight_factor, f));
		brightness_count++;
	}
	*result = 0;
	if(brightness_count == 0)
		return false;
	*result = brightness_sum / brightness_count;
	/*std::cerr<<"Sampled "<<brightness_count<<" points; result="
			<<(*result)<<std::endl;*/
	return true;
}

int ClientMap::getBackgroundBrightness(float max_d, u32 daylight_factor,
		int oldvalue, bool *sunlight_seen_result)
{
	ScopeProfiler sp(g_profiler, "CM::getBackgroundBrightness", SPT_AVG);
	static v3f z_directions[50] = {
		v3f(-100, 0, 0)
	};
	static f32 z_offsets[50] = {
		-1000,
	};

	if (z_directions[0].X < -99) {
		for (u32 i = 0; i < ARRLEN(z_directions); i++) {
			// Assumes FOV of 72 and 16/9 aspect ratio
			z_directions[i] = v3f(
				0.02 * myrand_range(-100, 100),
				1.0,
				0.01 * myrand_range(-100, 100)
			).normalize();
			z_offsets[i] = 0.01 * myrand_range(0,100);
		}
	}

	int sunlight_seen_count = 0;
	float sunlight_min_d = max_d*0.8;
	if(sunlight_min_d > 35*BS)
		sunlight_min_d = 35*BS;
	std::vector<int> values;
	values.reserve(ARRLEN(z_directions));
	for (u32 i = 0; i < ARRLEN(z_directions); i++) {
		v3f z_dir = z_directions[i];
		core::CMatrix4<f32> a;
		a.buildRotateFromTo(v3f(0,1,0), z_dir);
		v3f dir = m_camera_direction;
		a.rotateVect(dir);
		int br = 0;
		float step = BS*1.5;
		if(max_d > 35*BS)
			step = max_d / 35 * 1.5;
		float off = step * z_offsets[i];
		bool sunlight_seen_now = false;
		bool ok = getVisibleBrightness(this, m_camera_position, dir,
				step, 1.0, max_d*0.6+off, max_d, m_nodedef, daylight_factor,
				sunlight_min_d,
				&br, &sunlight_seen_now);
		if(sunlight_seen_now)
			sunlight_seen_count++;
		if(!ok)
			continue;
		values.push_back(br);
		// Don't try too much if being in the sun is clear
		if(sunlight_seen_count >= 20)
			break;
	}
	int brightness_sum = 0;
	int brightness_count = 0;
	std::sort(values.begin(), values.end());
	u32 num_values_to_use = values.size();
	if(num_values_to_use >= 10)
		num_values_to_use -= num_values_to_use/2;
	else if(num_values_to_use >= 7)
		num_values_to_use -= num_values_to_use/3;
	u32 first_value_i = (values.size() - num_values_to_use) / 2;

	for (u32 i=first_value_i; i < first_value_i + num_values_to_use; i++) {
		brightness_sum += values[i];
		brightness_count++;
	}

	int ret = 0;
	if(brightness_count == 0){
		MapNode n = getNode(floatToInt(m_camera_position, BS));
		ContentLightingFlags f = m_nodedef->getLightingFlags(n);
		if(f.has_light){
			ret = decode_light(n.getLightBlend(daylight_factor, f));
		} else {
			ret = oldvalue;
		}
	} else {
		ret = brightness_sum / brightness_count;
	}

	*sunlight_seen_result = (sunlight_seen_count > 0);
	return ret;
}

void ClientMap::renderPostFx(CameraMode cam_mode)
{
	// Sadly ISceneManager has no "post effects" render pass, in that case we
	// could just register for that and handle it in renderMap().

	MapNode n = getNode(floatToInt(m_camera_position, BS));

	const ContentFeatures& features = m_nodedef->get(n);
	video::SColor post_color = features.post_effect_color;

	if (features.post_effect_color_shaded) {
		auto apply_light = [] (u32 color, u32 light) {
			return core::clamp(core::round32(color * light / 255.0f), 0, 255);
		};
		post_color.setRed(apply_light(post_color.getRed(), m_camera_light_color.getRed()));
		post_color.setGreen(apply_light(post_color.getGreen(), m_camera_light_color.getGreen()));
		post_color.setBlue(apply_light(post_color.getBlue(), m_camera_light_color.getBlue()));
	}

	// If the camera is in a solid node, make everything black.
	// (first person mode only)
	if (features.solidness == 2 && cam_mode == CAMERA_MODE_FIRST &&
			!m_control.allow_noclip) {
		post_color = video::SColor(255, 0, 0, 0);
	}

	if (post_color.getAlpha() != 0) {
		// Draw a full-screen rectangle
		video::IVideoDriver* driver = SceneManager->getVideoDriver();
		v2u32 ss = driver->getScreenSize();
		core::rect<s32> rect(0,0, ss.X, ss.Y);
		driver->draw2DRectangle(post_color, rect);
	}
}

void ClientMap::PrintInfo(std::ostream &out)
{
	out<<"ClientMap: ";
}

void ClientMap::renderMapShadows(video::IVideoDriver *driver,
		const video::SMaterial &material, s32 pass, int frame, int total_frames)
{}
/*void ClientMap::renderMapShadows(video::IVideoDriver *driver,
		const video::SMaterial &material, s32 pass, int frame, int total_frames)
{
	bool is_transparent_pass = pass != scene::ESNRP_SOLID;
	std::string prefix;
	if (is_transparent_pass)
		prefix = "renderMap(SHADOW TRANS): ";
	else
		prefix = "renderMap(SHADOW SOLID): ";

	u32 drawcall_count = 0;
	u32 vertex_count = 0;

	MeshBufListMaps grouped_buffers;
	std::vector<DrawDescriptor> draw_order;


	std::size_t count = 0;
	std::size_t meshes_per_frame = m_drawlist_shadow.size() / total_frames + 1;
	std::size_t low_bound = is_transparent_pass ? 0 : meshes_per_frame * frame;
	std::size_t high_bound = is_transparent_pass ? m_drawlist_shadow.size() : meshes_per_frame * (frame + 1);

	// transparent pass should be rendered in one go
	if (is_transparent_pass && frame != total_frames - 1) {
		return;
	}

	const MeshGrid mesh_grid = m_client->getMeshGrid();
	for (const auto &i : m_drawlist_shadow) {
		// only process specific part of the list & break early
		++count;
		if (count <= low_bound)
			continue;
		if (count > high_bound)
			break;

		v3s16 block_pos = i.first;
		MapBlock *block = i.second;

		// If the mesh of the block happened to get deleted, ignore it
		if (!block->mesh)
			continue;*/

		/*
			Get the meshbuffers of the block
		*/
		/*if (is_transparent_pass) {
			// In transparent pass, the mesh will give us
			// the partial buffers in the correct order
			for (auto &buffer : block->mesh->getTransparentBuffers())
				draw_order.emplace_back(block_pos, &buffer);
		}
		else {
			// otherwise, group buffers across meshes
			// using MeshBufListMaps
			MapBlockMesh *mapBlockMesh = block->mesh;
			assert(mapBlockMesh);

			for (int layer = 0; layer < MAX_TILE_LAYERS; layer++) {
				scene::IMesh *mesh = mapBlockMesh->getMesh(layer);
				assert(mesh);

				u32 c = mesh->getMeshBufferCount();
				for (u32 i = 0; i < c; i++) {
					scene::IMeshBuffer *buf = mesh->getMeshBuffer(i);

					video::SMaterial &mat = buf->getMaterial();
					auto rnd = driver->getMaterialRenderer(mat.MaterialType);
					bool transparent = rnd && rnd->isTransparent();
					if (!transparent)
						grouped_buffers.add(buf, block_pos, layer);
				}
			}
		}
	}

	u32 buffer_count = 0;
	for (auto &map : grouped_buffers.maps)
		for (auto &list : map)
			buffer_count += list.second.size();

	draw_order.reserve(draw_order.size() + buffer_count);

	// Capture draw order for all solid meshes
	for (auto &map : grouped_buffers.maps) {
		for (auto &list : map) {
			// iterate in reverse to draw closest blocks first
			for (auto it = list.second.rbegin(); it != list.second.rend(); ++it)
				draw_order.emplace_back(it->first, it->second, it != list.second.rbegin());
		}
	}

	TimeTaker draw("Drawing shadow mesh buffers");

	core::matrix4 m; // Model matrix
	v3f offset = intToFloat(m_camera_offset, BS);
	u32 material_swaps = 0;

	// Render all mesh buffers in order
	drawcall_count += draw_order.size();

	for (auto &descriptor : draw_order) {
		scene::IMeshBuffer *buf = descriptor.getBuffer();

		if (!descriptor.m_reuse_material) {
			// override some material properties
			video::SMaterial local_material = buf->getMaterial();
			local_material.MaterialType = material.MaterialType;
			// do not override culling if the original material renders both back
			// and front faces in solid mode (e.g. plantlike)
			// Transparent plants would still render shadows only from one side,
			// but this conflicts with water which occurs much more frequently
			if (is_transparent_pass || local_material.BackfaceCulling || local_material.FrontfaceCulling) {
				local_material.BackfaceCulling = material.BackfaceCulling;
				local_material.FrontfaceCulling = material.FrontfaceCulling;
			}
			local_material.BlendOperation = material.BlendOperation;
			local_material.Lighting = false;
			driver->setMaterial(local_material);
			++material_swaps;
		}

		v3f block_wpos = intToFloat(mesh_grid.getMeshPos(descriptor.m_pos) * MAP_BLOCKSIZE, BS);
		m.setTranslation(block_wpos - offset);

		driver->setTransform(video::ETS_WORLD, m);
		descriptor.draw(driver);
		vertex_count += buf->getIndexCount();
	}

	// restore the driver material state
	video::SMaterial clean;
	clean.BlendOperation = video::EBO_ADD;
	driver->setMaterial(clean); // reset material to defaults
	driver->draw3DLine(v3f(), v3f(), video::SColor(0));

	g_profiler->avg(prefix + "draw meshes [ms]", draw.stop(true));
	g_profiler->avg(prefix + "vertices drawn [#]", vertex_count);
	g_profiler->avg(prefix + "drawcalls [#]", drawcall_count);
	g_profiler->avg(prefix + "material swaps [#]", material_swaps);
}*/

void ClientMap::updateDrawListShadow(v3f shadow_light_pos, v3f shadow_light_dir, float radius, float length)
{}
/*
	Custom update draw list for the pov of shadow light.
*/
/*void ClientMap::updateDrawListShadow(v3f shadow_light_pos, v3f shadow_light_dir, float radius, float length)
{
	ScopeProfiler sp(g_profiler, "CM::updateDrawListShadow()", SPT_AVG);

	for (auto &i : m_drawlist_shadow) {
		MapBlock *block = i.second;
		block->refDrop();
	}
	m_drawlist_shadow.clear();

	// Number of blocks currently loaded by the client
	u32 blocks_loaded = 0;
	// Number of blocks with mesh in rendering range
	u32 blocks_in_range_with_mesh = 0;

	for (auto &sector_it : m_sectors) {
		const MapSector *sector = sector_it.second;
		if (!sector)
			continue;
		blocks_loaded += sector->size();*/

		/*
			Loop through blocks in sector
		*/
		/*for (const auto &entry : sector->getBlocks()) {
			MapBlock *block = entry.second.get();
			MapBlockMesh *mesh = block->mesh;
			if (!mesh) {
				// Ignore if mesh doesn't exist
				continue;
			}

			v3f block_pos = intToFloat(block->getPosRelative(), BS) + mesh->getBoundingSphereCenter();
			v3f projection = shadow_light_pos + shadow_light_dir * shadow_light_dir.dotProduct(block_pos - shadow_light_pos);
			if (projection.getDistanceFrom(block_pos) > (radius + mesh->getBoundingRadius()))
				continue;

			blocks_in_range_with_mesh++;

			// This block is in range. Reset usage timer.
			block->resetUsageTimer();

			// Add to set
			if (m_drawlist_shadow.emplace(block->getPos(), block).second) {
				block->refGrab();
			}
		}
	}

	g_profiler->avg("SHADOW MapBlock meshes in range [#]", blocks_in_range_with_mesh);
	g_profiler->avg("SHADOW MapBlocks drawn [#]", m_drawlist_shadow.size());
	g_profiler->avg("SHADOW MapBlocks loaded [#]", blocks_loaded);
}*/

void ClientMap::reportMetrics(u64 save_time_us, u32 saved_blocks, u32 all_blocks)
{
	g_profiler->avg("CM::reportMetrics loaded blocks [#]", all_blocks);
}

/*void ClientMap::updateTransparentMeshBuffers()
{
	ScopeProfiler sp(g_profiler, "CM::updateTransparentMeshBuffers", SPT_AVG);
	u32 sorted_blocks = 0;
	u32 unsorted_blocks = 0;
	f32 sorting_distance_sq = std::pow(m_cache_transparency_sorting_distance * BS, 2.0f);


	// Update the order of transparent mesh buffers in each mesh
	for (auto it = m_drawlist.begin(); it != m_drawlist.end(); it++) {
		MapBlock* block = it->second;
		if (!block->mesh)
			continue;

		if (m_needs_update_transparent_meshes ||
				block->mesh->getTransparentBuffers().size() == 0) {

			v3s16 block_pos = block->getPos();
			v3f block_pos_f = intToFloat(block_pos * MAP_BLOCKSIZE + MAP_BLOCKSIZE / 2, BS);
			f32 distance = m_camera_position.getDistanceFromSQ(block_pos_f);
			if (distance <= sorting_distance_sq) {
				block->mesh->updateTransparentBuffers(m_camera_position, block_pos);
				++sorted_blocks;
			}
			else {
				block->mesh->consolidateTransparentBuffers();
				++unsorted_blocks;
			}
		}
	}

	g_profiler->avg("CM::Transparent Buffers - Sorted", sorted_blocks);
	g_profiler->avg("CM::Transparent Buffers - Unsorted", unsorted_blocks);
	m_needs_update_transparent_meshes = false;
}*/

//scene::IMeshBuffer* ClientMap::DrawDescriptor::getBuffer()
//{
//	return m_use_partial_buffer ? m_partial_buffer->getBuffer() : m_buffer;
//}

/*void ClientMap::DrawDescriptor::draw(video::IVideoDriver* driver)
{
	if (m_use_partial_buffer) {
		m_partial_buffer->beforeDraw();
		driver->drawMeshBuffer(m_partial_buffer->getBuffer());
		m_partial_buffer->afterDraw();
	} else {
		driver->drawMeshBuffer(m_buffer);
	}
}*/

/*bool ClientMap::isMeshOccluded(MapBlock *mesh_block, u16 mesh_size, v3s16 cam_pos_nodes)
{
	if (mesh_size == 1)
		return isBlockOccluded(mesh_block, cam_pos_nodes);

	v3s16 min_edge = mesh_block->getPosRelative();
	v3s16 max_edge = min_edge + mesh_size * MAP_BLOCKSIZE -1;
	bool check_axis[3] = { false, false, false };
	u16 closest_side[3] = { 0, 0, 0 };

	for (int axis = 0; axis < 3; axis++) {
		if (cam_pos_nodes[axis] < min_edge[axis])
			check_axis[axis] = true;
		else if (cam_pos_nodes[axis] > max_edge[axis]) {
			check_axis[axis] = true;
			closest_side[axis] = mesh_size - 1;
		}
	}

	std::vector<bool> processed_blocks(mesh_size * mesh_size * mesh_size);

	// scan the side
	for (u16 i = 0; i < mesh_size; i++)
	for (u16 j = 0; j < mesh_size; j++) {
		v3s16 offsets[3] = {
			v3s16(closest_side[0], i, j),
			v3s16(i, closest_side[1], j),
			v3s16(i, j, closest_side[2])
		};
		for (int axis = 0; axis < 3; axis++) {
			v3s16 offset = offsets[axis];
			int block_index = offset.X + offset.Y * mesh_size + offset.Z * mesh_size * mesh_size;
			if (check_axis[axis] && !processed_blocks[block_index]) {
				processed_blocks[block_index] = true;
				v3s16 block_pos = mesh_block->getPos() + offset;
				MapBlock *block;

				if (mesh_block->getPos() == block_pos)
					block = mesh_block;
				else
					block = getBlockNoCreateNoEx(block_pos);

				if (block && !isBlockOccluded(block, cam_pos_nodes))
					return false;
			}
		}
	}

	return true;
}*/
