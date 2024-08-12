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

#include "client/client.h"
#include "client/mesh_storage.h"
#include "client/texture_atlas.h"
#include "threading/mutex_auto_lock.h"
#include "client/light_colors.h"
#include <cassert>
#include <sstream>
#include <cmath>
#include "log.h"
#include "client/renderingengine.h"
#include "IMaterialRenderer.h"
#include "util/timetaker.h"

template<typename T>
u32 getMinFreeIndex(const std::unordered_map<u32, T> &map)
{
	u32 free_index = 0;

	auto it = map.find(free_index);

	while (it != map.end()) {
		free_index++;
		it = map.find(free_index);
	}

	return free_index;
}

void MeshLayer::mergeArrays(u32 layer_index, const MeshPart &mesh, MeshInfo &mesh_info)
{
	MutexAutoLock arrays_lock(m_arrays_mutex);

	u32 arrays_index = getMinFreeIndex(m_arrays);

	mesh_info.layers_to_arrays_map[layer_index] = arrays_index;

	m_arrays.emplace(arrays_index, mesh);

	arrays_lock.unlock();

	//video::IVideoDriver *driver = RenderingEngine::get_video_driver();
	// Split the mesh part into triangles if it is transparent
	/*if (driver->getMaterialRenderer(material.MaterialType)->isTransparent()) {
		infostream << "mergeArrays() found the transparent material" << std::endl;
		u32 triangles_count = mesh.indices.size() / 3;
		for (u32 t = 0; t < triangles_count; t++) {
			u32 index1 = mesh.indices[t * 3];
			u32 index2 = mesh.indices[t * 3 + 1];
			u32 index3 = mesh.indices[t * 3 + 2];

			auto &verts = mesh.vertices;
			v3f centre = (verts[index1].Pos + verts[index2].Pos + verts[index3].Pos) / 3.0f;

            mesh_info.transparent_triangles.emplace(centre, MeshTriangle(layer_index, arrays_index, {index1, index2, index3}));
		}
	}*/
}

size_t MeshLayer::removeArrays(u32 arrays_index)
{
	size_t size = 0;

	MutexAutoLock arrays_lock(m_arrays_mutex);

	auto arrays_it = m_arrays.find(arrays_index);

	if (arrays_it != m_arrays.end()) {
		size += arrays_it->second.vertices.size() * sizeof(video::S3DVertex);
		size += arrays_it->second.indices.size() * sizeof(u32);

		m_arrays.erase(arrays_it);
	}

	return size;
}

void MeshLayer::prepareSolidMesh(const std::vector<u32> &arrays)
{
	std::vector<MeshPart> solid_mesh;
	solid_mesh.push_back(MeshPart());

	u32 i = 0;
	for (auto array : arrays) {
		TimeTaker array_time("Loop for mesh part array i = " + std::to_string(i), nullptr, PRECISION_MICRO);
		MutexAutoLock arrays_lock(m_arrays_mutex);

		auto arrays_it = m_arrays.find(array);

		if (arrays_it == m_arrays.end())
			continue;

		MeshPart part = arrays_it->second;
		arrays_lock.unlock();
		infostream << "MeshLayer::prepareSolidMesh() 1: " << array_time.getTimerTime() << "us" << std::endl;

		MeshPart &last_mesh = solid_mesh.back();

		if (last_mesh.vertices.size() + part.vertices.size() > U32_MAX) {
			solid_mesh.push_back(MeshPart());
			last_mesh = solid_mesh.back();
		}

		u32 vertex_count = last_mesh.vertices.size();

		last_mesh.vertices.insert(last_mesh.vertices.end(), part.vertices.begin(), part.vertices.end());

		infostream << "MeshLayer::prepareSolidMesh() 2: " << array_time.getTimerTime() << "us" << std::endl;
        for (auto &index : part.indices)
			index += vertex_count;
		infostream << "MeshLayer::prepareSolidMesh() 3: " << array_time.getTimerTime() << "us" << std::endl;

        last_mesh.indices.insert(last_mesh.indices.end(), part.indices.begin(), part.indices.end());
		array_time.stop(false);
		++i;
	}

	MutexAutoLock solid_mesh_lock(m_solid_mesh_mutex);
	TimeTaker solid_mesh_time("Solid mesh copying", nullptr, PRECISION_MICRO);

	m_solid_mesh = solid_mesh;

	// Flag for rebuilding vbos in the clientmap
	needs_rebuild_vbos = true;
	solid_mesh_time.stop(false);
}

void MeshLayer::rebuildSolidVBOs(video::IVideoDriver *driver, std::vector<scene::IVertexBuffer *> &output)
{
	if (!needs_rebuild_vbos)
		return;

	needs_rebuild_vbos = false;

	MutexAutoLock solid_mesh_lock(m_solid_mesh_mutex);

	for (auto &solid_mesh : m_solid_mesh) {
		scene::IVertexBuffer *buffer = driver->createVertexBuffer();
		buffer->formatBuffer();

		buffer->uploadVertexData(solid_mesh.vertices.size(), solid_mesh.vertices.data());
		buffer->uploadIndexData(solid_mesh.indices.size(), solid_mesh.indices.data());

		output.push_back(buffer);
	}
}
/*void MeshLayer::clearVBOs()
{
    MutexAutoLock vbos_lock(m_vbos_mutex);

    for (scene::IVertexBuffer *buffer : m_solid_buffers)
        if (buffer) {
            delete buffer;
            buffer = nullptr;
        }
}

void MeshLayer::addVertexArrayInVBOs(video::IVideoDriver *driver, u32 arrays_index)
{
    if (!needs_rebuild_vbos)
        return;

    MutexAutoLock arrays_lock(m_arrays_mutex);
    auto arrays_it = m_arrays.find(arrays_index);

    if (arrays_it == m_arrays.end())
        return;

	MeshPart part = arrays_it->second;

    arrays_lock.unlock();

    MutexAutoLock vbos_lock(m_vbos_mutex);

    if (!m_cur_buffer) {
        m_cur_buffer = driver->createVertexBuffer();
        m_cur_buffer->formatBuffer();
        m_solid_buffers.push_back(m_cur_buffer);
    }

    if (m_cur_vertices.size() + part.vertices.size() > U32_MAX) {
        m_cur_buffer->uploadVertexData(m_cur_vertices.size(), m_cur_vertices.data());
        m_cur_buffer->uploadIndexData(m_cur_indices.size(), m_cur_indices.data());

        m_cur_vertices.clear();
        m_cur_indices.clear();

        m_cur_buffer = driver->createVertexBuffer();
        m_cur_buffer->formatBuffer();
        m_solid_buffers.push_back(m_cur_buffer);
    }

    vbos_lock.unlock();

    u32 vertex_count = m_cur_vertices.size();

    m_cur_vertices.insert(m_cur_vertices.end(), part.vertices.begin(), part.vertices.end());

    for (auto &ind : part.indices)
        ind += vertex_count;

    m_cur_indices.insert(m_cur_indices.end(), part.indices.begin(), part.indices.end());
}

void MeshLayer::clearTempData()
{
    m_cur_buffer = nullptr;

    m_cur_vertices.clear();
    m_cur_indices.clear();
}*/

void MeshLayer::fetchVertexTriple(u32 arrays_index, const std::array<u32, 3> &indices, std::array<video::S3DVertex, 3> &vertices)
{
    MutexAutoLock arrays_lock(m_arrays_mutex);

	auto arrays_it = m_arrays.find(arrays_index);

	if (arrays_it == m_arrays.end())
		return;

	vertices[0] = arrays_it->second.vertices.at(indices[0]);
	vertices[1] = arrays_it->second.vertices.at(indices[1]);
	vertices[2] = arrays_it->second.vertices.at(indices[2]);
}

void MeshLayer::updateLighting(video::SColorf &day_color)
{
	MutexAutoLock solid_mesh_lock(m_solid_mesh_mutex);

	for (auto &solid_mesh : m_solid_mesh) {
		auto &vertices = solid_mesh.vertices;

		for (u32 i = 0; i < vertices.size(); i++)
			if (vertices[i].Color.getAlpha() != 0)
				final_color_blend(&(vertices[i].Color), vertices[i].Color,
					day_color);
	}

	needs_rebuild_vbos = true;
}

/*void MeshLayer::render(video::IVideoDriver *driver,
	bool wireframe, u32 &drawcall_count)
{
	if (material.isTransparent())
		return;

	video::SMaterial mat_copy(material);
	mat_copy.Wireframe = wireframe;

	driver->setMaterial(mat_copy);

	MutexAutoLock vbos_lock(m_vbos_mutex);

	for (auto buffer : m_solid_buffers) {
		driver->drawVertexBuffer(buffer);

		drawcall_count++;
	}
}*/

MeshStorage::MeshStorage(Client *client)
	: m_client(client)
{}

void MeshStorage::addArrays(const MapblockMeshCollector &collector)
{
	MeshInfo &info = *collector.info;

	std::shared_lock layers_sl(m_layers_mutex);

	//infostream << "addArrays() count of prelayer meshes in mapblock: " << collector.prelayers.size() << std::endl;
    for (const std::pair<video::SMaterial, MeshPart> &prelayer : collector.prelayers) {
		auto layer_it = std::find_if(m_layers.begin(), m_layers.end(),
            [prelayer] (std::pair<u32, MeshLayer *> layer)
			{
				return layer.second->material == prelayer.first;
			});

		u32 layer_index = 0;
		if (layer_it == m_layers.end()) {
			layer_index = getMinFreeIndex(m_layers);

			layers_sl.unlock();

			std::unique_lock layers_ul(m_layers_mutex);
			m_layers[layer_index] = new MeshLayer(prelayer.first);
			layers_ul.unlock();

			layers_sl.lock();
		}
		else
			layer_index = layer_it->first;

		m_layers.at(layer_index)->mergeArrays(layer_index, prelayer.second, info);
	}
}

size_t MeshStorage::deleteArrays(const MeshRef &mesh)
{
	size_t size = 0;

	std::shared_lock layers_sl(m_layers_mutex);

	for (auto p : mesh) {
		auto layer_it = m_layers.find(p.first);

		if (layer_it == m_layers.end())
			continue;

        size += (layer_it->second->removeArrays(p.second));

		// Remove the layer from the storage if it has appeared to be empty
		if (layer_it->second->isEmpty()) {
			layers_sl.unlock();

			std::unique_lock layers_ul(m_layers_mutex);
			delete layer_it->second;
			m_layers.erase(layer_it);
			layers_ul.unlock();

			layers_sl.lock();
		}
	}

	return size;
}

void MeshStorage::prepareSolidMeshes(const std::vector<MeshRef> &mesh_parts)
{
	TimeTaker prepare_solidmeshes_time("Prepare Solid Meshes", nullptr, PRECISION_MICRO);
	std::unordered_map<u32, std::vector<u32>> collected_layers;

	for (auto mesh_part : mesh_parts)
        for (auto layer_p : mesh_part) {
            auto layer_it = collected_layers.find(layer_p.first);

			if (layer_it == collected_layers.end())
				collected_layers.emplace(layer_p.first, std::vector<u32>());

			collected_layers[layer_p.first].push_back(layer_p.second);
		}
	infostream << "prepareSolidMeshes() mesh_parts loop: " << prepare_solidmeshes_time.getTimerTime() << "us" << std::endl;
	std::shared_lock layers_sl(m_layers_mutex);
	infostream << "prepareSolidMeshes() collected_layers count: " << collected_layers.size() << std::endl;

	video::IVideoDriver *driver = RenderingEngine::get_video_driver();
	u32 i = 0;
	for (auto &layer_p : collected_layers) {
		infostream << "prepareSolidMeshes() collected_layers loop layer: " << (i++) << std::endl;
		auto layer_it = m_layers.find(layer_p.first);

		if (layer_it == m_layers.end())
			continue;

		if (driver->getMaterialRenderer(layer_it->second->material.MaterialType)->isTransparent())
			continue;

		layer_it->second->prepareSolidMesh(layer_p.second);
	}
	infostream << "prepareSolidMeshes() collected_layers loop: " << prepare_solidmeshes_time.getTimerTime() << "us" << std::endl;
}

void MeshStorage::prepareTransparentMeshes(const std::map<v3f, MeshTriangle, TriangleComparer> &triangles)
{
	if (triangles.empty())
		return;

	std::list<std::pair<video::SMaterial, MeshPart>> layers;

	std::shared_lock layers_lock(m_layers_mutex);

	layers.emplace_back(m_layers.at(triangles.begin()->second.layer_ref)->material, MeshPart());

	for (auto trig_p : triangles) {
		MeshTriangle trig = trig_p.second;

		auto layer_it = m_layers.find(trig.layer_ref);

		if (layer_it == m_layers.end())
			continue;

		std::pair<video::SMaterial, MeshPart> &last_layer = layers.back();

		MeshLayer *new_layer = layer_it->second;
		if (last_layer.first != new_layer->material || last_layer.second.vertices.size() + 3 > U32_MAX) {
			layers.emplace_back(new_layer->material, MeshPart());
			last_layer = layers.back();
		}

		std::array<video::S3DVertex, 3> vertices;
		new_layer->fetchVertexTriple(trig.arrays_ref, trig.indices, vertices);

		std::vector<video::S3DVertex> &last_vertices = last_layer.second.vertices;
		std::vector<u32> &last_indices = last_layer.second.indices;

		u32 vertex_count = last_vertices.size();

		last_vertices.insert(last_vertices.end(), {vertices[0], vertices[1], vertices[2]});
		last_indices.insert(last_indices.end(), {vertex_count, vertex_count+1, vertex_count+2});
	}

	layers_lock.unlock();

	MutexAutoLock tmesh_lock(m_tmesh_mutex);

	m_transparent_mesh = layers;

	needs_rebuild_tvbos = true;
}

void MeshStorage::updateLighting(video::SColorf &day_color)
{
	std::shared_lock layers_lock(m_layers_mutex);

	// Update for solid meshes
	for (auto &layer : m_layers)
		layer.second->updateLighting(day_color);
	layers_lock.unlock();

	MutexAutoLock tmesh_lock(m_tmesh_mutex);
	// Update for transparent meshes
	for (auto &tlayer : m_transparent_mesh) {
		auto &vertices = tlayer.second.vertices;

		for (u32 i = 0; i < vertices.size(); i++)
			if (vertices[i].Color.getAlpha() != 0)
				final_color_blend(&(vertices[i].Color), vertices[i].Color,
					day_color);
	}

	needs_rebuild_tvbos = true;
}

void MeshStorage::rebuildSolidVBOs(video::IVideoDriver *driver,
	std::vector<std::pair<video::SMaterial, std::vector<scene::IVertexBuffer *>>> &vbos)
{
	std::shared_lock layers_sl(m_layers_mutex);

	for (auto &layer_p : m_layers) {
		std::vector<scene::IVertexBuffer *> layer_vbos;
		layer_p.second->rebuildSolidVBOs(driver, layer_vbos);

		if (!layer_vbos.empty()) {// the layer is rebuilt
			auto vbos_layer_it = std::find_if(vbos.begin(), vbos.end(),
				[layer_p] (std::pair<video::SMaterial, std::vector<scene::IVertexBuffer *>> &layer)
				{
					return layer_p.second->material == layer.first;
				});

			// Before such layer didn`t exist yet
			if (vbos_layer_it == vbos.end())
				vbos.emplace_back(layer_p.second->material, layer_vbos);
			// Already exists, then just clear VBOs of the layer and add the new ones
			else {
				for (auto buffer : vbos_layer_it->second)
					if (buffer)
						delete buffer;

				vbos_layer_it->second = layer_vbos;
			}
		}
	}
}

void MeshStorage::rebuildTransparentVBOs(video::IVideoDriver *driver,
	std::vector<std::pair<video::SMaterial, scene::IVertexBuffer *>> &vbos)
{
	if (!needs_rebuild_tvbos)
		return;

	needs_rebuild_tvbos = false;

	// Clear all old VBOs
	for (auto &tbuffer_p : vbos)
		if (tbuffer_p.second)
			delete tbuffer_p.second;

	vbos.clear();

    MutexAutoLock tbuffers_lock(m_tmesh_mutex);

	for (auto &tmesh_p : m_transparent_mesh) {
		scene::IVertexBuffer *buffer = driver->createVertexBuffer();
		buffer->formatBuffer();

		MeshPart part = tmesh_p.second;
		buffer->uploadVertexData(part.vertices.size(), part.vertices.data());
		buffer->uploadIndexData(part.indices.size(), part.indices.data());

		vbos.emplace_back(tmesh_p.first, buffer);
	}
}

/*void MeshStorage::renderSolidVBOs(video::IVideoDriver *driver,
	bool wireframe, u32 &drawcall_count)
{
	std::shared_lock layers_sl(m_layers_mutex);

    for (auto &layer_p : m_layers)
		layer_p.second->render(driver, wireframe, drawcall_count);
}

void MeshStorage::renderTransparentVBOs(video::IVideoDriver *driver,
	bool wireframe, u32 &drawcall_count)
{
	MutexAutoLock tbuffers_lock(m_tbuffers_mutex);

	for (auto &tbuffer_p : m_transparent_buffers) {
		video::SMaterial mat_copy(tbuffer_p.first);
		mat_copy.Wireframe = wireframe;

		driver->setMaterial(mat_copy);

		driver->drawVertexBuffer(tbuffer_p.second);

		drawcall_count++;
	}
}*/
