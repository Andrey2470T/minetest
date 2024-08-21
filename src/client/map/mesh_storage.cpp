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
#include "mesh_storage.h"
#include "threading/mutex_auto_lock.h"
#include "client/light_colors.h"
#include <cassert>
#include <sstream>
#include <cmath>
#include "log.h"
#include "client/renderingengine.h"
#include "IMaterialRenderer.h"
#include "util/timetaker.h"


MeshStorage::~MeshStorage()
{
    for (auto layer : m_layers)
        delete layer;

    for (auto &buffers_layer : m_buffers)
        for (auto buffer : buffers_layer.second)
            if (buffer)
                delete buffer;
}

void MeshStorage::mergeNewLayers(const std::list<MeshLayer *> &new_layers)
{
	// Remove old layers
	for (auto layer : m_layers)
		delete layer;

	m_layers.clear();

	// Add new ones
	for (auto new_layer : new_layers)
		m_layers.push_back(new_layer);

	storage_updated = true;
}

void MeshStorage::updateLighting(video::SColorf &day_color)
{
	for (auto layer : m_layers)
		for (auto &mesh_part : layer->merged_mesh) {
			auto &vertices = mesh_part.vertices;

			for (u32 i = 0; i < vertices.size(); i++)
				if (vertices[i].Color.getAlpha() != 0)
					final_color_blend(&(vertices[i].Color), vertices[i].Color,
						day_color);
		}

	storage_updated = true;
}

void MeshStorage::rebuildSolidBuffers(video::IVideoDriver *driver)
{
	if (!storage_updated)
		return;

	TimeTaker rebuild_vbos_time("Rebuilding VBOs", nullptr, PRECISION_MICRO);
	storage_updated = false;


	for (auto &buffers_layer : m_buffers)
		for (auto buffer : buffers_layer.second)
			if (buffer)
				delete buffer;

	m_buffers.clear();

	for (auto layer : m_layers) {
        m_buffers.push_back(std::make_pair(layer->material, std::list<scene::IVertexBuffer *>()));

		for (auto &mesh_part : layer->merged_mesh) {
			scene::IVertexBuffer *new_buffer = driver->createVertexBuffer();
			new_buffer->uploadData(mesh_part.vertices.size(), mesh_part.vertices.data(),
				mesh_part.indices.size(), mesh_part.indices.data());

			m_buffers.back().second.push_back(new_buffer);
		}
	}
}

void MeshStorage::renderBuffers(video::IVideoDriver *driver, bool wireframe, u32 &drawcall_count)
{
	for (auto &buffers_layer : m_buffers) {
		video::SMaterial mat_copy(buffers_layer.first);
		mat_copy.Wireframe = wireframe;

		driver->setMaterial(mat_copy);

		for (auto buffer : buffers_layer.second) {
			driver->drawVertexBuffer(buffer);

			drawcall_count++;
		}
	}
}

/*
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
}*/
