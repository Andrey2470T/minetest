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


MeshStorage::MeshStorage(Client *client)
	: m_client(client)
{}

void MeshStorage::mergeNewLayers(const std::list<MeshLayer *> &new_layers)
{
	MutexAutoLock layers_lock(m_layers_mutex);

	// Remove old layers
	for (auto layer : m_layers)
		delete layer;

	m_layers.clear();

	// Add new ones
	for (auto new_layer : new_layers)
		m_layers.push_back(new_layer);

	layers_updated = true;
}

void MeshStorage::updateLighting(video::SColorf &day_color)
{
	MutexAutoLock layers_lock(m_layers_mutex);

	// Update for solid meshes
	for (auto layer : m_layers)
		for (auto &mesh_part : layer->merged_mesh) {
			auto &vertices = mesh_part.vertices;

			for (u32 i = 0; i < vertices.size(); i++)
				if (vertices[i].Color.getAlpha() != 0)
					final_color_blend(&(vertices[i].Color), vertices[i].Color,
						day_color);
		}

	layers_updated = true;
}

void MeshStorage::rebuildSolidVBOs(video::IVideoDriver *driver,
	std::list<std::pair<video::SMaterial, std::list<scene::IVertexBuffer *>>> &vbos)
{
	if (!layers_updated)
		return;

	layers_updated = false;

	// Remove old vbos
	for (auto &old_layer_vbos : vbos)
		for (auto old_vbo : old_layer_vbos.second)
			if (old_vbo)
				delete old_vbo;

	vbos.clear();

	// Add new vbos
	MutexAutoLock layers_lock(m_layers_mutex);

	for (auto new_layer : m_layers) {
		std::list<scene::IVertexBuffer *> new_vbos;
		for (auto &mesh_part : new_layer->merged_mesh) {
			scene::IVertexBuffer *new_vbo = driver->createVertexBuffer();
			new_vbo->formatBuffer();
			new_vbo->uploadVertexData(mesh_part.vertices.size(), mesh_part.vertices.data());
			new_vbo->uploadIndexData(mesh_part.indices.size(), mesh_part.indices.data());

			new_vbos.push_back(new_vbo);
		}

		vbos.emplace_back(new_layer->material, new_vbos);
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
