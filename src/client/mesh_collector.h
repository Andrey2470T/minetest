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
#include <array>
#include "client/tile.h"

class Client;

// Map of indices to layers and indices of arrays pairs inside that layer
using MeshRef = std::unordered_map<u32, u32>;

class TriangleComparer
{
public:
	TriangleComparer(const v3f &camera_pos) : m_camera_pos(camera_pos) {}

	bool operator() (const v3f &pos1, const v3f &pos2) const
	{
		auto dist1 = pos1.getDistanceFromSQ(m_camera_pos);
		auto dist2 = pos2.getDistanceFromSQ(m_camera_pos);

		return dist1 > dist2;
	}
private:
	v3f m_camera_pos;
};

// represents a triangle as indexes into the vertex buffer in SMeshBuffer
struct MeshTriangle
{
	u32 layer_ref;
	u32 arrays_ref;

	std::array<u32, 3> indices;

    MeshTriangle(u32 _layer_ref, u32 _arrays_ref, std::array<u32, 3> _indices)
        : layer_ref(_layer_ref), arrays_ref(_arrays_ref), indices(_indices)
    {}
};

// Structure saving info about the currently generated mesh part (mapblock or wieldmesh)
// for which buffers or materials are being added
struct MeshInfo
{
	// bounding sphere radius and center
	f32 bounding_radius_sq = 0.0f;
	v3f center_pos;
	v3f offset;
	v3f translation;

    std::map<v3f, MeshTriangle, TriangleComparer> transparent_triangles;
	// Map specific layers to specific vertex arrays which the given mesh has
	MeshRef layers_to_arrays_map;

    MeshInfo(f32 _bounding_radius_sq, v3f _center_pos, v3f _offset, v3f _translation)
        : bounding_radius_sq(_bounding_radius_sq), center_pos(_center_pos), offset(_offset),
          translation(_translation), transparent_triangles{TriangleComparer(v3f(0.0f))}
    {}
};

struct MeshPart
{
	std::vector<video::S3DVertex> vertices;

	std::vector<u32> indices;
};

class MeshCollector
{
public:
    virtual void addTileMesh(const TileSpec &tile,
		const video::S3DVertex *vertices, u32 numVertices,
		const u16 *indices, u32 numIndices, v3f pos = v3f(0.0f),
		video::SColor clr = video::SColor(), u8 light_source = 0) = 0;
};

class MapblockMeshCollector final : public MeshCollector
{
public:
	Client *client;
	MeshInfo *info;

	std::vector<std::pair<video::SMaterial, MeshPart>> prelayers;

    MapblockMeshCollector(Client *_client, MeshInfo *_mesh_info)
		: client(_client), info(_mesh_info) {}

    void addTileMesh(const TileSpec &tile,
		const video::S3DVertex *vertices, u32 numVertices,
		const u16 *indices, u32 numIndices, v3f pos = v3f(0.0f),
		video::SColor clr = video::SColor(), u8 light_source = 0) override;
};

// The code adopted from the deleted client/meshgen
struct WieldPreMeshBuffer
{
	TileLayer layer;
	std::vector<u16> indices;
	std::vector<video::S3DVertex> vertices;

	WieldPreMeshBuffer() = default;
	explicit WieldPreMeshBuffer(const TileLayer &layer) : layer(layer) {}
};

class WieldMeshCollector final : public MeshCollector
{
public:
	std::vector<WieldPreMeshBuffer> prebuffers;
	// bounding sphere radius and center
	f32 m_bounding_radius_sq = 0.0f;
	v3f m_center_pos;
	v3f offset;

	// center_pos: pos to use for bounding-sphere, in BS-space
	// offset: offset added to vertices
    WieldMeshCollector(const v3f center_pos, v3f offset = v3f()) : m_center_pos(center_pos), offset(offset) {}

	void addTileMesh(const TileSpec &tile,
		const video::S3DVertex *vertices, u32 numVertices,
		const u16 *indices, u32 numIndices, v3f pos = v3f(0.0f),
		video::SColor clr = video::SColor(), u8 light_source = 0) override;

private:
	WieldPreMeshBuffer &findBuffer(const TileLayer &layer, u32 numVertices);
};
