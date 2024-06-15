/*
Minetest
Copyright (C) 2024 Andrey2470T, AndreyT <andreyt2203@gmail.com>

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

#include "client/mesh_cache.h"
#include "client/client.h"
#include "client/mapblock_mesh.h"
#include "client/content_mapblock.h"
#include "client/meshgen/collector.h"
#include "nodedef.h"

scene::SMesh* MeshBuilder::createExtrusionMesh(core::dimension2du dim)
{
	const f32 r = 0.5;

	scene::SMeshBuffer *buf = new scene::SMeshBuffer();
	video::SColor c(255,255,255,255);
	v3f scale(1.0, 1.0, 0.1);

	// Front and back
	{
		video::S3DVertex vertices[8] = {
			// z-
			video::S3DVertex(-r,+r,-r, 0,0,-1, c, 0,0),
			video::S3DVertex(+r,+r,-r, 0,0,-1, c, 1,0),
			video::S3DVertex(+r,-r,-r, 0,0,-1, c, 1,1),
			video::S3DVertex(-r,-r,-r, 0,0,-1, c, 0,1),
			// z+
			video::S3DVertex(-r,+r,+r, 0,0,+1, c, 0,0),
			video::S3DVertex(-r,-r,+r, 0,0,+1, c, 0,1),
			video::S3DVertex(+r,-r,+r, 0,0,+1, c, 1,1),
			video::S3DVertex(+r,+r,+r, 0,0,+1, c, 1,0),
		};
		u16 indices[12] = {0,1,2,2,3,0,4,5,6,6,7,4};
		buf->append(vertices, 8, indices, 12);
	}

	f32 pixelsize_x = 1 / (f32) dim.Width;
	f32 pixelsize_y = 1 / (f32) dim.Height;

	for (u32 i = 0; i < dim.Width; ++i) {
		f32 pixelpos_x = i * pixelsize_x - 0.5;
		f32 x0 = pixelpos_x;
		f32 x1 = pixelpos_x + pixelsize_x;
		f32 tex0 = (i + 0.1) * pixelsize_x;
		f32 tex1 = (i + 0.9) * pixelsize_x;
		video::S3DVertex vertices[8] = {
			// x-
			video::S3DVertex(x0,-r,-r, -1,0,0, c, tex0,1),
			video::S3DVertex(x0,-r,+r, -1,0,0, c, tex1,1),
			video::S3DVertex(x0,+r,+r, -1,0,0, c, tex1,0),
			video::S3DVertex(x0,+r,-r, -1,0,0, c, tex0,0),
			// x+
			video::S3DVertex(x1,-r,-r, +1,0,0, c, tex0,1),
			video::S3DVertex(x1,+r,-r, +1,0,0, c, tex0,0),
			video::S3DVertex(x1,+r,+r, +1,0,0, c, tex1,0),
			video::S3DVertex(x1,-r,+r, +1,0,0, c, tex1,1),
		};
		u16 indices[12] = {0,1,2,2,3,0,4,5,6,6,7,4};
		buf->append(vertices, 8, indices, 12);
	}
	for (u32 i = 0; i < dim.Height; ++i) {
		f32 pixelpos_y = i * pixelsize_y - 0.5;
		f32 y0 = -pixelpos_y - pixelsize_y;
		f32 y1 = -pixelpos_y;
		f32 tex0 = (i + 0.1) * pixelsize_y;
		f32 tex1 = (i + 0.9) * pixelsize_y;
		video::S3DVertex vertices[8] = {
			// y-
			video::S3DVertex(-r,y0,-r, 0,-1,0, c, 0,tex0),
			video::S3DVertex(+r,y0,-r, 0,-1,0, c, 1,tex0),
			video::S3DVertex(+r,y0,+r, 0,-1,0, c, 1,tex1),
			video::S3DVertex(-r,y0,+r, 0,-1,0, c, 0,tex1),
			// y+
			video::S3DVertex(-r,y1,-r, 0,+1,0, c, 0,tex0),
			video::S3DVertex(-r,y1,+r, 0,+1,0, c, 0,tex1),
			video::S3DVertex(+r,y1,+r, 0,+1,0, c, 1,tex1),
			video::S3DVertex(+r,y1,-r, 0,+1,0, c, 1,tex0),
		};
		u16 indices[12] = {0,1,2,2,3,0,4,5,6,6,7,4};
		buf->append(vertices, 8, indices, 12);
	}

	// Create mesh object
	scene::SMesh *mesh = new scene::SMesh();
	mesh->addMeshBuffer(buf);
	buf->drop();
	scaleMesh(mesh, scale);  // also recalculates bounding box

	return mesh;
}

scene::SMesh* MeshBuilder::createSpecialNodeMesh(Client *client, content_t id, const std::optional<u8> &place_param2)
{
	auto nodedef = client->ndef();

	MeshMakeData mesh_make_data(nodedef, 1, false);
	MeshCollector collector(v3f(0.0f * BS), v3f());
	mesh_make_data.setSmoothLighting(false);
	MapblockMeshGenerator gen(&mesh_make_data, &collector,
		client->getSceneManager()->getMeshManipulator());

	MapNode n(id);

	const ContentFeatures &f = nodedef->get(n);

	if (place_param2)
		n.setParam2(*place_param2);

	if (n.getParam2()) {
		// keep it
	} else if (f.param_type_2 == CPT2_WALLMOUNTED ||
			f.param_type_2 == CPT2_COLORED_WALLMOUNTED) {
		if (f.drawtype == NDT_TORCHLIKE ||
				f.drawtype == NDT_SIGNLIKE ||
				f.drawtype == NDT_NODEBOX ||
				f.drawtype == NDT_MESH) {
			n.setParam2(4);
		}
	} else if (f.drawtype == NDT_SIGNLIKE || f.drawtype == NDT_TORCHLIKE) {
		n.setParam2(1);
	}
	gen.renderSingle(n.getContent(), n.getParam2());

	scene::SMesh *mesh = new scene::SMesh();
	for (auto &prebuffers : collector.prebuffers)
		for (PreMeshBuffer &p : prebuffers) {
			if (p.layer.material_flags & MATERIAL_FLAG_ANIMATION) {
				const FrameSpec &frame = (*p.layer.frames)[0];
				p.layer.texture = frame.texture;
				p.layer.normal_texture = frame.normal_texture;
			}
			for (video::S3DVertex &v : p.vertices) {
				v.Color.setAlpha(255);
			}
			scene::SMeshBuffer *buf = new scene::SMeshBuffer();
			buf->Material.setTexture(0, p.layer.texture);
			p.layer.applyMaterialOptions(buf->Material);
			mesh->addMeshBuffer(buf);
			buf->append(&p.vertices[0], p.vertices.size(),
					&p.indices[0], p.indices.size());
			buf->drop();
		}
	return mesh;
}


void MeshCacheManager::getSpecialNodeMesh(ItemMesh *imesh, Client *client,
	content_t id, const std::optional<u8> &place_param2)
{
	imesh->mesh = MeshBuilder::createSpecialNodeMesh(client, id, place_param2);

	auto ndef = client->getNodeDefManager();
	const ContentFeatures &f = ndef->get(id);

	for (int layer = 0; layer < MAX_TILE_LAYERS; layer++)
		for (const auto &tile : f.tiles) {
			const TileLayer &l = tile.layers[layer];
			imesh->buffer_colors.push_back(l.has_color ? l.color : imesh->base_color);
		}
}

void MeshCacheManager::postProcessNodeMesh(ItemMesh *imesh, const ContentFeatures &f, bool use_shaders,
		bool set_material, bool apply_scale)
{
	scene::SMesh *smesh = dynamic_cast<scene::SMesh*>(imesh->mesh);

	for (u32 i = 0; i < smesh->getMeshBufferCount(); ++i) {

		scene::IMeshBuffer *buf = smesh->getMeshBuffer(i);

		for (int layernum = 0; layernum < MAX_TILE_LAYERS; layernum++) {

			const TileLayer &layer = f.tiles[i].layers[layernum];
			if (layer.texture_id == 0)
				continue;

			if (layernum != 0) {
				scene::IMeshBuffer *copy = cloneMeshBuffer(buf);
				copy->getMaterial() = buf->getMaterial();
				smesh->addMeshBuffer(copy);
				copy->drop();
				buf = copy;
			}

			imesh->buffer_colors.push_back(layer.has_color ? layer.color : imesh->base_color);

			video::SMaterial &material = buf->getMaterial();
			if (set_material)
				layer.applyMaterialOptions(material);

			if (layer.animation_frame_count > 1) {
				const FrameSpec &animation_frame = (*layer.frames)[0];
				material.setTexture(0, animation_frame.texture);
			} else {
				material.setTexture(0, layer.texture);
			}
			if (use_shaders) {
				if (layer.normal_texture) {
					if (layer.animation_frame_count > 1) {
						const FrameSpec &animation_frame = (*layer.frames)[0];
						material.setTexture(1, animation_frame.normal_texture);
					} else
						material.setTexture(1, layer.normal_texture);
				}
				material.setTexture(2, layer.flags_texture);
			}

			if (apply_scale && f.tiles[i].world_aligned) {
				u32 n = buf->getVertexCount();
				for (u32 k = 0; k != n; ++k)
					buf->getTCoords(k) /= layer.scale;
			}
		}
	}
}
