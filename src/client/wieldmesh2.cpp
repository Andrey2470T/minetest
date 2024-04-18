/*
Minetest
Copyright (C) 2010-2014 celeron55, Perttu Ahola <celeron55@gmail.com>

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

#include "wieldmesh.h"
#include "settings.h"
#include "shader.h"
#include "inventory.h"
#include "client.h"
#include "itemdef.h"
#include "nodedef.h"
#include "mesh.h"
#include "content_mapblock.h"
#include "mapblock_mesh.h"
#include "client/meshgen/collector.h"
#include "client/tile.h"
#include "log.h"
#include "util/numeric.h"
#include <map>
#include <IMeshManipulator.h>
#include "client/renderingengine.h"

#define WIELD_SCALE_FACTOR 30.0
#define WIELD_SCALE_FACTOR_EXTRUDED 40.0

#define MIN_EXTRUSION_MESH_RESOLUTION 16
#define MAX_EXTRUSION_MESH_RESOLUTION 512

scene::IMesh* MeshHUDBuilder::getCachedMesh(core::dimension2du dim)
{
	auto found = std::find_if(m_extrusion_meshes.begin(), m_extrusion_meshes.end(),
		[] (core::dimension2du c_dim, scene::IMesh* c_mesh) {
			return c_dim.Width == dim.Width && c_dim.Height == dim.Height;
	});

	if (!found) {
		scene::IMesh *new_mesh = createExtrusionMesh(dim);

		m_extrusion_meshes.emplace(std::make_pair(dim, new_mesh));
		return new_mesh;
	}

	return (*found).second;
}

scene::IMesh* MeshHUDBuilder::createExtrusionMesh(core::dimension2du dim)
{
	const f32 r = 0.5;

	scene::IMeshBuffer *buf = new scene::SMeshBuffer();
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

	for (int i = 0; i < dim.Width; ++i) {
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
	for (int i = 0; i < dim.Height; ++i) {
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


MeshHUDSceneNode::MeshHUDSceneNode(scene::ISceneManager *mgr, s32 id = -1, Client *client, MeshHUDBuilder *mh_builder,
		const HudElement *elem, bool lighting):
	scene::IMeshSceneNode(mgr->getRootSceneNode(), mgr, id), m_client(client), m_meshhud_builder(mh_builder),
	m_hud_elem(elem), m_material_type(video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF), m_lighting(lighting), m_player_light_color(0xFFFFFFFF)
{
	m_enable_shaders = g_settings->getBool("enable_shaders");
	m_anisotropic_filter = g_settings->getBool("anisotropic_filter");
	m_bilinear_filter = g_settings->getBool("bilinear_filter");
	m_trilinear_filter = g_settings->getBool("trilinear_filter");

	m_hud_mesh = m_meshhud_builder->getCubeMesh();

	setAutomaticCulling(scene::EAC_OFF);

	m_shadow = RenderingEngine::get_shadow_renderer();

	if (m_shadow)
		m_shadow->addNodeToShadowList(this);
}

void MeshHUDSceneNode::setMesh(scene::IMesh *mesh)
{
	if (!mesh)
		m_hud_mesh = cloneMesh(m_meshhud_builder->getCubeMesh());
	else {
		m_hud_mesh = mesh;

		if (m_enable_shaders)
			m_hud_mesh->setHardwareMappingHint(scene::EHM_STATIC);
		else
			m_hud_mesh->setHardwareMappingHint(scene::EHM_DYNAMIC);
	}

	forEachMaterial([this] (auto &mat) {
		mat.Lighting = m_lighting;
		// need to normalize normals when lighting is enabled (because of setScale())
		mat.NormalizeNormals = m_lighting;
	});
}

void MeshHUDSceneNode::OnRegisterSceneNode()
{
	if (!isVisible())
		return;

	scene::ISceneManager *mgr = getSceneManager();
	video::IVideoDriver *driver = RenderingEngine::get_video_driver();

	bool transparent_pass = false;
	bool solid_pass = false;

	for (u32 i = 0; i < m_hud_mesh->getMeshBufferCount(); i++) {
		const video::SMaterial &material = m_hud_mesh->getMeshBuffer(i)->getMaterial();

		if (driver->needsTransparentRenderPass(material))
			transparent_pass = true;
		else
			solid_pass = true;

		if (solid_pass && transparent_pass)
			break;
	}

	if (transparent_pass)
		mgr->registerNodeForRendering(this, scene::ESNRP_TRANSPARENT);

	if (solid_pass)
		mgr->registerNodeForRendering(this, scene::ESNRP_SOLID);

	ISceneNode::OnRegisterSceneNode();
}

void MeshHUDSceneNode::render()
{
	if (!isVisible())
		return;

	scene::ISceneManager *mgr = getSceneManager();
	video::IVideoDriver *driver = RenderingEngine::get_video_driver();

	driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);

	for (u32 i = 0; i < m_hud_mesh->getMeshBufferCount(); i++) {
		scene::IMeshBuffer *mb = m_hud_mesh->getMeshBuffer(i);

		driver->setMaterial(mb->getMaterial());
		driver->drawMeshBuffer(mb);
	}
}

v3f MeshHUDSceneNode::calculateViewPos(v2f screen_pos)
{
	v2f device_coords = (screen_pos - 0.5) * 2.0;

	f32 pos_to_v[4] = {device_coords.X, device_coords.Y, 0.0, 1.0};

	scene::ICameraSceneNode *cam = getSceneManager()->getActiveCamera();
	core::matrix4 proj = cam->getProjectionMatrix();
	core::matrix4 projinv;
	proj.getInverse(projinv);

	projinv.multiplyWith1x4Matrix(pos_to_v);

	return v3f(
		pos_to_v[0] / pos_to_v[3],
		pos_to_v[1] / pos_to_v[3],
		pos_to_v[2] / pos_to_v[3]
	);
}

void MeshHUDSceneNode::setColor(video::SColor light_color)
{
	u8 red = light_color.getRed();
	u8 green = light_color.getGreen();
	u8 blue = light_color.getBlue();

	const u32 mc = m_meshhud->getMeshBufferCount();

	if (mc > m_colors.size())
		m_colors.resize(mc);

	for (u32 j = 0; j < mc; j++) {
		video::SColor bc(m_base_color);
		m_colors[j].applyOverride(bc);
		video::SColor buffercolor(255,
			bc.getRed() * red / 255,
			bc.getGreen() * green / 255,
			bc.getBlue() * blue / 255);
		scene::IMeshBuffer *buf = mesh->getMeshBuffer(j);

		if (m_colors[j].needColorize(buffercolor)) {
			buf->setDirty(scene::EBT_VERTEX);
			if (m_enable_shaders)
				setMeshBufferColor(buf, buffercolor);
			else
				colorizeMeshBuffer(buf, &buffercolor);
		}
	}
}

void MeshHUDSceneNode::setNodeLightColor(video::SColor light_color)
{
	if (!m_lighting)
		return;

	if (m_enable_shaders)
		for (u32 i = 0; i < getMaterialCount(); ++i) {
			video::SMaterial &material = getMaterial(i);
			material.EmissiveColor = light_color;
		}
	else
		setColor(light_color);
}

void MeshHUDSceneNode::setCubeMesh(const ContentFeatures &f, v3f wield_scale, v3f rel_scale)
{
	m_hud_mesh->drop();

	scene::IMesh *mesh = cloneMesh(m_meshhud_builder->getCubeMesh());

	u32 mc = mesh->getMeshBufferCount();
	for (u32 i = 0; i < mc; ++i) {
		scene::IMeshBuffer *buf = mesh->getMeshBuffer(i);
		for (int layernum = 0; layernum < MAX_TILE_LAYERS; layernum++) {
			const TileLayer &layer = f.tiles[i].layers[layernum];
			if (layer.texture_id == 0)
				continue;
			if (layernum != 0) {
				scene::IMeshBuffer *copy = cloneMeshBuffer(buf);
				copy->getMaterial() = buf->getMaterial();
				mesh->addMeshBuffer(copy);
				copy->drop();
				buf = copy;
			}
			video::SMaterial &material = buf->getMaterial();
			if (layer.animation_frame_count > 1) {
				const FrameSpec &animation_frame = *(layer.frames)[0];
				material.setTexture(0, animation_frame.texture);
			} else {
				material.setTexture(0, layer.texture);
			}
			if (m_enable_shaders) {
				if (layer.normal_texture) {
					if (layer.animation_frame_count > 1) {
						const FrameSpec &animation_frame = *(layer.frames)[0];
						material.setTexture(1, animation_frame.normal_texture);
					} else
						material.setTexture(1, layer.normal_texture);
				}
				material.setTexture(2, layer.flags_texture);
			}
		}
	}

	postProcessNodeMesh(mesh, f, true, &m_material_type, &m_colors, true);
	postProcessMesh(mesh, false, true, true, true);
	setMesh(mesh);
	m_constant_scale = wield_scale * WIELD_SCALE_FACTOR;
	setScale(m_constant_scale * rel_scale);
}

void MeshHUDSceneNode::setExtrudedMesh(const std::string &imagename, const std::string &overlay_name,
		v3f wield_scale, u8 num_frames, v3f rel_scale)
{
	ITextureSource *tsrc = m_client->getTextureSource();
	video::ITexture *texture = tsrc->getTexture(imagename);
	if (!texture) {
		setMesh(nullptr);
		return;
	}
	video::ITexture *overlay_texture =
		overlay_name.empty() ? nullptr : tsrc->getTexture(overlay_name);

	core::dimension2du dim = texture->getSize();
	// Detect animation texture and pull off top frame instead of using entire thing
	if (num_frames > 1) {
		u32 frame_height = dim.Height / num_frames;
		dim = core::dimension2du(dim.Width, frame_height);
	}

	m_hud_mesh->drop();

	scene::IMesh *mesh = cloneMesh(m_meshhud_builder->getCachedMesh(dim));
	mesh->getMeshBuffer(0)->getMaterial().setTexture(0, texture);

	if (overlay_texture) {
		scene::IMeshBuffer *copy = cloneMeshBuffer(mesh->getMeshBuffer(0));
		copy->getMaterial().setTexture(0, overlay_texture);
		mesh->addMeshBuffer(copy);
		copy->drop();
	}

	postProcessMesh(mesh, true, true, dim.Width > 32, false);
	setMesh(mesh);
	m_constant_scale = wield_scale * WIELD_SCALE_FACTOR_EXTRUDED;
	setScale(m_constant_scale * rel_scale);
}

void MeshHUDSceneNode::setItem(const ItemStack &item, bool check_wield_image, v3f rel_scale)
{
	ITextureSource *tsrc = m_client->getTextureSource();
	IItemDefManager *idef = m_client->getItemDefManager();
	IShaderSource *shdrsrc = m_client->getShaderSource();
	const NodeDefManager *ndef = m_client->getNodeDefManager();
	const ItemDefinition &def = item.getDefinition(idef);
	const ContentFeatures &f = ndef->get(def.name);
	content_t id = ndef->getId(def.name);

	scene::SMesh *mesh = nullptr;

	if (m_enable_shaders) {
		u32 shader_id = shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL);
		m_material_type = shdrsrc->getShaderInfo(shader_id).material;
	}

	// Color-related
	m_colors.clear();
	m_base_color = idef->getItemstackColor(item, client);

	const std::string wield_image = item.getWieldImage(idef);
	const std::string wield_overlay = item.getWieldOverlay(idef);
	const v3f wield_scale = item.getWieldScale(idef);

	// If wield_image needs to be checked and is defined, it overrides everything else
	if (!wield_image.empty() && check_wield_image) {
		setExtrudedMesh(wield_image, wield_overlay, wield_scale, tsrc,
			1, rel_scale);
		m_colors.emplace_back();
		// overlay is white, if present
		m_colors.emplace_back(true, video::SColor(0xFFFFFFFF));
	}
	else if (def.type == ITEM_NODE) {
		// Select rendering method
		switch (f.drawtype) {
		case NDT_AIRLIKE:
			setExtrudedMesh("no_texture_airlike.png", "",
				v3f(1.0, 1.0, 1.0), tsrc, 1, rel_scale);
			break;
		case NDT_SIGNLIKE:
		case NDT_TORCHLIKE:
		case NDT_RAILLIKE:
		case NDT_PLANTLIKE:
		case NDT_FLOWINGLIQUID: {
			v3f wscale = wield_scale;
			if (f.drawtype == NDT_FLOWINGLIQUID)
				wscale.Z *= 0.1f;
			setExtrudedMesh(tsrc->getTextureName(f.tiles[0].layers[0].texture_id),
				tsrc->getTextureName(f.tiles[0].layers[1].texture_id),
				wscale, tsrc,
				f.tiles[0].layers[0].animation_frame_count, rel_scale);
			// Add color
			postProcessNodeMesh(mesh, f, true, &m_material_type, &m_colors, true);
			break;
		}
		case NDT_PLANTLIKE_ROOTED: {
			setExtruded(tsrc->getTextureName(f.special_tiles[0].layers[0].texture_id),
				"", wield_scale, tsrc,
				f.special_tiles[0].layers[0].animation_frame_count, rel_scale);
			// Add color
			postProcessNodeMesh(mesh, f, true, &m_material_type, &m_colors, true);
			break;
		}
		case NDT_NORMAL:
		case NDT_ALLFACES:
		case NDT_LIQUID:
			setCubeMesh(f, wield_scale, rel_scale);
			break;
		default: {
			// Render non-trivial drawtypes like the actual node
			MapNode n(id);
			if (def.place_param2)
				n.setParam2(*def.place_param2);

			mesh = createSpecialNodeMesh(client, n, &m_colors, f);
			changeToMesh(mesh);
			mesh->drop();
			m_constant_scale = wield_scale * WIELD_SCALE_FACTOR / (BS * f.visual_scale);
			m_meshnode->setScale(m_constant_scale * rel_scale);
			break;
		}
		}
	} else {
		const std::string inventory_image = item.getInventoryImage(idef);
		if (!inventory_image.empty()) {
			const std::string inventory_overlay = item.getInventoryOverlay(idef);
			setExtrudedMesh(inventory_image, inventory_overlay, def.wield_scale, tsrc, 1, rel_scale);
		} else {
			setExtrudedMesh("no_texture.png", "", def.wield_scale, tsrc, 1, rel_scale);
		}

		m_colors.emplace_back();
		// overlay is white, if present
		m_colors.emplace_back(true, video::SColor(0xFFFFFFFF));
	}
	setColor(video::SColor(0xFFFFFFFF));
}

void MeshHUDSceneNode::postProcessMesh(scene::IMesh *mesh, bool set_wrap_to_edge, bool backface_culling,
	bool enable_filter, bool use_mipmaps)
{
	forEachMaterial([this] (auto &mat) {
		if (set_wrap_to_edge) {
			mat.TextureLayers[0].TextureWrapU = video::ETC_CLAMP_TO_EDGE;
			mat.TextureLayers[0].TextureWrapV = video::ETC_CLAMP_TO_EDGE;
		}
		mat.MaterialType = m_material_type;
		mat.MaterialTypeParam = 0.5f;
		mat.BackfaceCulling = backface_culling;

		mat.forEachTexture([this] (auto &tex) {
			setMaterialFilters(tex, enable_filter && m_bilinear_filter, enable_filter && m_trilinear_filter,
				m_anisotropic_filter);
		});

		mat.UseMipMaps = use_mipmaps;
	});
}

void MeshHUDSceneNode::postProcessNodeMesh(scene::IMesh *mesh, const ContentFeatures &f, bool set_material,
	const video::E_MATERIAL_TYPE *mattype, std::vector<ItemPartColor> *colors, bool apply_scale)
	colors->clear();

	switch (f.drawtype) {
	case NDT_SIGNLIKE:
	case NDT_TORCHLIKE:
	case NDT_RAILLIKE:
	case NDT_PLANTLIKE:
	case NDT_FLOWINGLIQUID: {
		const TileLayer &l0 = f.tiles[0].layers[0];
		colors->emplace_back(l0.has_color, l0.color);
		const TileLayer &l1 = f.tiles[0].layers[1];
		colors->emplace_back(l1.has_color, l1.color);
		break;
	}
	case NDT_PLANTLIKE_ROOTED: {
		const TileLayer &l0 = f.special_tiles[0].layers[0];
		colors->emplace_back(l0.has_color, l0.color);
		break;
	}
	case NDT_NORMAL:
	case NDT_ALLFACES:
	case NDT_LIQUID: {
		for (int layernum = 0; layernum < MAX_TILE_LAYERS; layernum++)
			for (u32 i = 0; i < 6; i++) {
				const TileLayer &layer = f.tiles[i].layers[layernum];

				if (layer.texture_id == 0)
					continue;

				colors->emplace_back(layer.has_color, layer.color);

				u32 mb_i = (layernum*6) + i;
				scene::IMeshBuffer *buf = mesh->getMeshBuffer(mb_i);
				video::SMaterial &material = buf->getMaterial();

				if (set_material)
					layer->applyMaterialOptions(material);
				if (mattype)
					material.MaterialType = *mattype;
				if (apply_scale && f.tiles[i].world_aligned) {
					u32 n = buf->getVertexCount();
					for (u32 k = 0; k != n; ++k)
						buf->getTCoords(k) /= layer->scale;
				}
			}
		break;
	}}
}

void MeshHUDSceneNode::step(f32 dtime)
{
	video::SColor cur_color = m_client->getEnv().getLocalPlayer()->light_color;

	if (m_player_light_color != cur_color) {
		m_player_light_color = cur_color;
		setNodeLightColor(m_player_light_color);
	}

	if (!m_hud_elem)
		return;

	v3f cur_pos = getPosition();
	v3f new_pos = calculateViewPos(m_hud_elem->pos);
	new_pos.Z = m_hud_elem->z_offset;

	if (cur_pos != new_pos)
		setPosition(new_pos);

	if (getRotation() != m_hud_elem->rotation)
		setRotation(m_hud_elem->rotation);

	v3f new_scale(std::fabs(m_hud_elem->scale.X));

	switch (m_hud_elem->dir) {
		case 0:
			new_scale.X *= 1;
			break;
		case 1:
			new_scale.X *= -1;
			break;
		case 2:
			new_scale.Y *= -1;
			break;
		case 3:
			new_scale.Y *= 1;
			break;
		default:
			break;
	}
	if (m_last_text != m_hud_elem->text) {
		scene::IMesh *mesh = m_client->getMesh(m_hud_elem->text)

		if (!mesh)
			setItem(ItemStack(m_hud_elem->text, 1, 0, m_client->idef()), m_client, true, new_scale);
		else {
			if (m_enable_shaders) {
				IShaderSource *shdrsrc = m_client->getShaderSource();
				m_material_type = shdsrc->getShaderInfo(shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL)).material;
			}
			m_hud_elem->textures.reserve(mesh->getMeshBufferCount());

			for (int i = 0; i < mesh->getMeshBufferCount(); i++) {
				m_colors.emplace_back(true, video::SColor(0xFFFFFFFF));

				const video::SMaterial &material = mesh->getMeshBuffer(i)->getMaterial();
				material.setTexture(0, m_client->getTextureSource()->getTextureForMesh(m_hud_elem->textures[i]))
			}

			postProcessMesh(mesh, false, false, true, false);
			setMesh(mesh);
			m_constant_scale = v3f(WIELD_SCALE_FACTOR);
			setScale(m_constant_scale * new_scale);
		}
		m_last_relative_scale = new_scale;
	}

	if (m_last_relative_scale != new_scale) {
		setScale(m_constant_scale * new_scale);
		m_last_relative_scale = new_scale;
	}
}
