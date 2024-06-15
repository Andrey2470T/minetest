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

#include "hudmesh.h"
#include "settings.h"
#include "shader.h"
#include "inventory.h"
#include "client.h"
#include "itemdef.h"
#include "nodedef.h"
#include "content_mapblock.h"
#include "mapblock_mesh.h"
#include "client/meshgen/collector.h"
#include "client/tile.h"
#include "log.h"
#include "util/numeric.h"
#include <map>
#include <IMeshManipulator.h>
#include "client/renderingengine.h"
#include <ICameraSceneNode.h>
#include "client/localplayer.h"
#include "client/camera.h"
#include "hud.h"

#define WIELD_SCALE_FACTOR 30.0
#define WIELD_SCALE_FACTOR_EXTRUDED 40.0


/*
 * HUDMeshCacheManager class
 */

HUDMeshCacheManager::HUDMeshCacheManager()
{
	m_enable_shaders = g_settings->getBool("enable_shaders");
	m_anisotropic_filter = g_settings->getBool("anisotropic_filter");
	m_bilinear_filter = g_settings->getBool("bilinear_filter");
	m_trilinear_filter = g_settings->getBool("trilinear_filter");
}

ItemMesh* HUDMeshCacheManager::getOrCreateMesh(const ItemStack &item, Client *client)
{
	//infostream << "getOrCreateMesh(): 1" << std::endl;
	ITextureSource *tsrc = client->getTextureSource();
	IItemDefManager *idef = client->getItemDefManager();
	IShaderSource *shdrsrc = client->getShaderSource();
	const NodeDefManager *ndef = client->getNodeDefManager();

	const ItemDefinition &def = item.getDefinition(idef);
	const ContentFeatures &f = ndef->get(def.name);

	std::string wield_image = item.getWieldImage(idef);
	std::string wield_overlay = item.getWieldOverlay(idef);
	std::string inventory_image = item.getInventoryImage(idef);
	std::string inventory_overlay = item.getInventoryOverlay(idef);
	v3f wield_scale = item.getWieldScale(idef);

	std::string cache_key = def.name;
	//infostream << "getOrCreateMesh(): 2" << std::endl;
	if (!wield_image.empty()) {
		cache_key += "/" + wield_image;

		if (!wield_overlay.empty())
			cache_key += ":" + wield_overlay;
	}
	else if (!inventory_image.empty()) {
		cache_key += "/" + inventory_image;

		if (!inventory_overlay.empty())
			cache_key += ":" + inventory_overlay;
	}
	//infostream << "getOrCreateMesh(): name: " << cache_key << std::endl;
	auto it = m_cache.find(cache_key);
	if (it != m_cache.end()) {
		//infostream << "getOrCreateMesh(): 3.1" << std::endl;
		return it->second.get();
	}
	//infostream << "getOrCreateMesh(): 4" << std::endl;
	// Create new ItemMesh
	ItemMesh *imesh = new ItemMesh();

	imesh->base_color = idef->getItemstackColor(item, client);

	f32 scale_factor = WIELD_SCALE_FACTOR_EXTRUDED;
	//infostream << "getOrCreateMesh(): 5" << std::endl;
	// If wield_image needs to be checked and is defined, it overrides everything else
	if (!wield_image.empty()) {// && check_wield_image)
		//infostream << "getOrCreateMesh(): 6" << std::endl;
		getExtrusionMesh(imesh, tsrc, wield_image, wield_overlay, imesh->base_color);
	}
	else if (def.type == ITEM_NODE)
		// Select rendering method
		switch (f.drawtype) {
		case NDT_AIRLIKE:
			//infostream << "getOrCreateMesh(): 7" << std::endl;
			wield_scale.set(1.0f, 1.0f, 1.0f);
			getExtrusionMesh(imesh, tsrc, "no_texture_airlike.png", "");
			break;
		case NDT_SIGNLIKE:
		case NDT_TORCHLIKE:
		case NDT_RAILLIKE:
		case NDT_PLANTLIKE:
		case NDT_FLOWINGLIQUID: {
			//infostream << "getOrCreateMesh(): 8" << std::endl;
			//v3f wscale = wield_scale;
			if (f.drawtype == NDT_FLOWINGLIQUID)
				wield_scale.Z *= 0.1f;
			const TileLayer &l0 = f.tiles[0].layers[0];
			const TileLayer &l1 = f.tiles[0].layers[1];
			getExtrusionMesh(imesh, tsrc, tsrc->getTextureName(l0.texture_id),
				tsrc->getTextureName(l1.texture_id), l0.has_color ? l0.color : imesh->base_color,
				l1.has_color ? l1.color : imesh->base_color, l0.animation_frame_count);
			break;
		}
		case NDT_PLANTLIKE_ROOTED: {
			//infostream << "getOrCreateMesh(): 9" << std::endl;
			const TileLayer &l0 = f.special_tiles[0].layers[0];
			getExtrusionMesh(imesh, tsrc, tsrc->getTextureName(l0.texture_id),
				"", l0.has_color ? l0.color : imesh->base_color, video::SColor(0xFFFFFFFF),
				l0.animation_frame_count);
			break;
		}
		case NDT_NORMAL:
		case NDT_ALLFACES:
		case NDT_LIQUID:
			//infostream << "getOrCreateMesh(): 10" << std::endl;
			getCubeMesh(imesh, f);
			scale_factor = WIELD_SCALE_FACTOR;
			break;
		default:
			//infostream << "getOrCreateMesh(): 11" << std::endl;
			getSpecialNodeMesh(imesh, client, ndef->getId(def.name), def.place_param2);
			scale_factor = WIELD_SCALE_FACTOR / (BS * f.visual_scale);
			break;
		}
	else if (!inventory_image.empty()) {
		//infostream << "getOrCreateMesh(): 12" << std::endl;
		getExtrusionMesh(imesh, tsrc, inventory_image, inventory_overlay, imesh->base_color);
	}
	else {
		//infostream << "getOrCreateMesh(): 13" << std::endl;
		getExtrusionMesh(imesh, tsrc, "no_texture.png", "", imesh->base_color);
	}

	for (u32 i = 0; i < imesh->mesh->getMeshBufferCount(); ++i) {
		//infostream << "getOrCreateMesh(): 14" << std::endl;
		video::SMaterial &material = imesh->mesh->getMeshBuffer(i)->getMaterial();

		if (m_enable_shaders) {
			u32 shader_id = shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL);
			material.MaterialType = shdrsrc->getShaderInfo(shader_id).material;
		}
		else
			material.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;

		material.MaterialTypeParam = 0.5f;
		material.BackfaceCulling = f.needsBackfaceCulling();
		material.forEachTexture([this] (auto &tex) {
			setMaterialFilters(tex, m_bilinear_filter, m_trilinear_filter,
					m_anisotropic_filter);
		});
		//infostream << "getOrCreateMesh(): 15" << std::endl;
	}

	if (m_enable_shaders)
		imesh->mesh->setHardwareMappingHint(scene::EHM_STATIC);
	else
		imesh->mesh->setHardwareMappingHint(scene::EHM_DYNAMIC);

	imesh->constant_scale = wield_scale * scale_factor;
	//scaleMesh(imesh->mesh, imesh->constant_scale);

	m_cache[cache_key] = std::unique_ptr<ItemMesh>(imesh);

	//infostream << "getOrCreateMesh(): 16" << std::endl;
	return imesh;
}

void HUDMeshCacheManager::getFileMesh(ItemMesh *imesh, std::string text,
		std::vector<std::string> textures, Client *client)
{
	imesh->mesh = client->getMesh(text);

	if (!(imesh->mesh))
		return;
	textures.resize(imesh->mesh->getMeshBufferCount());

	ITextureSource *tsrc = client->getTextureSource();
	IShaderSource *shdrsrc = client->getShaderSource();
	for (u32 i = 0; i < imesh->mesh->getMeshBufferCount(); i++) {
		imesh->buffer_colors.emplace_back(video::SColor(0xFFFFFFFF));
		video::SMaterial &material = imesh->mesh->getMeshBuffer(i)->getMaterial();
		material.setTexture(0, tsrc->getTextureForMesh(textures[i]));
		if (m_enable_shaders) {
			u32 shader_id = shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL);
			material.MaterialType = shdrsrc->getShaderInfo(shader_id).material;
		}
		else
			material.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;
		material.MaterialTypeParam = 0.5f;
		material.BackfaceCulling = false;
		material.forEachTexture([this] (auto &tex) {
			setMaterialFilters(tex, m_bilinear_filter,
				m_trilinear_filter, m_anisotropic_filter);
		});
	}
	if (m_enable_shaders)
		imesh->mesh->setHardwareMappingHint(scene::EHM_STATIC);
	else
		imesh->mesh->setHardwareMappingHint(scene::EHM_DYNAMIC);

	imesh->constant_scale = v3f(WIELD_SCALE_FACTOR);
}

void HUDMeshCacheManager::getCubeMesh(ItemMesh *imesh, const ContentFeatures &f)
{
	imesh->mesh = MeshBuilder::createCubicMesh();

	postProcessNodeMesh(imesh, f, false, true, true);
}

void HUDMeshCacheManager::getExtrusionMesh(ItemMesh *imesh, ITextureSource *tsrc, std::string image,
		std::string overlay_image, video::SColor color,
		video::SColor overlay_color, u8 num_frames)
{
	video::ITexture *texture = tsrc->getTextureForMesh(image);

	if (!texture)
		return;

	core::dimension2du dim = texture->getSize();
	// Detect animation texture and pull off top frame instead of using entire thing
	if (num_frames > 1) {
		u32 frame_height = dim.Height / num_frames;
		dim = core::dimension2du(dim.Width, frame_height);
	}

	scene::SMesh *mesh = MeshBuilder::createExtrusionMesh(dim);
	mesh->getMeshBuffer(0)->getMaterial().setTexture(0, texture);

	if (!overlay_image.empty()) {
		scene::IMeshBuffer *clone_mbuf = cloneMeshBuffer(mesh->getMeshBuffer(0));
		clone_mbuf->getMaterial().setTexture(0,tsrc->getTexture(overlay_image));
		mesh->addMeshBuffer(clone_mbuf);
		clone_mbuf->drop();
	}

	for (u32 layer = 0; layer < mesh->getMeshBufferCount(); layer++) {
		video::SMaterial &material = mesh->getMeshBuffer(layer)->getMaterial();

		material.TextureLayers[0].TextureWrapU = video::ETC_CLAMP_TO_EDGE;
		material.TextureLayers[0].TextureWrapV = video::ETC_CLAMP_TO_EDGE;

		// Enable bi/trilinear filtering only for high resolution textures
		bool bilinear_filter = dim.Width > 32 && m_bilinear_filter;
		bool trilinear_filter = dim.Width > 32 && m_trilinear_filter;
		material.forEachTexture([=] (auto &tex) {
			setMaterialFilters(tex, bilinear_filter, trilinear_filter,
					m_anisotropic_filter);
		});
		// mipmaps cause "thin black line" artifacts
		material.UseMipMaps = false;
		if (m_enable_shaders)
			material.setTexture(2, tsrc->getShaderFlagsTexture(false));
	}

	imesh->mesh = mesh;
	imesh->buffer_colors.push_back(color);
	imesh->buffer_colors.push_back(overlay_color);
}


/*
 * MeshHUDSceneNode class
 */

MeshHUDSceneNode::MeshHUDSceneNode(scene::ISceneManager *mgr, Client *client, bool lighting, HudElement *elem):
	scene::IMeshSceneNode(mgr->getRootSceneNode(), mgr, -1), m_client(client),
	m_lighting(lighting), m_hud_elem(elem)
{
	m_enable_shaders = g_settings->getBool("enable_shaders");

	m_hud_mesh = m_client->getEnv().getMeshManager()->getOrCreateMesh(ItemStack(), m_client);
	m_mesh_cached = true;
	setScale(m_hud_mesh->constant_scale);

	setAutomaticCulling(scene::EAC_OFF);

	v2u32 size = RenderingEngine::getWindowSize();
	m_original_rect = core::recti(0, 0, size.X, size.Y);
	m_rect = m_original_rect;

	m_shadow = RenderingEngine::get_shadow_renderer();

	if (m_shadow)
		m_shadow->addNodeToShadowList(this);
}

void MeshHUDSceneNode::OnRegisterSceneNode()
{
	if (!isVisible())
		return;

	scene::ISceneManager *mgr = getSceneManager();
	video::IVideoDriver *driver = RenderingEngine::get_video_driver();

	bool transparent_pass = false;
	bool solid_pass = false;

	scene::IMesh *mesh = m_hud_mesh->mesh;

	if (!mesh)
		return;
	for (u32 i = 0; i < mesh->getMeshBufferCount(); i++) {
		const video::SMaterial &material = mesh->getMeshBuffer(i)->getMaterial();

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

	IMeshSceneNode::OnRegisterSceneNode();
}

void MeshHUDSceneNode::render()
{
	if (!isVisible())
		return;

	video::IVideoDriver *driver = RenderingEngine::get_video_driver();

	driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);

	scene::IMesh *mesh = m_hud_mesh->mesh;

	if (!mesh)
		return;
	for (u32 i = 0; i < mesh->getMeshBufferCount(); i++) {
		scene::IMeshBuffer *mb = mesh->getMeshBuffer(i);

		driver->setMaterial(mb->getMaterial());
		driver->drawMeshBuffer(mb);
	}
}

void MeshHUDSceneNode::setMeshColor(video::SColor light_color)
{
	scene::IMesh *mesh = m_hud_mesh->mesh;

	if (!mesh)
		return;

	IShaderSource *shdrsrc = m_client->getShaderSource();

	// Apply the light color to the buffers only if the vertex lighting is enabled
	video::SColor buffer_light_color = (!m_lighting || m_enable_shaders) ? 0xFFFFFFFF : light_color;

	for (u32 i = 0; i < mesh->getMeshBufferCount(); i++) {
		video::SColor bc(m_hud_mesh->buffer_colors[i]);

		bc.setRed(bc.getRed() * buffer_light_color.getRed() / 255);
		bc.setGreen(bc.getGreen() * buffer_light_color.getGreen() / 255);
		bc.setBlue(bc.getBlue() * buffer_light_color.getBlue() / 255);
		bc.setAlpha(255);

		scene::IMeshBuffer *buf = mesh->getMeshBuffer(i);
		buf->setDirty(scene::EBT_VERTEX);

		video::SMaterial &mat = buf->getMaterial();

		// No lighting at all. Only the hardware color is applied.
		if (!m_lighting) {
			mat.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;

			setMeshBufferColor(buf, bc);
		}
		else {
			// Shader lighting
			if (m_enable_shaders) {
				setMeshBufferColor(buf, bc);

				u32 shader_id = shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL);
				mat.MaterialType = shdrsrc->getShaderInfo(shader_id).material;
				mat.EmissiveColor = light_color;
			}
			// Vertex lighting
			else {
				colorizeMeshBuffer(buf, &bc);

				mat.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;
				mat.EmissiveColor = video::SColor(0xFFFFFFFF);
			}
		}
	}
}

void MeshHUDSceneNode::updateMesh(std::string text, std::vector<std::string> textures)
{
	//if (m_last_text == text)
	//	return;

	//m_last_text = text;

	if (!(m_hud_elem->change_flags & u8(MeshHUDStatsChange::TEXT)))
		return;

	// Delete the mesh if it was non-cached before
	if (!m_mesh_cached && m_hud_mesh)
		delete m_hud_mesh;

	HUDMeshCacheManager *mesh_mgr = m_client->getEnv().getMeshManager();
	// Assume that the text string contains the itemstack name that is necessary to display
	if (text.find(':') != text.npos) {
		m_hud_mesh = mesh_mgr->getOrCreateMesh(ItemStack(text, 1, 0, m_client->idef()), m_client);
		m_mesh_cached = true;
	}
	// Else assume that it is a mesh file name
	else {
		m_hud_mesh = new ItemMesh();
		mesh_mgr->getFileMesh(m_hud_mesh, text, textures, m_client);
		m_mesh_cached = false;
	}

	m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::TEXT);

	if (m_hud_elem->change_flags & u8(MeshHUDStatsChange::TEXS))
		m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::TEXS);
}

void MeshHUDSceneNode::updateRect()
{
	if (!(m_hud_elem->change_flags & u8(MeshHUDStatsChange::POS)))
		return;

	//v2f uv_pos = m_scene_pos / 2.0f + 0.5f;

	//if (uv_pos == m_hud_elem->pos)
	//	return;

	v2f screen_pos = (m_hud_elem->pos - 0.5f) * 2.0f;

	v2s32 new_pos(s32(screen_pos.X*m_original_rect.getWidth()/2.0f),
		s32(screen_pos.Y*m_original_rect.getHeight()/2.0f));

	m_rect = m_original_rect + new_pos;

	m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::POS);
}

void MeshHUDSceneNode::updateScale()
{
	if (m_hud_elem->change_flags & u8(MeshHUDStatsChange::DIR)) {
		switch (m_hud_elem->dir) {
		case 0:
			m_last_dir.X *= 1;
			break;
		case 1:
			m_last_dir.X *= -1;
			break;
		case 2:
			m_last_dir.Y *= -1;
			break;
		case 3:
			m_last_dir.Y *= 1;
			break;
		default:
			break;
		}

		m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::DIR);
	}

	if (m_hud_elem->change_flags & u8(MeshHUDStatsChange::SCALE)) {
		m_last_relative_scale = v3f(std::fabs(m_hud_elem->scale.X));

		m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::SCALE);
	}

	setScale(m_hud_mesh->constant_scale * m_last_relative_scale * m_last_dir);
}

void MeshHUDSceneNode::updateTextures()
{
	if (!(m_hud_elem->change_flags & u8(MeshHUDStatsChange::TEXS)))
		return;

	ITextureSource *tsrc = m_client->getTextureSource();

	for (u32 i = 0; i < m_hud_elem->textures.size(); i++) {
		video::SMaterial &mat = m_hud_mesh->mesh->getMeshBuffer(i)->getMaterial();

		mat.setTexture(0, tsrc->getTextureForMesh(m_hud_elem->textures[i]));
	}

	m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::TEXS);
}

void MeshHUDSceneNode::updateRotation()
{
	if (!(m_hud_elem->change_flags & u8(MeshHUDStatsChange::ROT)))
		return;

	setRotation(m_hud_elem->rotation);

	m_hud_elem->change_flags &= (~(u8)MeshHUDStatsChange::ROT);
}

void MeshHUDSceneNode::step(f32 dtime)
{
	setMeshColor(m_client->getEnv().getLocalPlayer()->light_color);

	updateRect();

	updateRotation();

	updateScale();

	updateMesh(m_hud_elem->text, m_hud_elem->textures);

	updateTextures();

	m_lighting = m_hud_elem->lighting;
}

/*
 * HUDScene class
 */
HUDScene::HUDScene(scene::ISceneManager *mgr, Client *client,
		HudElement *elem, const v3f &mesh_pos,
		const v3f &mesh_rot, bool lighting, bool is_hand)
	: m_client(client)
{
	m_smgr = mgr->createNewSceneManager(false);

	m_cam = m_smgr->addCameraSceneNode();
	m_cam->setAspectRatio(client->getCamera()->getCameraNode()->getAspectRatio());
	m_cam->setFOV(72.0*M_PI/180.0);
	m_cam->setNearValue(10);
	m_cam->setFarValue(1000);

	if (!is_hand)
		m_hud_node = new MeshHUDSceneNode(m_smgr, client, lighting, elem);
	else
		m_hud_node = new WieldMeshHUDSceneNode(m_smgr, client, lighting);
	v3f pos = mesh_pos;
	// Makes each HUD node locate always at the same depth
	pos.Z = 65.0f;
	m_hud_node->setPosition(pos);
	m_hud_node->setRotation(mesh_rot);
}

HUDScene::~HUDScene()
{
	m_smgr->drop();
}
