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
	infostream << "getFileMesh(): 1" << std::endl;
	imesh->mesh = client->getMesh(text);
	infostream << "getFileMesh(): 2" << std::endl;

	if (!(imesh->mesh))
		return;
	textures.resize(imesh->mesh->getMeshBufferCount());
	infostream << "getFileMesh(): 3" << std::endl;
	ITextureSource *tsrc = client->getTextureSource();
	IShaderSource *shdrsrc = client->getShaderSource();
	for (u32 i = 0; i < imesh->mesh->getMeshBufferCount(); i++) {
		imesh->buffer_colors.emplace_back(video::SColor(0xFFFFFFFF));
		infostream << "getFileMesh(): 4" << std::endl;
		video::SMaterial &material = imesh->mesh->getMeshBuffer(i)->getMaterial();
		material.setTexture(0, tsrc->getTextureForMesh(textures[i]));
		infostream << "getFileMesh(): 5" << std::endl;
		if (m_enable_shaders) {
			u32 shader_id = shdrsrc->getShader("object_shader", TILE_MATERIAL_BASIC, NDT_NORMAL);
			material.MaterialType = shdrsrc->getShaderInfo(shader_id).material;
		}
		else
			material.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;
		infostream << "getFileMesh(): 6" << std::endl;
		material.MaterialTypeParam = 0.5f;
		material.BackfaceCulling = false;
		material.forEachTexture([this] (auto &tex) {
			setMaterialFilters(tex, m_bilinear_filter,
				m_trilinear_filter, m_anisotropic_filter);
		});
	}
	infostream << "getFileMesh(): 7" << std::endl;
	if (m_enable_shaders)
		imesh->mesh->setHardwareMappingHint(scene::EHM_STATIC);
	else
		imesh->mesh->setHardwareMappingHint(scene::EHM_DYNAMIC);

	imesh->constant_scale = v3f(WIELD_SCALE_FACTOR);
	//scaleMesh(imesh->mesh, imesh->constant_scale);
	infostream << "getFileMesh(): 8" << std::endl;
}

void HUDMeshCacheManager::getCubeMesh(ItemMesh *imesh, const ContentFeatures &f)
{
	//infostream << "getCubeMesh(): 1" << std::endl;
	imesh->mesh = MeshBuilder::createCubicMesh();
	//infostream << "getCubeMesh(): 2" << std::endl;

	postProcessNodeMesh(imesh, f, false, true, true);
	//infostream << "getCubeMesh(): 3" << std::endl;
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
		//imesh->overlay_texture = tsrc->getTexture(overlay_image);

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

	//imesh->wield_texture = texture;
	imesh->mesh = mesh;
	imesh->buffer_colors.push_back(color);
	imesh->buffer_colors.push_back(overlay_color);
}


/*
 * MeshHUDSceneNode class
 */

MeshHUDSceneNode::MeshHUDSceneNode(scene::ISceneManager *mgr, Client *client, bool lighting):
	scene::IMeshSceneNode(mgr->getRootSceneNode(), mgr, -1), m_client(client),
	m_lighting(lighting)
{
	m_enable_shaders = g_settings->getBool("enable_shaders");

	m_hud_mesh = m_client->getEnv().getMeshManager()->getOrCreateMesh(ItemStack(), m_client);
	setScale(m_hud_mesh->constant_scale);

	setAutomaticCulling(scene::EAC_OFF);

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

void MeshHUDSceneNode::setLightColor(video::SColor light_color)
{
	scene::IMesh *mesh = m_hud_mesh->mesh;

	if (!mesh)
		return;
	for (u32 i = 0; i < mesh->getMeshBufferCount(); i++) {
		video::SColor bc(m_hud_mesh->buffer_colors[i]);

		bc.setRed(bc.getRed() * (m_lighting ? light_color.getRed() : 255) / 255);
		bc.setGreen(bc.getGreen() * (m_lighting ? light_color.getGreen() : 255) / 255);
		bc.setBlue(bc.getBlue() * (m_lighting ? light_color.getBlue() : 255) / 255);
		bc.setAlpha(255);

		scene::IMeshBuffer *buf = mesh->getMeshBuffer(i);
		buf->setDirty(scene::EBT_VERTEX);

		if (m_enable_shaders) {
			setMeshBufferColor(buf, bc);
			buf->getMaterial().EmissiveColor = m_lighting ? light_color : video::SColor(0xFFFFFFFF);
		}
		else
			colorizeMeshBuffer(buf, &bc);
	}
}

void MeshHUDSceneNode::updateMesh(std::string text, std::vector<std::string> textures)
{
	if (m_last_text == text)
		return;

	m_last_text = text;

	HUDMeshCacheManager *mesh_mgr = m_client->getEnv().getMeshManager();
	// Assume that the text string contains the itemstack name that is necessary to display
	//infostream << "m_last_text: " << m_last_text << std::endl;
	infostream << "updateMesh(): begin" << std::endl;
	if (m_last_text.find(':') != m_last_text.npos) {
		infostream << "updateMesh(): this is itemstack mesh" << std::endl;
		//infostream << "this is itemstack" << std::endl;
		m_hud_mesh = mesh_mgr->getOrCreateMesh(ItemStack(m_last_text, 1, 0, m_client->idef()), m_client);
		m_mesh_cached = true;
	}
	// Else assume that it is a mesh file name
	else {
		infostream << "updateMesh(): this is mesh from file" << std::endl;
		//infostream << "this is arbitrary mesh" << std::endl;
		m_hud_mesh = new ItemMesh();
		mesh_mgr->getFileMesh(m_hud_mesh, text, textures, m_client);
		m_mesh_cached = false;
	}
}

/*void MeshHUDSceneNode::step(f32 dtime, HUDMeshCacheManager *mesh_mgr)
{
	setLightColor(m_client->getEnv().getLocalPlayer()->light_color);

	if (!m_hud_elem)
		return;

	v3f cur_pos = getPosition();
	v3f new_pos = calculateViewPos(m_hud_elem->pos);
	new_pos.Z = m_hud_elem->z_offset;
	//infostream << "new pos: X: " << new_pos.X << ", Y: " << new_pos.Y << std::endl;

	if (cur_pos != new_pos) {
		//infostream << "set a new pos" << std::endl;
		setPosition(new_pos);
	}

	if (getRotation() != m_hud_elem->rotation)
		setRotation(m_hud_elem->rotation);

	// Relative scale
	v3f old_relative_scale = m_last_relative_scale;
	v3f new_scale(std::fabs(m_hud_elem->scale.X));
	//infostream << "new_scale: X: " << new_scale.X << ", Y: " << new_scale.Y << ", Z: " << new_scale.Z << std::endl;

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
	m_last_relative_scale = new_scale;

	v3f old_constant_scale = m_hud_mesh->constant_scale;
	//infostream << "old_constant_scale: X: " << old_constant_scale.X << ", Y: " << old_constant_scale.Y << ", Z: " << old_constant_scale.Z << std::endl;

	if (m_last_text != m_hud_elem->text) {
		m_last_text = m_hud_elem->text;

		//delete m_hud_mesh;

		// Assume that the text string contains the itemstack name that is necessary to display
		infostream << "m_last_text: " << m_last_text << std::endl;
		if (m_last_text.find(':') != m_last_text.npos) {
			infostream << "this is itemstack" << std::endl;
			m_hud_mesh = mesh_mgr->getOrCreateMesh(ItemStack(m_last_text, 1, 0, m_client->idef()), m_client);
			m_mesh_cached = true;
		}
		// Else assume that it is a mesh file name
		else {
			infostream << "this is arbitrary mesh" << std::endl;
			m_hud_mesh = new ItemMesh();
			mesh_mgr->getFileMesh(m_hud_mesh, m_hud_elem, m_client);
			m_mesh_cached = false;
		}
	}

	if (m_hud_mesh->constant_scale != old_constant_scale ||
			m_last_relative_scale != old_relative_scale)
		setScale(m_hud_mesh->constant_scale * m_last_relative_scale);

	m_lighting = m_hud_elem->lighting;
}*/


/*
 * HUDScene class
 */
HUDScene::HUDScene(scene::ISceneManager *mgr, Client *client,
		const HudElement *elem, const v3f &mesh_pos,
		const v3f &mesh_rot, bool lighting, bool is_hand)
	: m_client(client), m_hud_elem(elem)
{
	m_smgr = mgr->createNewSceneManager(false);

	m_cam = m_smgr->addCameraSceneNode();
	m_cam->setAspectRatio(client->getCamera()->getCameraNode()->getAspectRatio());
	m_cam->setFOV(72.0*M_PI/180.0);
	m_cam->setNearValue(10);
	m_cam->setFarValue(1000);

	if (!is_hand)
		m_hud_node = new MeshHUDSceneNode(m_smgr, client, lighting);
	else
		m_hud_node = new WieldMeshHUDSceneNode(m_smgr, client, lighting);
	v3f pos = mesh_pos;
	// Makes each HUD node locate always at the same depth
	pos.Z = 65.0f;
	m_hud_node->setPosition(pos);
	m_hud_node->setRotation(mesh_rot);

	v2u32 size = RenderingEngine::getWindowSize();
	m_original_rect = core::recti(0, 0, size.X, size.Y);
	m_rect = m_original_rect;
	//m_constant_ul_crn = v2s32(-(s32)size.X/2, (s32)size.Y/2);

	//m_render_texture_size = core::dimension2du(size.X, size.Y);
	//m_original_rect = core::recti(core::dimension2di(size.X, size.Y));
	//m_rect = m_original_rect;

	//infostream << "size: " << m_original_rect.getWidth() << ", " << m_original_rect.getHeight() << std::endl;
}

HUDScene::~HUDScene()
{
	m_smgr->drop();
}

void HUDScene::updateRect()
{
	v2f uv_pos = m_scene_pos / 2.0f + 0.5f;

	//infostream << "uv_pos: " << uv_pos.X << ", " << uv_pos.Y << std::endl;
	//infostream << "pos: " << m_hud_elem->pos.X << ", " << m_hud_elem->pos.Y << std::endl;

	if (uv_pos == m_hud_elem->pos)
		return;

	m_scene_pos = (m_hud_elem->pos - 0.5f) * 2.0f;
	//infostream << "scene pos: " << m_scene_pos.X << ", " << m_scene_pos.Y << std::endl;

	v2s32 new_pos(s32(m_scene_pos.X*m_original_rect.getWidth()/2.0f),
		s32(m_scene_pos.Y*m_original_rect.getHeight()/2.0f));

	//infostream << "new pos: " << new_pos.X << ", " << new_pos.Y << std::endl;

	m_rect = m_original_rect + new_pos;
	//m_ul_crn_shift = m_constant_ul_crn + new_pos;
	//infostream << "ul crn shift: " << m_ul_crn_shift.X << ", " << m_ul_crn_shift.Y << std::endl;
}

void HUDScene::updateScale()
{
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

	if (m_hud_node->getConstantMeshScale() != m_last_constant_scale ||
			new_scale != m_last_relative_scale) {
		m_last_constant_scale = m_hud_node->getConstantMeshScale();
		m_last_relative_scale = new_scale;

		m_hud_node->setScale(m_last_constant_scale * m_last_relative_scale);
	}
}

void HUDScene::step(f32 dtime)
{
	m_hud_node->setLightColor(m_client->getEnv().getLocalPlayer()->light_color);

	if (!m_hud_elem)
		return;

	updateRect();

	m_hud_node->setRotation(m_hud_elem->rotation);

	updateScale();
	//infostream << "old_constant_scale: X: " << old_constant_scale.X << ", Y: " << old_constant_scale.Y << ", Z: " << old_constant_scale.Z << std::endl;

	m_hud_node->updateMesh(m_hud_elem->text, m_hud_elem->textures);

	m_hud_node->setLighting(m_hud_elem->lighting);
}

/*v3f HUDScene::calculateViewPos(v2f screen_pos)
{
	v2f device_coords = (screen_pos - 0.5) * 2.0;
	infostream << "device_coords: x: " << device_coords.X << ", y: " << device_coords.Y << std::endl;

	f32 pos_to_v[4] = {device_coords.X, device_coords.Y, 0.0, 1.0};

	scene::ICameraSceneNode *cam = getSceneManager()->getActiveCamera();
	core::matrix4 proj = cam->getProjectionMatrix();
	core::matrix4 projinv;
	proj.getInverse(projinv);

	core::matrix4 view = cam->getViewMatrix();
	core::matrix4 viewinv;
	view.getInverse(viewinv);

	core::matrix4 projviewinv = projinv * viewinv;

	projviewinv.multiplyWith1x4Matrix(pos_to_v);

	return v3f(
		pos_to_v[0] / pos_to_v[3],
		pos_to_v[1] / pos_to_v[3],
		pos_to_v[2] / pos_to_v[3]
	);
}*/
