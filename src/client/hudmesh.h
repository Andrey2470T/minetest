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

#pragma once

#include <string>
#include <vector>
#include "irrlichttypes_extrabloated.h"
#include "client/mesh_cache.h"
#include "settings.h"
#include "client/shadows/dynamicshadowsrender.h"
#include "client/mesh.h"

struct ItemStack;
class Client;
class ITextureSource;
struct ContentFeatures;
class ShadowRenderer;
class HudElement;


class HUDMeshCacheManager : public MeshCacheManager
{
public:
	HUDMeshCacheManager();

	~HUDMeshCacheManager() = default;

	DISABLE_CLASS_COPY(HUDMeshCacheManager);

	virtual ItemMesh* getOrCreateMesh(const ItemStack &item, Client *client) override;

	void getFileMesh(ItemMesh *imesh, std::string text,
		std::vector<std::string> textures, Client *client);

protected:
	virtual void getCubeMesh(ItemMesh *imesh, const ContentFeatures &f) override;

	virtual void getExtrusionMesh(ItemMesh *imesh, ITextureSource *tsrc, std::string image,
		std::string overlay_image, video::SColor color=video::SColor(0xFFFFFFFF),
		video::SColor overlay_color=video::SColor(0xFFFFFFFF), u8 num_frames=1) override;

private:
	bool m_enable_shaders;
	bool m_anisotropic_filter;
	bool m_bilinear_filter;
	bool m_trilinear_filter;

	std::unordered_map<std::string, std::unique_ptr<ItemMesh>> m_cache;
};

class MeshHUDSceneNode : public scene::IMeshSceneNode
{
public:
	MeshHUDSceneNode(scene::ISceneManager *mgr, Client *client=nullptr, bool lighting=false);

	virtual ~MeshHUDSceneNode()
	{
		if (m_shadow)
			m_shadow->removeNodeFromShadowList(this);
		if (!m_mesh_cached)
			delete m_hud_mesh;
	}

	virtual scene::IMesh *getMesh(void) override { return m_hud_mesh->mesh; }

	virtual bool isReadOnlyMaterials() const override { return false; }

	virtual void setMesh(scene::IMesh *mesh) override {}

	virtual void setReadOnlyMaterials(bool readonly) override {};

	virtual const core::aabbox3df &getBoundingBox() const { return m_box; };

	virtual void OnRegisterSceneNode();
	virtual void render();

	//void setMeshScale(const v3f &scale) { scaleMesh(m_hud_mesh->mesh, scale); }
	v3f getConstantMeshScale() const { return m_hud_mesh->constant_scale; }

	void setLighting(bool lighting) { m_lighting = lighting; }

	// Gets the override colors and multiply them by the light color
	void setLightColor(video::SColor light_color);

	void updateMesh(std::string text, std::vector<std::string> textures);

protected:
	Client *m_client;
	ItemMesh *m_hud_mesh = nullptr;

	core::aabbox3df m_box;

	// Non-cached mesh memory is fully handled by this class
	bool m_mesh_cached = false;

protected:
	ShadowRenderer *m_shadow = nullptr;

	bool m_lighting;
	bool m_enable_shaders;

	// Last set 'text' hud def field
	std::string m_last_text = "";
};


class HUDScene : virtual public IReferenceCounted
{
public:
	HUDScene(scene::ISceneManager *mgr, Client *client,
		const HudElement *elem=nullptr, const v3f &mesh_pos=v3f(),
		const v3f &mesh_rot=v3f(), bool lighting=false, bool is_hand=false);

	~HUDScene();

	// Transforms the position from the UV to the view space
	//v3f calculateViewPos(v2f screen_pos);

	void updateRect();
	void updateScale();

	MeshHUDSceneNode* getMeshNode() const { return m_hud_node; }
	scene::ISceneManager *getSceneManager() const { return m_smgr; }

	core::recti getViewPortRect() const { return m_rect; }
	//v2s32 getULCrnShift() const { return m_ul_crn_shift; }
	//core::dimension2du getRenderTextureSize() const { return m_render_texture_size; }

	void step(f32 dtime);
private:
	Client *m_client;
	scene::ISceneManager *m_smgr;
	scene::ICameraSceneNode *m_cam;
	MeshHUDSceneNode *m_hud_node;

	//core::dimension2du m_render_texture_size{0, 0};
	core::recti m_original_rect;
	core::recti m_rect;
	//v2s32 m_constant_ul_crn{0, 0};
	//v2s32 m_ul_crn_shift{0, 0};
	// HUDScene position range: [-1.0..1.0]
	v2f m_scene_pos{0.0f, 0.0f};

	const HudElement *m_hud_elem;

	// Last set 'scale.X' hud def field
	v3f m_last_relative_scale{1.0f};
	// Mainly WIELD_SCALE_FACTOR/WIELD_SCALE_FACTOR_EXTRUDED
	// calculated by getOrCreateMesh()
	v3f m_last_constant_scale{1.0f};
};
