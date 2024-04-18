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

#pragma once

#include "irrlichttypes_extrabloated.h"
#include "mapnode.h"
#include "client/mesh.h"

struct ItemStack;

/*
 * Structure for saving a cached mesh (cube, extrusion or loaded from file).
 */
struct ItemMesh
{
	scene::IMesh *mesh = nullptr;

	//video::ITexture *inventory_texture = nullptr;
	//video::ITexture *wield_texture = nullptr;
	//video::ITexture *overlay_texture = nullptr;

	/*
	 * Color taken from `color` itemstack's metafield or from a palette.
	 */
	video::SColor base_color = video::SColor(0xFFFFFFFF);
	/*
	 * Stores the tile color of each mesh buffer.
	 */
	std::vector<video::SColor> buffer_colors;
	/*
	 * If false, all faces of the item should have the same brightness.
	 * Disables shading based on normal vectors.
	 */
	bool needs_shading = true;

	/*
	 * This scale depends on the mesh kind (extruded, cube, ordinary mesh)
	 */
	v3f constant_scale = v3f(1.0f);

	ItemMesh()
	{
		buffer_colors.clear();
	}

	~ItemMesh()
	{
		mesh->drop();
	}

	DISABLE_CLASS_COPY(ItemMesh);
};

/*
 * Builds extrusion, cube and special meshes. It doesn`t need in creating.
 */
class MeshBuilder
{
public:
	MeshBuilder() = delete;

	~MeshBuilder() = delete;

	static scene::IMesh* createCubicMesh() {
		scene::IMesh *anim_mesh = createCubeMesh(v3f(1.0f));
		scene::IMesh *smesh = cloneMesh(anim_mesh);
		anim_mesh->drop();

		return smesh;
	}

	static scene::SMesh* createExtrusionMesh(core::dimension2du dim);

	static scene::SMesh* createSpecialNodeMesh(Client *client, content_t id, const std::optional<u8> &place_param2);
};


/*
 * Handles and caches meshes for inventory and HUD itemstacks
 */
class MeshCacheManager
{
public:
	MeshCacheManager() = default;

	virtual ~MeshCacheManager() = default;

	/*
	 * Gets a mesh for the itemstack from the own cache storage (e.g. unordered map) and
	 * creates it if it doesn't exist yet.
	 */
	virtual ItemMesh* getOrCreateMesh(const ItemStack &item, Client *client) = 0;

protected:
	virtual void getCubeMesh(ItemMesh *imesh, const ContentFeatures &f) = 0;

	virtual void getExtrusionMesh(ItemMesh *imesh, ITextureSource *tsrc, std::string image,
		std::string overlay_image, video::SColor color=video::SColor(0xFFFFFFFF),
		video::SColor overlay_color=video::SColor(0xFFFFFFFF), u8 num_frames=1) = 0;

	void getSpecialNodeMesh(ItemMesh *imesh, Client *client,
		content_t id, const std::optional<u8> &place_param2);

	void postProcessNodeMesh(ItemMesh *imesh, const ContentFeatures &f, bool use_shaders,
		bool set_material, bool apply_scale);
};
