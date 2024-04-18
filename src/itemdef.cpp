/*
Minetest
Copyright (C) 2010-2013 celeron55, Perttu Ahola <celeron55@gmail.com>
Copyright (C) 2013 Kahrl <kahrl@gmx.net>

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

#include "itemdef.h"

#include "nodedef.h"
#include "tool.h"
#include "inventory.h"
#ifndef SERVER
#include "client/mapblock_mesh.h"
#include "client/mesh.h"
#include "client/mesh_cache.h"
#include "client/client.h"
#endif
#include "log.h"
#include "settings.h"
#include "util/serialize.h"
#include "util/container.h"
#include "util/thread.h"
#include "util/pointedthing.h"
#include <map>
#include <set>

TouchInteraction::TouchInteraction()
{
	pointed_nothing = TouchInteractionMode_USER;
	pointed_node    = TouchInteractionMode_USER;
	pointed_object  = TouchInteractionMode_USER;
}

TouchInteractionMode TouchInteraction::getMode(PointedThingType pointed_type) const
{
	TouchInteractionMode result;
	switch (pointed_type) {
	case POINTEDTHING_NOTHING:
		result = pointed_nothing;
		break;
	case POINTEDTHING_NODE:
		result = pointed_node;
		break;
	case POINTEDTHING_OBJECT:
		result = pointed_object;
		break;
	default:
		FATAL_ERROR("Invalid PointedThingType given to TouchInteraction::getMode");
	}

	if (result == TouchInteractionMode_USER) {
		if (pointed_type == POINTEDTHING_OBJECT)
			result = g_settings->get("touch_punch_gesture") == "long_tap" ?
					LONG_DIG_SHORT_PLACE : SHORT_DIG_LONG_PLACE;
		else
			result = LONG_DIG_SHORT_PLACE;
	}

	return result;
}

void TouchInteraction::serialize(std::ostream &os) const
{
	writeU8(os, pointed_nothing);
	writeU8(os, pointed_node);
	writeU8(os, pointed_object);
}

void TouchInteraction::deSerialize(std::istream &is)
{
	u8 tmp = readU8(is);
	if (is.eof())
		throw SerializationError("");
	if (tmp < TouchInteractionMode_END)
		pointed_nothing = (TouchInteractionMode)tmp;

	tmp = readU8(is);
	if (is.eof())
		throw SerializationError("");
	if (tmp < TouchInteractionMode_END)
		pointed_node = (TouchInteractionMode)tmp;

	tmp = readU8(is);
	if (is.eof())
		throw SerializationError("");
	if (tmp < TouchInteractionMode_END)
		pointed_object = (TouchInteractionMode)tmp;
}

/*
	ItemDefinition
*/
ItemDefinition::ItemDefinition()
{
	resetInitial();
}

ItemDefinition::ItemDefinition(const ItemDefinition &def)
{
	resetInitial();
	*this = def;
}

ItemDefinition& ItemDefinition::operator=(const ItemDefinition &def)
{
	if(this == &def)
		return *this;

	reset();

	type = def.type;
	name = def.name;
	description = def.description;
	short_description = def.short_description;
	inventory_image = def.inventory_image;
	inventory_overlay = def.inventory_overlay;
	wield_image = def.wield_image;
	wield_overlay = def.wield_overlay;
	wield_scale = def.wield_scale;
	stack_max = def.stack_max;
	usable = def.usable;
	liquids_pointable = def.liquids_pointable;
	pointabilities = def.pointabilities;
	if (def.tool_capabilities)
		tool_capabilities = new ToolCapabilities(*def.tool_capabilities);
	wear_bar_params = def.wear_bar_params;
	groups = def.groups;
	node_placement_prediction = def.node_placement_prediction;
	place_param2 = def.place_param2;
	wallmounted_rotate_vertical = def.wallmounted_rotate_vertical;
	sound_place = def.sound_place;
	sound_place_failed = def.sound_place_failed;
	sound_use = def.sound_use;
	sound_use_air = def.sound_use_air;
	range = def.range;
	palette_image = def.palette_image;
	color = def.color;
	touch_interaction = def.touch_interaction;
	return *this;
}

ItemDefinition::~ItemDefinition()
{
	reset();
}

void ItemDefinition::resetInitial()
{
	// Initialize pointers to NULL so reset() does not delete undefined pointers
	tool_capabilities = NULL;
	wear_bar_params = std::nullopt;
	reset();
}

void ItemDefinition::reset()
{
	type = ITEM_NONE;
	name.clear();
	description.clear();
	short_description.clear();
	inventory_image.clear();
	inventory_overlay.clear();
	wield_image.clear();
	wield_overlay.clear();
	palette_image.clear();
	color = video::SColor(0xFFFFFFFF);
	wield_scale = v3f(1.0, 1.0, 1.0);
	stack_max = 99;
	usable = false;
	liquids_pointable = false;
	pointabilities = std::nullopt;
	delete tool_capabilities;
	tool_capabilities = NULL;
	wear_bar_params.reset();
	groups.clear();
	sound_place = SoundSpec();
	sound_place_failed = SoundSpec();
	sound_use = SoundSpec();
	sound_use_air = SoundSpec();
	range = -1;
	node_placement_prediction.clear();
	place_param2.reset();
	wallmounted_rotate_vertical = false;
	touch_interaction = TouchInteraction();
}

void ItemDefinition::serialize(std::ostream &os, u16 protocol_version) const
{
	// protocol_version >= 37
	u8 version = 6;
	writeU8(os, version);
	writeU8(os, type);
	os << serializeString16(name);
	os << serializeString16(description);
	os << serializeString16(inventory_image);
	os << serializeString16(wield_image);
	writeV3F32(os, wield_scale);
	writeS16(os, stack_max);
	writeU8(os, usable);
	writeU8(os, liquids_pointable);

	std::string tool_capabilities_s;
	if (tool_capabilities) {
		std::ostringstream tmp_os(std::ios::binary);
		tool_capabilities->serialize(tmp_os, protocol_version);
		tool_capabilities_s = tmp_os.str();
	}
	os << serializeString16(tool_capabilities_s);

	writeU16(os, groups.size());
	for (const auto &group : groups) {
		os << serializeString16(group.first);
		writeS16(os, group.second);
	}

	os << serializeString16(node_placement_prediction);

	// Version from ContentFeatures::serialize to keep in sync
	sound_place.serializeSimple(os, protocol_version);
	sound_place_failed.serializeSimple(os, protocol_version);

	writeF32(os, range);
	os << serializeString16(palette_image);
	writeARGB8(os, color);
	os << serializeString16(inventory_overlay);
	os << serializeString16(wield_overlay);

	os << serializeString16(short_description);

	if (protocol_version <= 43) {
		// Uncertainity whether 0 is the specified prediction or means disabled
		if (place_param2)
			os << *place_param2;
		else
			os << (u8)0;
	}

	sound_use.serializeSimple(os, protocol_version);
	sound_use_air.serializeSimple(os, protocol_version);

	os << (u8)place_param2.has_value(); // protocol_version >= 43
	if (place_param2)
		os << *place_param2;

	writeU8(os, wallmounted_rotate_vertical);
	touch_interaction.serialize(os);

	std::string pointabilities_s;
	if (pointabilities) {
		std::ostringstream tmp_os(std::ios::binary);
		pointabilities->serialize(tmp_os);
		pointabilities_s = tmp_os.str();
	}
	os << serializeString16(pointabilities_s);

	if (wear_bar_params.has_value()) {
		writeU8(os, 1);
		wear_bar_params->serialize(os);
	} else {
		writeU8(os, 0);
	}
}

void ItemDefinition::deSerialize(std::istream &is, u16 protocol_version)
{
	// Reset everything
	reset();

	// Deserialize
	int version = readU8(is);
	if (version < 6)
		throw SerializationError("unsupported ItemDefinition version");

	type = static_cast<ItemType>(readU8(is));
	if (type >= ItemType_END) {
		type = ITEM_NONE;
	}

	name = deSerializeString16(is);
	description = deSerializeString16(is);
	inventory_image = deSerializeString16(is);
	wield_image = deSerializeString16(is);
	wield_scale = readV3F32(is);
	stack_max = readS16(is);
	usable = readU8(is);
	liquids_pointable = readU8(is);

	std::string tool_capabilities_s = deSerializeString16(is);
	if (!tool_capabilities_s.empty()) {
		std::istringstream tmp_is(tool_capabilities_s, std::ios::binary);
		tool_capabilities = new ToolCapabilities;
		tool_capabilities->deSerialize(tmp_is);
	}

	groups.clear();
	u32 groups_size = readU16(is);
	for(u32 i=0; i<groups_size; i++){
		std::string name = deSerializeString16(is);
		int value = readS16(is);
		groups[name] = value;
	}

	node_placement_prediction = deSerializeString16(is);

	sound_place.deSerializeSimple(is, protocol_version);
	sound_place_failed.deSerializeSimple(is, protocol_version);

	range = readF32(is);
	palette_image = deSerializeString16(is);
	color = readARGB8(is);
	inventory_overlay = deSerializeString16(is);
	wield_overlay = deSerializeString16(is);

	// If you add anything here, insert it inside the try-catch
	// block to not need to increase the version.
	try {
		short_description = deSerializeString16(is);

		if (protocol_version <= 43) {
			place_param2 = readU8(is);
			// assume disabled prediction
			if (place_param2 == 0)
				place_param2.reset();
		}

		sound_use.deSerializeSimple(is, protocol_version);
		sound_use_air.deSerializeSimple(is, protocol_version);

		if (is.eof())
			throw SerializationError("");

		if (readU8(is)) // protocol_version >= 43
			place_param2 = readU8(is);

		wallmounted_rotate_vertical = readU8(is); // 0 if missing
		touch_interaction.deSerialize(is);

		std::string pointabilities_s = deSerializeString16(is);
		if (!pointabilities_s.empty()) {
			std::istringstream tmp_is(pointabilities_s, std::ios::binary);
			pointabilities = std::make_optional<Pointabilities>();
			pointabilities->deSerialize(tmp_is);
		}

		if (readU8(is)) {
			wear_bar_params = WearBarParams::deserialize(is);
		}
	} catch(SerializationError &e) {};
}

#ifndef SERVER
// Creates and handles item meshes for inventory slots
class InvMeshCacheManager : public MeshCacheManager
{
public:
	InvMeshCacheManager()
	{
		m_main_thread = std::this_thread::get_id();
	}

	~InvMeshCacheManager() = default;

	DISABLE_CLASS_COPY(InvMeshCacheManager);

	virtual ItemMesh* getOrCreateMesh(const ItemStack &item, Client *client) override
	{
		//infostream << "Inv getOrCreateMesh(): 1" << std::endl;
		// This is not thread-safe
		sanity_check(std::this_thread::get_id() == m_main_thread);

		ITextureSource *tsrc = client->getTextureSource();
		IItemDefManager *idef = client->getItemDefManager();
		const NodeDefManager *ndef = client->getNodeDefManager();

		const ItemDefinition &def = item.getDefinition(idef);
		const ContentFeatures &f = ndef->get(def.name);

		std::string inventory_image = item.getInventoryImage(idef);
		std::string inventory_overlay = item.getInventoryOverlay(idef);
		std::string cache_key = def.name;

		if (!inventory_image.empty())
			cache_key += "/" + inventory_image;
		if (!inventory_overlay.empty())
			cache_key += ":" + inventory_overlay;
		//infostream << "Inv getOrCreateMesh(): 2" << std::endl;
		// Skip if already in cache
		auto it = m_cache.find(cache_key);
		//infostream << "Inv getOrCreateMesh(): 2.1" << std::endl;
		//infostream << "Inv getOrCreateMesh(): " << cache_key << std::endl;
		if (it != m_cache.end()) {
			//infostream << "Inv getOrCreateMesh(): drawtype: " << f.drawtype << std::endl;
			return it->second.get();
		}
		//infostream << "Inv getOrCreateMesh(): 3" << std::endl;
		infostream << "Lazily creating item texture and mesh for \""
				<< cache_key << "\"" << std::endl;

		// Create new ItemMesh
		ItemMesh *imesh = new ItemMesh();

		imesh->base_color = idef->getItemstackColor(item, client);
		//infostream << "Inv getOrCreateMesh(): 4" << std::endl;
		if (!inventory_image.empty()) {
			//infostream << "Inv getOrCreateMesh(): 5" << std::endl;
			getExtrusionMesh(imesh, tsrc, inventory_image, inventory_overlay);
			imesh->needs_shading = false;
		}
		else if (def.type == ITEM_NODE) {
			switch (f.drawtype) {
			case NDT_AIRLIKE: {
				//infostream << "Inv getOrCreateMesh(): 6" << std::endl;
				getExtrusionMesh(imesh, tsrc, "no_texture_airlike.png", inventory_overlay);
				imesh->needs_shading = false;
				break;
			}
			case NDT_NORMAL:
			case NDT_ALLFACES: {
			case NDT_LIQUID:
			case NDT_FLOWINGLIQUID:
				//infostream << "Inv getOrCreateMesh(): 7" << std::endl;
				getCubeMesh(imesh, f);
				break;
			}
			case NDT_PLANTLIKE: {
				//infostream << "Inv getOrCreateMesh(): 8" << std::endl;
				const TileLayer &l0 = f.tiles[0].layers[0];
				const TileLayer &l1 = f.tiles[0].layers[1];

				getExtrusionMesh(imesh, tsrc, tsrc->getTextureName(l0.texture_id),
					tsrc->getTextureName(l1.texture_id), l0.has_color ? l0.color : imesh->base_color,
					l1.has_color ? l1.color : imesh->base_color);
				break;
			}
			case NDT_PLANTLIKE_ROOTED: {
				//infostream << "Inv getOrCreateMesh(): 9" << std::endl;
				const TileLayer &l0 = f.special_tiles[0].layers[0];

				getExtrusionMesh(imesh, tsrc, tsrc->getTextureName(l0.texture_id), "",
					l0.has_color ? l0.color : imesh->base_color);
				break;
			}
			default: {
				//infostream << "Inv getOrCreateMesh(): 10" << std::endl;
				getSpecialNodeMesh(imesh, client, ndef->getId(def.name), def.place_param2);
				//infostream << "Inv getOrCreateMesh(): 10.1" << std::endl;
				scaleMesh(imesh->mesh, v3f(0.12f));
				//infostream << "Inv getOrCreateMesh(): 10.2" << std::endl;
				break;
			}}

			//infostream << "Inv getOrCreateMesh(): 10.3" << std::endl;
			rotateMeshXZby(imesh->mesh, -45);
			//infostream << "Inv getOrCreateMesh(): 10.4" << std::endl;
			rotateMeshYZby(imesh->mesh, -30);
		}

		//infostream << "Inv getOrCreateMesh(): 11" << std::endl;
		for (u32 i = 0; i < imesh->mesh->getMeshBufferCount(); ++i) {
			//infostream << "Inv getOrCreateMesh(): 11.1" << std::endl;
			scene::IMeshBuffer *buf = imesh->mesh->getMeshBuffer(i);
			//infostream << "Inv getOrCreateMesh(): 11.2" << std::endl;
			video::SMaterial &material = buf->getMaterial();
			//infostream << "Inv getOrCreateMesh(): 11.3" << std::endl;
			material.MaterialType = video::EMT_TRANSPARENT_ALPHA_CHANNEL;
			material.MaterialTypeParam = 0.5f;
			material.forEachTexture([] (auto &tex) {
				tex.MinFilter = video::ETMINF_NEAREST_MIPMAP_NEAREST;
				tex.MagFilter = video::ETMAGF_NEAREST;
			});
			material.BackfaceCulling = f.needsBackfaceCulling();
			material.Lighting = false;
			//infostream << "Inv getOrCreateMesh(): 11.4" << std::endl;
			buf->setDirty(scene::EBT_VERTEX);
			if (imesh->needs_shading)
				colorizeMeshBuffer(buf, &imesh->buffer_colors[i]);
			else
				setMeshBufferColor(buf, imesh->buffer_colors[i]);
			//infostream << "Inv getOrCreateMesh(): 11.5" << std::endl;
		}
		//infostream << "Inv getOrCreateMesh(): 12" << std::endl;
		if (imesh->mesh) {
			imesh->mesh->setHardwareMappingHint(scene::EHM_DYNAMIC, scene::EBT_VERTEX);
			imesh->mesh->setHardwareMappingHint(scene::EHM_STATIC, scene::EBT_INDEX);
		}

		m_cache[cache_key] = std::unique_ptr<ItemMesh>(imesh);
		//infostream << "Inv getOrCreateMesh(): 13" << std::endl;
		return imesh;
	}

protected:
	virtual void getCubeMesh(ItemMesh *imesh, const ContentFeatures &f) override
	{
		imesh->mesh = MeshBuilder::createCubicMesh();

		if (f.drawtype == NDT_FLOWINGLIQUID) {
			scaleMesh(imesh->mesh, v3f(1.2f, 0.03f, 1.2f));
			translateMesh(imesh->mesh, v3f(0.0f, -0.57f, 0.0f));
		} else if (f.drawtype == NDT_ALLFACES) {
			scaleMesh(imesh->mesh, v3f(f.visual_scale));
		} else
			scaleMesh(imesh->mesh, v3f(1.2f));

		postProcessNodeMesh(imesh, f, false, false, true);
	}

	virtual void getExtrusionMesh(ItemMesh *imesh, ITextureSource *tsrc,
		std::string image, std::string overlay_image, video::SColor color=video::SColor(0xFFFFFFFF),
		video::SColor overlay_color=video::SColor(0xFFFFFFFF), u8 num_frames=1) override
	{
		video::ITexture *texture = tsrc->getTextureForMesh(image);

		if (!texture)
			return;

		scene::SMesh *mesh = MeshBuilder::createExtrusionMesh(texture->getSize());
		mesh->getMeshBuffer(0)->getMaterial().setTexture(0, texture);

		if (!overlay_image.empty()) {
			//imesh->overlay_texture = tsrc->getTexture(overlay_image);

			scene::IMeshBuffer *clone_mbuf = cloneMeshBuffer(mesh->getMeshBuffer(0));
			clone_mbuf->getMaterial().setTexture(0, tsrc->getTexture(overlay_image));
			mesh->addMeshBuffer(clone_mbuf);
			clone_mbuf->drop();
		}

		for (u32 layer = 0; layer < mesh->getMeshBufferCount(); layer++) {
			video::SMaterial &material = mesh->getMeshBuffer(layer)->getMaterial();

			material.TextureLayers[0].TextureWrapU = video::ETC_CLAMP_TO_EDGE;
			material.TextureLayers[0].TextureWrapV = video::ETC_CLAMP_TO_EDGE;

			material.BackfaceCulling = true;
		}

		//imesh->inventory_texture = tsrc->getTexture(image);
		imesh->mesh = mesh;
		imesh->buffer_colors.push_back(color);
		imesh->buffer_colors.push_back(overlay_color);
	}

private:
	// The id of the thread that is allowed to use irrlicht directly
	std::thread::id m_main_thread;

	std::unordered_map<std::string, std::unique_ptr<ItemMesh>> m_cache;
};
#endif

/*
	CItemDefManager
*/

// SUGG: Support chains of aliases?

class CItemDefManager: public IWritableItemDefManager
{
public:
	CItemDefManager()
	{
		clear();
	}

	virtual ~CItemDefManager()
	{
		for (auto &item_definition : m_item_definitions) {
			delete item_definition.second;
		}
		m_item_definitions.clear();
	}
	virtual const ItemDefinition& get(const std::string &name_) const
	{
		// Convert name according to possible alias
		std::string name = getAlias(name_);
		// Get the definition
		auto i = m_item_definitions.find(name);
		if (i == m_item_definitions.cend())
			i = m_item_definitions.find("unknown");
		assert(i != m_item_definitions.cend());
		return *(i->second);
	}
	virtual const std::string &getAlias(const std::string &name) const
	{
		auto it = m_aliases.find(name);
		if (it != m_aliases.cend())
			return it->second;
		return name;
	}
	virtual void getAll(std::set<std::string> &result) const
	{
		result.clear();
		for (const auto &item_definition : m_item_definitions) {
			result.insert(item_definition.first);
		}

		for (const auto &alias : m_aliases) {
			result.insert(alias.first);
		}
	}
	virtual bool isKnown(const std::string &name_) const
	{
		// Convert name according to possible alias
		std::string name = getAlias(name_);
		// Get the definition
		return m_item_definitions.find(name) != m_item_definitions.cend();
	}

#ifndef SERVER
	virtual MeshCacheManager* getMeshManager() override { return &m_mesh_mgr; }

	virtual video::SColor getItemstackColor(const ItemStack &stack,
		Client *client) const
	{
		// Look for direct color definition
		const std::string &colorstring = stack.metadata.getString("color", 0);
		video::SColor directcolor;
		if (!colorstring.empty() && parseColorString(colorstring, directcolor, true))
			return directcolor;

		// See if there is a palette
		auto tsrc = client->getTextureSource();
		auto item_mgr = client->getItemDefManager();
		Palette *palette = tsrc->getPalette(stack.getDefinition(item_mgr).palette_image);
		const std::string &index = stack.metadata.getString("palette_index", 0);
		if (palette && !index.empty())
			return (*palette)[mystoi(index, 0, 255)];
		// Fallback color
		return get(stack.name).color;
	}
#endif
	void applyTextureOverrides(const std::vector<TextureOverride> &overrides)
	{
		infostream << "ItemDefManager::applyTextureOverrides(): Applying "
			"overrides to textures" << std::endl;

		for (const TextureOverride& texture_override : overrides) {
			if (m_item_definitions.find(texture_override.id) == m_item_definitions.end()) {
				continue; // Ignore unknown item
			}

			ItemDefinition* itemdef = m_item_definitions[texture_override.id];

			if (texture_override.hasTarget(OverrideTarget::INVENTORY))
				itemdef->inventory_image = texture_override.texture;

			if (texture_override.hasTarget(OverrideTarget::WIELD))
				itemdef->wield_image = texture_override.texture;
		}
	}
	void clear()
	{
		for (auto &i : m_item_definitions)
		{
			delete i.second;
		}
		m_item_definitions.clear();
		m_aliases.clear();

		// Add the four builtin items:
		//   "" is the hand
		//   "unknown" is returned whenever an undefined item
		//     is accessed (is also the unknown node)
		//   "air" is the air node
		//   "ignore" is the ignore node

		ItemDefinition* hand_def = new ItemDefinition;
		hand_def->name.clear();
		hand_def->wield_image = "wieldhand.png";
		hand_def->tool_capabilities = new ToolCapabilities;
		m_item_definitions.insert(std::make_pair("", hand_def));

		ItemDefinition* unknown_def = new ItemDefinition;
		unknown_def->type = ITEM_NODE;
		unknown_def->name = "unknown";
		m_item_definitions.insert(std::make_pair("unknown", unknown_def));

		ItemDefinition* air_def = new ItemDefinition;
		air_def->type = ITEM_NODE;
		air_def->name = "air";
		m_item_definitions.insert(std::make_pair("air", air_def));

		ItemDefinition* ignore_def = new ItemDefinition;
		ignore_def->type = ITEM_NODE;
		ignore_def->name = "ignore";
		m_item_definitions.insert(std::make_pair("ignore", ignore_def));
	}
	virtual void registerItem(const ItemDefinition &def)
	{
		TRACESTREAM(<< "ItemDefManager: registering " << def.name << std::endl);
		// Ensure that the "" item (the hand) always has ToolCapabilities
		if (def.name.empty())
			FATAL_ERROR_IF(!def.tool_capabilities, "Hand does not have ToolCapabilities");

		if(m_item_definitions.count(def.name) == 0)
			m_item_definitions[def.name] = new ItemDefinition(def);
		else
			*(m_item_definitions[def.name]) = def;

		// Remove conflicting alias if it exists
		bool alias_removed = (m_aliases.erase(def.name) != 0);
		if(alias_removed)
			infostream<<"ItemDefManager: erased alias "<<def.name
					<<" because item was defined"<<std::endl;
	}
	virtual void unregisterItem(const std::string &name)
	{
		verbosestream<<"ItemDefManager: unregistering \""<<name<<"\""<<std::endl;

		delete m_item_definitions[name];
		m_item_definitions.erase(name);
	}
	virtual void registerAlias(const std::string &name,
			const std::string &convert_to)
	{
		if (m_item_definitions.find(name) == m_item_definitions.end()) {
			TRACESTREAM(<< "ItemDefManager: setting alias " << name
				<< " -> " << convert_to << std::endl);
			m_aliases[name] = convert_to;
		}
	}
	void serialize(std::ostream &os, u16 protocol_version)
	{
		writeU8(os, 0); // version
		u16 count = m_item_definitions.size();
		writeU16(os, count);

		for (const auto &it : m_item_definitions) {
			ItemDefinition *def = it.second;
			// Serialize ItemDefinition and write wrapped in a string
			std::ostringstream tmp_os(std::ios::binary);
			def->serialize(tmp_os, protocol_version);
			os << serializeString16(tmp_os.str());
		}

		writeU16(os, m_aliases.size());

		for (const auto &it : m_aliases) {
			os << serializeString16(it.first);
			os << serializeString16(it.second);
		}
	}
	void deSerialize(std::istream &is, u16 protocol_version)
	{
		// Clear everything
		clear();

		if(readU8(is) != 0)
			throw SerializationError("unsupported ItemDefManager version");

		u16 count = readU16(is);
		for(u16 i=0; i<count; i++)
		{
			// Deserialize a string and grab an ItemDefinition from it
			std::istringstream tmp_is(deSerializeString16(is), std::ios::binary);
			ItemDefinition def;
			def.deSerialize(tmp_is, protocol_version);
			// Register
			registerItem(def);
		}
		u16 num_aliases = readU16(is);
		for(u16 i=0; i<num_aliases; i++)
		{
			std::string name = deSerializeString16(is);
			std::string convert_to = deSerializeString16(is);
			registerAlias(name, convert_to);
		}
	}

private:
	// Key is name
	std::map<std::string, ItemDefinition*> m_item_definitions;
	// Aliases
	StringMap m_aliases;
#ifndef SERVER
	mutable InvMeshCacheManager m_mesh_mgr;
#endif
};

IWritableItemDefManager* createItemDefManager()
{
	return new CItemDefManager();
}
