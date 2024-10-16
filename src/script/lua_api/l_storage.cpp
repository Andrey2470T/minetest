/*
Minetest
Copyright (C) 2013 celeron55, Perttu Ahola <celeron55@gmail.com>
Copyright (C) 2017 nerzhul, Loic Blot <loic.blot@unix-experience.fr>

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

#include "lua_api/l_storage.h"
#include "l_internal.h"
#include "server.h"

int ModApiStorage::l_get_mod_storage(lua_State *L)
{
	// Note that this is wrapped in Lua, see builtin/common/mod_storage.lua
	std::string mod_name = readParam<std::string>(L, 1);

	if (IGameDef *gamedef = getGameDef(L)) {
		StorageRef::create(L, mod_name, gamedef->getModStorageDatabase());
	} else {
		assert(false); // this should not happen
		lua_pushnil(L);
	}
	return 1;
}

void ModApiStorage::Initialize(lua_State *L, int top)
{
	API_FCT(get_mod_storage);
}

void StorageRef::create(lua_State *L, const std::string &mod_name, ModMetadataDatabase *db)
{
	StorageRef *o = new StorageRef(mod_name, db);
	*(void **)(lua_newuserdata(L, sizeof(void *))) = o;
	luaL_getmetatable(L, className);
	lua_setmetatable(L, -2);
}

int StorageRef::gc_object(lua_State *L)
{
	StorageRef *o = *(StorageRef **)(lua_touserdata(L, 1));
	delete o;
	return 0;
}

void StorageRef::Register(lua_State *L)
{
	lua_newtable(L);
	int methodtable = lua_gettop(L);
	luaL_newmetatable(L, className);
	int metatable = lua_gettop(L);

	lua_pushliteral(L, "__metatable");
	lua_pushvalue(L, methodtable);
	lua_settable(L, metatable);  // hide metatable from Lua getmetatable()

	lua_pushliteral(L, "metadata_class");
	lua_pushlstring(L, className, strlen(className));
	lua_settable(L, metatable);

	lua_pushliteral(L, "__index");
	lua_pushvalue(L, methodtable);
	lua_settable(L, metatable);

	lua_pushliteral(L, "__gc");
	lua_pushcfunction(L, gc_object);
	lua_settable(L, metatable);

	lua_pushliteral(L, "__eq");
	lua_pushcfunction(L, l_equals);
	lua_settable(L, metatable);

	lua_pop(L, 1);  // drop metatable

	luaL_register(L, nullptr, methods);  // fill methodtable
	lua_pop(L, 1);  // drop methodtable
}

StorageRef* StorageRef::checkobject(lua_State *L, int narg)
{
	luaL_checktype(L, narg, LUA_TUSERDATA);
	void *ud = luaL_checkudata(L, narg, className);
	if (!ud) luaL_typerror(L, narg, className);
	return *(StorageRef**)ud;  // unbox pointer
}

IMetadata* StorageRef::getmeta(bool auto_create)
{
	return &m_object;
}

void StorageRef::clearMeta()
{
	m_object.clear();
}

const char StorageRef::className[] = "StorageRef";
const luaL_Reg StorageRef::methods[] = {
	luamethod(MetaDataRef, contains),
	luamethod(MetaDataRef, get),
	luamethod(MetaDataRef, get_string),
	luamethod(MetaDataRef, set_string),
	luamethod(MetaDataRef, get_int),
	luamethod(MetaDataRef, set_int),
	luamethod(MetaDataRef, get_float),
	luamethod(MetaDataRef, set_float),
	luamethod(MetaDataRef, to_table),
	luamethod(MetaDataRef, from_table),
	luamethod(MetaDataRef, equals),
	{0,0}
};
