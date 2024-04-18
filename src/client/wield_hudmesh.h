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

#include "client/hudmesh.h"
#include "inventory.h"

struct AnimationTimer
{
	bool start = false;

	f32 time = 0.0f; // from 0.0 - 1.0
};

class WieldMeshHUDSceneNode final : public MeshHUDSceneNode
{
public:
	WieldMeshHUDSceneNode(scene::ISceneManager *mgr, Client *client=nullptr, bool lighting=false);

	virtual ~WieldMeshHUDSceneNode() = default;

	// Start digging animation
	// Pass 0 for left click, 1 for right click
	void setDigging(s32 button);

	// Replace the wielded item mesh
	void wield(const ItemStack &item);

	inline void addArmInertia(f32 player_yaw);

private:
	Camera *m_camera;

	v2f m_wieldmesh_offset = v2f(55.0f, -35.0f);
	v2f m_arm_dir;
	v2f m_cam_vel;
	v2f m_cam_vel_old;
	v2f m_last_cam_pos;

	// Digging animation frame (0 <= m_digging_anim < 1)
	f32 m_digging_anim = 0.0f;
	// If -1, no digging animation
	// If 0, left-click digging animation
	// If 1, right-click digging animation
	s32 m_digging_button = -1;

	// Animation when changing wielded item
	AnimationTimer m_wield_change_timer;

	bool m_arm_inertia;

	ItemStack m_cur_item;
};
