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

#include "client/wield_hudmesh.h"
#include "noise.h"
#include "client/client.h"
#include "client/localplayer.h"

WieldMeshHUDSceneNode::WieldMeshHUDSceneNode(scene::ISceneManager *mgr, Client *client, bool lighting)
	: MeshHUDSceneNode(mgr, client, lighting)
{
	m_camera = client->getCamera();

	m_arm_inertia = g_settings->getBool("arm_inertia");

	m_mesh_cached = true;
}

void WieldMeshHUDSceneNode::setDigging(s32 button)
{
	if (m_digging_button == -1)
		m_digging_button = button;
}

void WieldMeshHUDSceneNode::wield(const ItemStack &item)
{
	if (item.name != m_cur_item.name ||
			item.metadata != m_cur_item.metadata) {
		m_cur_item = item;
		m_hud_mesh = m_client->getEnv().getMeshManager()->getOrCreateMesh(item, m_client);
		setScale(m_hud_mesh->constant_scale);
	}

}

void WieldMeshHUDSceneNode::step(f32 dtime)
{
	setMeshColor(m_client->getEnv().getLocalPlayer()->light_color);
}

/*static inline v2f dir(const v2f &pos_dist)
{
	f32 x = pos_dist.X - WIELDMESH_OFFSET_X;
	f32 y = pos_dist.Y - WIELDMESH_OFFSET_Y;

	f32 x_abs = std::fabs(x);
	f32 y_abs = std::fabs(y);

	if (x_abs >= y_abs) {
		y *= (1.0f / x_abs);
		x /= x_abs;
	}

	if (y_abs >= x_abs) {
		x *= (1.0f / y_abs);
		y /= y_abs;
	}

	return v2f(std::fabs(x), std::fabs(y));
}*/

/*void WieldMeshHUDSceneNode::addArmInertia(f32 player_yaw)
{
	m_cam_vel.X = std::fabs(rangelim(m_last_cam_pos.X - player_yaw,
		-100.0f, 100.0f) / 0.016f) * 0.01f;
	m_cam_vel.Y = std::fabs((m_last_cam_pos.Y - m_camera_direction.Y) / 0.016f);
	f32 gap_X = std::fabs(WIELDMESH_OFFSET_X - m_wieldmesh_offset.X);
	f32 gap_Y = std::fabs(WIELDMESH_OFFSET_Y - m_wieldmesh_offset.Y);

	if (m_cam_vel.X > 1.0f || m_cam_vel.Y > 1.0f) {*/
		/*
		    The arm moves relative to the camera speed,
		    with an acceleration factor.
		*/

		/*if (m_cam_vel.X > 1.0f) {
			if (m_cam_vel.X > m_cam_vel_old.X)
				m_cam_vel_old.X = m_cam_vel.X;

			f32 acc_X = 0.12f * (m_cam_vel.X - (gap_X * 0.1f));
			m_wieldmesh_offset.X += m_last_cam_pos.X < player_yaw ? acc_X : -acc_X;

			if (m_last_cam_pos.X != player_yaw)
				m_last_cam_pos.X = player_yaw;

			m_wieldmesh_offset.X = rangelim(m_wieldmesh_offset.X,
				WIELDMESH_OFFSET_X - (WIELDMESH_AMPLITUDE_X * 0.5f),
				WIELDMESH_OFFSET_X + (WIELDMESH_AMPLITUDE_X * 0.5f));
		}

		if (m_cam_vel.Y > 1.0f) {
			if (m_cam_vel.Y > m_cam_vel_old.Y)
				m_cam_vel_old.Y = m_cam_vel.Y;

			f32 acc_Y = 0.12f * (m_cam_vel.Y - (gap_Y * 0.1f));
			m_wieldmesh_offset.Y +=
				m_last_cam_pos.Y > m_camera_direction.Y ? acc_Y : -acc_Y;

			if (m_last_cam_pos.Y != m_camera_direction.Y)
				m_last_cam_pos.Y = m_camera_direction.Y;

			m_wieldmesh_offset.Y = rangelim(m_wieldmesh_offset.Y,
				WIELDMESH_OFFSET_Y - (WIELDMESH_AMPLITUDE_Y * 0.5f),
				WIELDMESH_OFFSET_Y + (WIELDMESH_AMPLITUDE_Y * 0.5f));
		}

		m_arm_dir = dir(m_wieldmesh_offset);
	} else {*/
		/*
		    Now the arm gets back to its default position when the camera stops,
		    following a vector, with a smooth deceleration factor.
		*/

		/*f32 dec_X = 0.35f * (std::min(15.0f, m_cam_vel_old.X) * (1.0f +
			(1.0f - m_arm_dir.X))) * (gap_X / 20.0f);

		f32 dec_Y = 0.25f * (std::min(15.0f, m_cam_vel_old.Y) * (1.0f +
			(1.0f - m_arm_dir.Y))) * (gap_Y / 15.0f);

		if (gap_X < 0.1f)
			m_cam_vel_old.X = 0.0f;

		m_wieldmesh_offset.X -=
			m_wieldmesh_offset.X > WIELDMESH_OFFSET_X ? dec_X : -dec_X;

		if (gap_Y < 0.1f)
			m_cam_vel_old.Y = 0.0f;

		m_wieldmesh_offset.Y -=
			m_wieldmesh_offset.Y > WIELDMESH_OFFSET_Y ? dec_Y : -dec_Y;
	}
}

// Returns the fractional part of x
inline f32 my_modf(f32 x)
{
	float dummy;
	return std::modf(x, &dummy);
}*/

/*void WieldMeshHUDSceneNode::step(f32 dtime, HUDMeshCacheManager *mesh_mgr)
{
	setLightColor(m_client->getEnv().getLocalPlayer()->light_color);

	// Arm inertia animation
	/*if (m_arm_inertia)
		addArmInertia(yaw);*/

	// Initial position and rotation
	//v3f wield_position = v3f(m_wieldmesh_offset.X, m_wieldmesh_offset.Y, 65);
	//v3f wield_rotation = v3f(-100, 120, -100);

	// Wield change animation
	//wield_position.Y += std::abs(m_wield_change_timer)*320 - 40;
	/*if (m_wield_change_timer.start) {
		m_wield_change_timer.time = MYMIN(m_wield_change_timer.time + dtime, 1.0f);
		wield_position.Y += 40*m_wield_change_timer.time - 40;

		if (m_wield_change_timer.time >= 1.0f)
		{
			m_wield_change_timer.time = 0.0f;
			m_wield_change_timer.start = false;
		}
	}*/

	// Digging animation
	/*if(m_digging_anim < 0.05 || m_digging_anim > 0.5)
	{
		f32 frac = 1.0;
		if(m_digging_anim > 0.5)
			frac = 2.0 * (m_digging_anim - 0.5);
		// This value starts from 1 and settles to 0
		f32 ratiothing = std::pow((1.0f - tool_reload_ratio), 0.5f);
		f32 ratiothing2 = (easeCurve(ratiothing*0.5))*2.0;
		wield_position.Y -= frac * 25.0f * std::pow(ratiothing2, 1.7f);
		wield_position.X -= frac * 35.0f * std::pow(ratiothing2, 1.1f);
		wield_rotation.Y += frac * 70.0f * std::pow(ratiothing2, 1.4f);
	}
	// If player is digging/placing, do the digging animation
	if (m_digging_button != -1)
	{
		f32 digfrac = m_digging_anim;
		wield_position.X -= 50 * std::sin(std::pow(digfrac, 0.8f) * M_PI);
		wield_position.Y += 24 * std::sin(digfrac * 1.8 * M_PI);
		wield_position.Z += 25 * 0.5;

		// Euler angles are PURE EVIL, so why not use quaternions?
		core::quaternion quat_begin(wield_rotation * core::DEGTORAD);
		core::quaternion quat_end(v3f(80, 30, 100) * core::DEGTORAD);
		core::quaternion quat_slerp;
		quat_slerp.slerp(quat_begin, quat_end, std::sin(digfrac * M_PI));
		quat_slerp.toEuler(wield_rotation);
		wield_rotation *= core::RADTODEG;

	// else do the bobbing animation
	} else {
		f32 bobfrac = my_modf(m_view_bobbing_anim);
		wield_position.X -= std::sin(bobfrac*M_PI*2.0) * 3.0;
		wield_position.Y += std::sin(my_modf(bobfrac*2.0)*M_PI) * 3.0;
	}*/
	//setPosition(wield_position);
	//setRotation(wield_rotation);




	/*if (m_digging_button != -1) {
		f32 offset = dtime * 3.5f;
		float m_digging_anim_was = m_digging_anim;
		m_digging_anim += offset;

		// Digging has finished
		if (m_digging_anim >= 1)
		{
			m_digging_anim = 0;
			m_digging_button = -1;
		}
		float lim = 0.15;
		// We have stepped over the limit value (0.15) on this iteration, trigger some event
		if(m_digging_anim_was < lim && m_digging_anim >= lim)
		{
			if (m_digging_button == 0) {
				m_client->getEventManager()->put(new SimpleTriggerEvent(MtEvent::CAMERA_PUNCH_LEFT));
			} else if(m_digging_button == 1) {
				m_client->getEventManager()->put(new SimpleTriggerEvent(MtEvent::CAMERA_PUNCH_RIGHT));
			}
		}
	}*/
//}
