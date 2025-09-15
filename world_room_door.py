#!/usr/bin/env python3
"""
world_room_door.py

Room+Door builder for PyBullet.

- Builds 4 thin static walls forming a rectangular room.
- Spawns a door with a revolute hinge (0..90 deg) using a tiny generated URDF.
- Utilities to (de)spawn, set door angle, and enable a motor later.

Usage:
    env = RoomDoorEnv(client_id=p.connect(p.GUI))
    env.build_room()
    env.spawn_door(base_pos=[1.5, 0.0, 0.0], size_xyz=[0.90, 0.04, 2.0], initial_angle_deg=20)
"""

from __future__ import annotations
import os
import math
import tempfile
from typing import List, Optional

import pybullet as p

class RoomDoorEnv:
    def __init__(
        self,
        client_id: int,
        room_size_xy=(8.0, 8.0),
        wall_height=2.5,
        wall_thickness=0.05,
        wall_color=(0.8, 0.8, 0.9, 1.0),
        door_color=(0.7, 0.4, 0.2, 1.0),
        # --- NEW params for a proper doorway gap and stability ---
        doorway_width=1.0,
        doorway_center_y=0.0,
        clearance_eps=0.01,  # small offset from wall to avoid contact jitter
    ):
        self.cid = client_id
        self.room_size_xy = room_size_xy
        self.wall_h = wall_height
        self.wall_t = wall_thickness
        self.wall_color = wall_color
        self.door_color = door_color

        # doorway config
        self.doorway_width = doorway_width
        self.doorway_center_y = doorway_center_y
        self.clearance_eps = clearance_eps

        self.wall_ids: List[int] = []
        self.door_id: Optional[int] = None
        self.door_joint_index: int = 0  # single revolute joint in our URDF
        self._tmp_urdf_path: Optional[str] = None

        # jamb segment ids (for optional collision filtering)
        self.left_jamb_id: Optional[int] = None
        self.right_jamb_id: Optional[int] = None

    # ---------- Room ----------

    def build_room(self):
        """Create 4 static walls (thin boxes) around the origin."""
        if self.wall_ids:
            return self.wall_ids

        sx, sy = self.room_size_xy
        h = self.wall_h
        t = self.wall_t
        half = lambda v: v * 0.5

        self.wall_ids = []

        # --- +X wall is SPLIT into two segments, leaving a doorway gap ---
        gap = max(0.0, float(self.doorway_width))
        y_center = float(self.doorway_center_y)

        # clamp doorway within room span
        y_min, y_max = -sy * 0.5, sy * 0.5
        half_gap = min(gap * 0.5, (y_max - y_min) * 0.5)
        y_left_end = max(y_min, y_center - half_gap)
        y_right_beg = min(y_max, y_center + half_gap)

        # Left +X segment: from y_min .. y_left_end
        left_len = max(0.0, (y_left_end - y_min))
        if left_len > 1e-6:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half(t), half(left_len), half(h)])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half(t), half(left_len), half(h)], rgbaColor=self.wall_color)
            pos = [half(sx), y_min + half(left_len), half(h)]
            self.left_jamb_id = p.createMultiBody(
                baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                basePosition=pos, baseOrientation=p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=self.cid
            )
            self.wall_ids.append(self.left_jamb_id)

        # Right +X segment: from y_right_beg .. y_max
        right_len = max(0.0, (y_max - y_right_beg))
        if right_len > 1e-6:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half(t), half(right_len), half(h)])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half(t), half(right_len), half(h)], rgbaColor=self.wall_color)
            pos = [half(sx), y_right_beg + half(right_len), half(h)]
            self.right_jamb_id = p.createMultiBody(
                baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                basePosition=pos, baseOrientation=p.getQuaternionFromEuler([0,0,0]),
                physicsClientId=self.cid
            )
            self.wall_ids.append(self.right_jamb_id)

        # -X wall (single slab)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half(t), half(sy), half(h)])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half(t), half(sy), half(h)], rgbaColor=self.wall_color)
        bid = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=[-half(sx), 0.0, half(h)], baseOrientation=p.getQuaternionFromEuler([0,0,0]),
            physicsClientId=self.cid
        )
        self.wall_ids.append(bid)

        # +Y wall
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half(sx), half(t), half(h)])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half(sx), half(t), half(h)], rgbaColor=self.wall_color)
        bid = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=[0.0, half(sy), half(h)], baseOrientation=p.getQuaternionFromEuler([0,0,0]),
            physicsClientId=self.cid
        )
        self.wall_ids.append(bid)

        # -Y wall
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half(sx), half(t), half(h)])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half(sx), half(t), half(h)], rgbaColor=self.wall_color)
        bid = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=[0.0, -half(sy), half(h)], baseOrientation=p.getQuaternionFromEuler([0,0,0]),
            physicsClientId=self.cid
        )
        self.wall_ids.append(bid)

        return self.wall_ids

    def clear_room(self):
        for wid in self.wall_ids:
            try:
                p.removeBody(wid, physicsClientId=self.cid)
            except Exception:
                pass
        self.wall_ids = []
        self.left_jamb_id = None
        self.right_jamb_id = None

    # ---------- Door (URDF) ----------

    def _generate_door_urdf(
        self,
        size_xyz,
        hinge_axis="z",
        limit_lower_rad=0.0,
        limit_upper_rad=math.pi / 2.0,
    ) -> str:
        """
        Create a temporary URDF for a door with one revolute joint ('hinge').

        The base link is an invisible frame anchored at the door jamb; the child link is the door leaf.
        The door leaf box is centered such that its hinge sits on the leaf's side edge.
        """
        sx, sy, sz = size_xyz  # full extents
        # Place the leaf so that hinge is on its -X edge and the leaf extends +X from hinge.
        # URDF box size uses full dimensions.
        rgba = f"{self.door_color[0]} {self.door_color[1]} {self.door_color[2]} {self.door_color[3]}"

        # joint axis
        axis_xyz = {"x": "1 0 0", "y": "0 1 0", "z": "0 0 1"}[hinge_axis]

        urdf = f"""<?xml version="1.0"?>
                <robot name="simple_door">
                <link name="frame"/>
                <joint name="hinge" type="revolute">
                    <parent link="frame"/>
                    <child link="leaf"/>
                    <!-- Hinge at origin; leaf origin offset places the slab to +X -->
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <axis xyz="{axis_xyz}"/>
                    <limit lower="{limit_lower_rad}" upper="{limit_upper_rad}" effort="200.0" velocity="5.0"/>
                </joint>

                <link name="leaf">
                    <inertial>
                    <mass value="8.0"/>
                    <origin xyz="{sx/2.0} 0 0" rpy="0 0 0"/>
                    <inertia ixx="2.0" iyy="2.0" izz="0.5" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                    <visual>
                    <origin xyz="{sx/2.0} 0 {sz/2.0}" rpy="0 0 0"/>
                    <geometry>
                        <box size="{sx} {sy} {sz}"/>
                    </geometry>
                    <material name="door">
                        <color rgba="{rgba}"/>
                    </material>
                    </visual>
                    <collision>
                    <origin xyz="{sx/2.0} 0 {sz/2.0}" rpy="0 0 0"/>
                    <geometry>
                        <box size="{sx} {sy} {sz}"/>
                    </geometry>
                    </collision>
                </link>
                </robot>
            """
        fd, path = tempfile.mkstemp(prefix="door_", suffix=".urdf")
        with os.fdopen(fd, "w") as f:
            f.write(urdf)
        self._tmp_urdf_path = path
        return path

    def spawn_door(
        self,
        base_pos,
        base_orn_rpy=(0.0, 0.0, 0.0),
        size_xyz=(0.9, 0.04, 2.0),
        initial_angle_deg=0.0,
        hinge_axis="z",
    ):
        """
        Spawn a single revolute-joint door leaf attached to an internal 'frame' link.
        The 'frame' is fixed to the world via useFixedBase=True.
        Returns: body_id
        """
        # 1) Build a valid URDF (frame --hinge--> leaf)
        urdf_path = self._generate_door_urdf(
            size_xyz=size_xyz,
            hinge_axis=hinge_axis,
            limit_lower_rad=0.0,
            limit_upper_rad=math.pi / 2.0,
        )  # writes to self._tmp_urdf_path
        base_orn = p.getQuaternionFromEuler(base_orn_rpy)

        # 2) Load with the frame fixed to world
        body_id = p.loadURDF(
            urdf_path,
            basePosition=base_pos,
            baseOrientation=base_orn,
            useFixedBase=True,  # <<< IMPORTANT: weld 'frame' to world
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.cid,
        )

        # 3) Set initial hinge angle (joint 0)
        p.resetJointState(body_id, 0, math.radians(initial_angle_deg), physicsClientId=self.cid)

        # 4) Mild damping for stability
        p.changeDynamics(body_id, -1, linearDamping=0.04, angularDamping=0.04, physicsClientId=self.cid)

        # 5) Expose & return id
        self.door_id = body_id
        return body_id

    def remove_door(self):
        if self.door_id is not None:
            try:
                p.removeBody(self.door_id, physicsClientId=self.cid)
            except Exception:
                pass
            self.door_id = None
        if self._tmp_urdf_path and os.path.exists(self._tmp_urdf_path):
            try:
                os.remove(self._tmp_urdf_path)
            except Exception:
                pass
            self._tmp_urdf_path = None

    # ---------- Door control ----------

    def set_door_angle(self, angle_rad: float, max_force=120.0):
        """Position-control the door to a specific angle (clamped to URDF limits)."""
        if self.door_id is None:
            return
        p.setJointMotorControl2(
            bodyUniqueId=self.door_id,
            jointIndex=self.door_joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=angle_rad,
            force=max_force,
            physicsClientId=self.cid,
        )

    def set_door_velocity(self, vel_rad_s: float, max_force=40.0):
        """Velocity-control the door (useful for interactive opening)."""
        if self.door_id is None:
            return
        p.setJointMotorControl2(
            bodyUniqueId=self.door_id,
            jointIndex=self.door_joint_index,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=vel_rad_s,
            force=max_force,
            physicsClientId=self.cid,
        )

    # ---------- convenience ----------

    def enable_room(self, enable: bool):
        if enable and not self.wall_ids:
            self.build_room()
        if not enable and self.wall_ids:
            self.clear_room()

    def enable_door(self, enable: bool):
        if enable and self.door_id is None:
            self.spawn_door()
        if not enable and self.door_id is not None:
            self.remove_door()
