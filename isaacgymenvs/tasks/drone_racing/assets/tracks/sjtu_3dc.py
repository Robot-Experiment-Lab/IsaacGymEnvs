from dataclasses import dataclass
from typing import List, Tuple

from isaacgym.gymapi import Gym, Sim, AssetOptions, Asset
from ..utils import TrackOptions
from ..utils.track_utils import create_track_asset
from ..utils.urdf_utils import random_cylinders_link
from ...waypoint import Waypoint


@dataclass
class TrackSjtu3dcOptions:
    file_name: str = "track_sjtu_3dc"
    track_options: TrackOptions = TrackOptions()
    asset_options: AssetOptions = AssetOptions()
    type_id: int = 0
    num_obstacles: int = 16


def create_track_sjtu_3dc(
    gym: Gym,
    sim: Sim,
    options: TrackSjtu3dcOptions,
) -> Tuple[Asset, List[Waypoint]]:
    wp = _define_wp(options.type_id)
    obs_links, obs_origins = _define_obs(options.num_obstacles)

    asset = create_track_asset(
        options.file_name,
        options.track_options,
        wp,
        obs_links,
        obs_origins,
        [False] * len(obs_links),
        options.asset_options,
        gym,
        sim,
    )
    return asset, wp


def _define_wp(type_id: int) -> List[Waypoint]:
    wp = []
    track_wps = [
        Waypoint(
            index=0,
            xyz=[4.0, 4.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=1,
            xyz=[8.0, 0.0, 2.0],
            rpy=[0.0, 0.0, -90.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=2,
            xyz=[5.0, -4.0, 1.0],
            rpy=[0.0, 0.0, 180.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=3,
            xyz=[1.0, -1.0, 1.0],
            rpy=[0.0, 0.0, 90.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
    ]
    if type_id == 0:
        wp = [
            Waypoint(
                index=0,
                xyz=[2.0, 2.0, 1.0],
                rpy=[0.0, 0.0, 45.0],
                length_y=1.0,
                length_z=1.0,
                gate=False,
            ),
            _mod_wp_id(track_wps[0], 1),
            _mod_wp_id(track_wps[1], 2),
            _mod_wp_id(track_wps[2], 3),
            _mod_wp_id(track_wps[3], 4),
        ]
    elif type_id == 1:
        wp = [
            Waypoint(
                index=0,
                xyz=[6.0, 2.0, 1.5],
                rpy=[0.0, 0.0, -45.0],
                length_y=1.0,
                length_z=1.0,
                gate=False,
            ),
            _mod_wp_id(track_wps[1], 1),
            _mod_wp_id(track_wps[2], 2),
            _mod_wp_id(track_wps[3], 3),
            _mod_wp_id(track_wps[0], 4),
        ]
    elif type_id == 2:
        wp = [
            Waypoint(
                index=0,
                xyz=[6.0, -2.0, 1.5],
                rpy=[0.0, 0.0, -100.0],
                length_y=1.0,
                length_z=1.0,
                gate=False,
            ),
            _mod_wp_id(track_wps[2], 1),
            _mod_wp_id(track_wps[3], 2),
            _mod_wp_id(track_wps[0], 3),
            _mod_wp_id(track_wps[1], 4),
        ]
    elif type_id == 3:
        wp = [
            Waypoint(
                index=0,
                xyz=[2.0, -2.0, 1.0],
                rpy=[0.0, 0.0, 135.0],
                length_y=1.0,
                length_z=1.0,
                gate=False,
            ),
            _mod_wp_id(track_wps[3], 1),
            _mod_wp_id(track_wps[0], 2),
            _mod_wp_id(track_wps[1], 3),
            _mod_wp_id(track_wps[2], 4),
        ]
    return wp


def _mod_wp_id(wp: Waypoint, id: int):
    return Waypoint(
        index=id,
        xyz=wp.xyz,
        rpy=wp.rpy,
        length_y=wp.length_y,
        length_z=wp.length_z,
        gate=wp.gate,
    )


def _define_obs(num_obstacles: int):
    links = []
    origins = []

    links.append(
        random_cylinders_link(
            "random_cylinders_0",
            num_obstacles // 4,
            [4.0, 3.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([3.0, 2.5, 1.0, 0.0, 0.0, 0.0])

    links.append(
        random_cylinders_link(
            "random_cylinders_1",
            num_obstacles // 4,
            [3.0, 4.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([6.5, 2.0, 1.0, 0.0, 0.0, 0.0])

    links.append(
        random_cylinders_link(
            "random_cylinders_2",
            num_obstacles // 4,
            [4.0, 4.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([6.0, -2.0, 1.0, 0.0, 0.0, 0.0])

    links.append(
        random_cylinders_link(
            "random_cylinders_3",
            num_obstacles // 4,
            [3.0, 5.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([2.5, -1.5, 1.0, 0.0, 0.0, 0.0])

    return links, origins
