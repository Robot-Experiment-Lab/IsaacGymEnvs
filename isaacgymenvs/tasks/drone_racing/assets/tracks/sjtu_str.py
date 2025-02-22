from dataclasses import dataclass
from typing import List, Tuple

from isaacgym.gymapi import Gym, Sim, AssetOptions, Asset
from ..utils import TrackOptions
from ..utils.track_utils import create_track_asset
from ..utils.urdf_utils import random_cylinders_link
from ...waypoint import Waypoint


@dataclass
class TrackSjtuStrOptions:
    file_name: str = "track_sjtu_str"
    track_options: TrackOptions = TrackOptions()
    asset_options: AssetOptions = AssetOptions()
    num_obstacles: int = 12


def create_track_sjtu_str(
    gym: Gym,
    sim: Sim,
    options: TrackSjtuStrOptions,
) -> Tuple[Asset, List[Waypoint]]:
    wp = _define_wp()
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


def _define_wp() -> List[Waypoint]:
    return [
        Waypoint(
            index=0,
            xyz=[1.0, 0.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=False,
        ),
        Waypoint(
            index=1,
            xyz=[3.0, 0.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=2,
            xyz=[6.0, -1.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=3,
            xyz=[9.0, 1.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=4,
            xyz=[12.0, 0.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
        Waypoint(
            index=5,
            xyz=[14.0, 0.0, 1.0],
            rpy=[0.0, 0.0, 0.0],
            length_y=1.0,
            length_z=1.0,
            gate=True,
        ),
    ]


def _define_obs(num_obstacles: int):
    links = []
    origins = []

    links.append(
        random_cylinders_link(
            "random_cylinders_0",
            num_obstacles // 3,
            [1.5, 2.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([0.0, 8.5 / 2, 1.0, 0.0, 0.0, 0.0])

    links.append(
        random_cylinders_link(
            "random_cylinders_1",
            num_obstacles // 3,
            [1.5, 2.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([0.0, 14.5 / 2, 1.0, 0.0, 0.0, 0.0])

    links.append(
        random_cylinders_link(
            "random_cylinders_2",
            num_obstacles // 3,
            [1.5, 2.0, 0.0],
            [0.0, 0.0, 0.0],
            0.1,
            0.15,
            3.0,
            3.0,
        )
    )
    origins.append([0.0, 20.5 / 2, 1.0, 0.0, 0.0, 0.0])

    return links, origins
