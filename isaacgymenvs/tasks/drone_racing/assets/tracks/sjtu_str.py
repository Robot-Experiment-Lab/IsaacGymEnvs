from dataclasses import dataclass
from typing import List, Tuple

from isaacgym.gymapi import Gym, Sim, AssetOptions, Asset
from ..utils import TrackOptions
from ..utils.track_utils import create_track_asset
from ...waypoint import Waypoint


@dataclass
class TrackSjtuStrOptions:
    file_name: str = "track_sjtu_str"
    track_options: TrackOptions = TrackOptions()
    asset_options: AssetOptions = AssetOptions()


def create_track_sjtu_str(
    gym: Gym,
    sim: Sim,
    options: TrackSjtuStrOptions,
) -> Tuple[Asset, List[Waypoint]]:
    wp = _define_wp()
    obs_links, obs_origins = _define_obs()
    
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


def _define_obs():
    links = []
    origins = []

    return links, origins