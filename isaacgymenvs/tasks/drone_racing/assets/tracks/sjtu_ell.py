from dataclasses import dataclass
from typing import List, Tuple

from isaacgym.gymapi import Gym, Sim, AssetOptions, Asset
from ..utils import TrackOptions
from ..utils.track_utils import create_track_asset
from ...waypoint import Waypoint


@dataclass
class TrackSjtuEllOptions:
    file_name: str = "track_sjtu_ell"
    track_options: TrackOptions = TrackOptions()
    asset_options: AssetOptions = AssetOptions()
    type_id: int = 0


def create_track_sjtu_ell(
    gym: Gym,
    sim: Sim,
    options: TrackSjtuEllOptions,
) -> Tuple[Asset, List[Waypoint]]:
    wp = _define_wp(options.type_id)
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


def _define_wp(type_id: int) -> List[Waypoint]:
    wp = []
    if type_id == 0:
        pass
    elif type_id == 1:
        pass
    elif type_id == 2:
        pass
    elif type_id == 3:
        pass
    return wp


def _define_obs():
    links = []
    origins = []

    return links, origins
