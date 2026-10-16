"""Declared aircraft performance profiles used by v2 planning."""

from .models import AircraftProfile


DJI_MINI_4_PRO = AircraftProfile()
DEFAULT_PROFILE_ID = DJI_MINI_4_PRO.id


def default_profile() -> AircraftProfile:
    """Return an independent profile; engineering values are explicitly named."""
    return DJI_MINI_4_PRO.model_copy(deep=True)


def resolve_profile(profile_id: str, profiles: list[AircraftProfile] | None = None) -> AircraftProfile:
    for profile in profiles or []:
        if profile.id == profile_id:
            return profile.model_copy(deep=True)
    if profile_id == DEFAULT_PROFILE_ID:
        return default_profile()
    raise ValueError(f"Unknown aircraft profile: {profile_id}")
