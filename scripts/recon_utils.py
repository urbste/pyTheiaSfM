"""
Helpers when using a filtered reconstruction (e.g. after CreateEstimatedSubreconstruction).

TrackId / ViewId values are NOT remapped: any track or view that survives keeps its
original id. Removed entries are absent from the maps, so old ids become invalid.

If localization still uses 2D–3D correspondences or a retrieval index built on the *full*
reconstruction, many track_ids will point to removed tracks → AddObservation will CHECK-fail.

Fix: rebuild the image retrieval / match database from the same .recon you load, or filter
correspondences to track_id in track_id_set(reconstruction) before AddObservation.

Alternative: keep the full .recon for localization and use the slim file only for
alignment / storage.
"""

from __future__ import annotations


def track_id_set(reconstruction) -> set:
    """Set of TrackId in this reconstruction (valid targets for AddObservation)."""
    return set(reconstruction.TrackIds())


def view_id_set(reconstruction) -> set:
    """Set of ViewId in this reconstruction."""
    return set(reconstruction.ViewIds())


def has_track(reconstruction, track_id: int) -> bool:
    """True if this track exists (was not removed). Uses O(1) map lookup."""
    return reconstruction.Track(track_id) is not None


def has_view(reconstruction, view_id: int) -> bool:
    """True if this view exists."""
    return reconstruction.View(view_id) is not None
