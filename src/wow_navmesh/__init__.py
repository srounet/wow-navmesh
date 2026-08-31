from importlib.metadata import PackageNotFoundError, version

from ._wow_navmesh import NavMesh

try:
    __version__ = version("wow-navmesh")
except PackageNotFoundError:  # pragma: no cover - package not installed (e.g. running from source)
    __version__ = "0.0.0"

__all__ = ["NavMesh", "__version__"]
