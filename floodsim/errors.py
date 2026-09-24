class FloodSimError(Exception):
    """A user-facing error: printed as `error: <message>` by the CLI."""


class BuildError(FloodSimError):
    """The native build (cmake configure or build) failed."""
