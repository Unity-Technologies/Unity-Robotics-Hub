class DownloadError(Exception):
    """Raise when download file failed."""


class ChecksumError(Exception):
    """Raises when the downloaded file checksum is not correct."""
