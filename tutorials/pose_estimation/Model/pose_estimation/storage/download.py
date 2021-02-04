import logging
import zlib
from pathlib import Path

import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

from .exceptions import ChecksumError, DownloadError

logger = logging.getLogger(__name__)

# Timeout of requests (in seconds)
DEFAULT_TIMEOUT = 1800
# Retry after failed request
DEFAULT_MAX_RETRIES = 5


class TimeoutHTTPAdapter(HTTPAdapter):
    def __init__(self, timeout, *args, **kwargs):
        self.timeout = timeout
        super().__init__(*args, **kwargs)

    def send(self, request, **kwargs):
        kwargs["timeout"] = self.timeout
        return super().send(request, **kwargs)


def download_file(source_uri: str, dest_path: str, use_cache: bool = True):
    """Download a file specified from a source uri

    Args:
        source_uri (str): source url where the file should be downloaded
        dest_path (str): destination path of the file
        use_cache (bool): use_cache (bool): use cache instead of
        re-download if file exists

    Returns:
        String of destination path.
    """
    dest_path = Path(dest_path)
    if dest_path.exists() and use_cache:
        return dest_path

    logger.debug(f"Trying to download file from {source_uri} -> {dest_path}")
    adapter = TimeoutHTTPAdapter(
        timeout=DEFAULT_TIMEOUT, max_retries=Retry(total=DEFAULT_MAX_RETRIES)
    )
    with requests.Session() as http:
        http.mount("https://", adapter)
        try:
            response = http.get(source_uri)
            response.raise_for_status()
        except requests.exceptions.RequestException as ex:
            logger.error(ex)
            err_msg = (
                f"The request download from {source_uri} -> {dest_path} can't "
                f"be completed."
            )

            raise DownloadError(err_msg)
        else:
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            with open(dest_path, "wb") as f:
                f.write(response.content)

    return dest_path


def validate_checksum(filepath, expected_checksum, algorithm="CRC32"):
    """Validate checksum of the downloaded file.

    Args:
        filepath (str): the doaloaded file path
        expected_checksum (int): expected checksum of the file
        algorithm (str): checksum algorithm. Defaults to CRC32

    Raises:
        ChecksumError if the file checksum does not match.
    """
    computed = compute_checksum(filepath, algorithm)
    if computed != expected_checksum:
        raise ChecksumError


def compute_checksum(filepath, algorithm="CRC32"):
    """Compute the checksum of a file.

    Args:
        filepath (str): the doaloaded file path
        algorithm (str): checksum algorithm. Defaults to CRC32

    Returns:
        int: the checksum value
    """
    if algorithm == "CRC32":
        chs = _crc32_checksum(filepath)
    else:
        raise ValueError("Unsupported checksum algorithm!")

    return chs


def _crc32_checksum(filepath):
    """Calculate the checksum of a file using CRC32."""
    with open(filepath, "rb") as f:
        checksum = zlib.crc32(f.read())

    return checksum
