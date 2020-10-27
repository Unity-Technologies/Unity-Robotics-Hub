class Error(Exception):
    """Base class for other exceptions"""
    pass


class TopicOrServiceNameDoesNotExistError(Error):
    """The topic or service name passed does not exist in the source destination dictionary."""
    pass
