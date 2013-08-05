import re


def is_valid_version(version):
    """Check if `version` is a valid version according to
    http://ros.org/reps/rep-0127.html#version
    """
    match = re.match(r'^\d+\.\d+\.\d+$', version)
    return match is not None
