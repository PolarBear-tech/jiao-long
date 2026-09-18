"""Configuration for the unknown-map exercise."""

from copy import deepcopy

from ex import CONFIG as _EX_CONFIG


CONFIG = deepcopy(_EX_CONFIG)
CONFIG["pyrobo"]["map"]["name"] = "unknown"
CONFIG["pyrobo"]["map"]["real_name"] = "unknown_real"
