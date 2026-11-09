#!/usr/bin/env python3
"""Grassy field, PX4 Iris, and one stationary marked paper plate."""

if __package__:
    from ._support.simulation import main
else:
    from _support.simulation import main


if __name__ == "__main__":
    main(moving=False)
