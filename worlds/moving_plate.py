#!/usr/bin/env python3
"""Grassy field, PX4 Iris, and one paper plate following a smooth random path."""

if __package__:
    from ._support.simulation import main
else:
    from _support.simulation import main


if __name__ == "__main__":
    main(moving=True)
