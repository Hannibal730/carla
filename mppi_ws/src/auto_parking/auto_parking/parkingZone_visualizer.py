"""Compatibility wrapper for the former parking-zone visualizer module."""

from auto_parking.zone_scan import ParkingZoneVisualizer, ZoneScan, main

__all__ = ['ParkingZoneVisualizer', 'ZoneScan', 'main']


if __name__ == '__main__':
    main()
