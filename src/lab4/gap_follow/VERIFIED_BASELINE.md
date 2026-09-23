# Verified tuning baseline

This file preserves the known-good parameters before further speed tuning.

## `levine_blocked`

```python
{'max_speed': 1.4}
```

- Start: `(-12.0, 0.0, 0.0)`
- Result: three consecutive laps, no collision
- Lap times: 73.66 s, 73.12 s, 71.18 s

## `levine_obs`

```python
{
    'bubble_radius': 0.22,
    'max_steering': 0.40,
    'stop_distance': 0.28,
}
```

- Start: `(9.96, 2.8, 1.5708)`
- Result: one complete timed lap, no collision
- Lap time: 96.78 s

The controller defaults used with both parameter sets are recorded in
`scripts/reactive_node.py`. Restore the dictionaries above to the corresponding
launch files if a later tuning candidate is not as reliable.

## Later verified candidate

- `levine_blocked`, `max_speed: 1.8`: three laps without collision;
  59.16 s, 59.62 s, 60.40 s.
- `levine_blocked`, `max_speed: 2.2`: three laps without collision;
  53.45 s, 51.40 s, 51.53 s.
- `levine_blocked`, `max_speed: 2.6`: three laps without collision;
  47.56 s, 47.52 s, 46.31 s.
- `levine_blocked`, `max_speed: 3.0`: three laps without collision;
  45.09 s, 42.14 s, 44.53 s.
- `levine_obs`, `max_speed: 1.4`: one complete timed lap without collision;
  90.91 s.

## 15 Hz gap-directed speed candidate

The empty-map controller was retested after matching the simulator scan rate
to the physical LiDAR's 15 Hz limit. Normal speed is based on clearance around
the selected gap target; the narrow forward sector is retained only as an
emergency stop check. Tight-corner speeds retain the previously verified
3 m/s tuning while near-straight sections can use the 5 m/s maximum.

```python
{
    'clearance_speed_gain': 1.0,
    'corner_speed_reference': 3.0,
    'max_speed': 5.0,
    'min_speed': 0.4,
    'turn_slowdown': 0.65,
}
```

- Map: `levine_blocked`, counter-clockwise from `(-12.0, 0.0, 0.0)`.
- Result: three consecutive laps, no collision or scan stall.
- Lap times: 33.92 s, 34.27 s, 33.17 s.
- Mean commanded speed: 2.14 m/s; maximum: 5.00 m/s.
- Mean commanded speed while `abs(steering) >= 0.10`: 1.65 m/s.
