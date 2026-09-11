"""Manual smoke test for the NavigationQuery/Path API.

Usage:
    python scripts/try_navigation.py MMAPS_DIR MAP_ID X1 Y1 Z1 X2 Y2 Z2

Example:
    python scripts/try_navigation.py "C:\\Games\\Wow-3.3.5 FR\\mmaps" 0 ^
        4900.0 -4200.0 -500.0 5025.0 -3825.0 -500.0
"""

import math
import sys

import wow_navmesh as wn

STEP_SIZE = 5.0  # one moveAlongSurface() call = one bounded movement step (e.g. one
                 # tick at walking speed), not a teleport straight to a distant corner.


def main():
    if len(sys.argv) != 9:
        print(__doc__)
        raise SystemExit(1)

    mmaps_dir, map_id, x1, y1, z1, x2, y2, z2 = sys.argv[1:]
    start = (float(x1), float(y1), float(z1))
    end = (float(x2), float(y2), float(z2))

    nm = wn.NavMesh(mmaps_dir)
    nm.load_map(int(map_id))
    q = nm.query

    print("=== find_nearest_poly ===")
    nearest_start = q.find_nearest_poly(start)
    nearest_end = q.find_nearest_poly(end)
    print(f"start -> found={nearest_start.found} poly_ref={nearest_start.poly_ref} "
          f"snapped={nearest_start.position} distance={nearest_start.distance:.2f}")
    print(f"end   -> found={nearest_end.found} poly_ref={nearest_end.poly_ref} "
          f"snapped={nearest_end.position} distance={nearest_end.distance:.2f}")

    print("\n=== find_path (Path) ===")
    path = q.find_path(start, end)
    print(f"status={path.status} is_valid={path.is_valid()} is_complete={path.is_complete()}")
    print(f"start_poly={path.start_poly} end_poly={path.end_poly}")
    print(f"actual_end_position={path.actual_end_position}")
    print(f"polygon_corridor ({len(path.polygon_corridor)} polys): {path.polygon_corridor}")
    print(f"straight_path ({len(path.straight_path)} points):")
    for p in path.straight_path:
        print(f"  {p.position}  flags={p.flags:#04x}  poly_ref={p.poly_ref}")

    if not path.is_valid():
        print("\nNo usable path -- stopping here.")
        return

    print("\n=== steering loop (get_steer_target) ===")
    position = path.start_position
    current_poly = path.start_poly  # updated below to wherever we actually end up
    max_steps = 2000
    for step in range(max_steps):
        steer = path.get_steer_target(position)
        if steer is None:
            print("  no steer target returned; stopping")
            break
        if steer.reached:
            print(f"  step {step}: reached destination at {position}")
            break

        # Library only tells us *where* to go -- we still have to move there ourselves.
        # A single moveAlongSurface() call is one bounded movement step (like one tick
        # at walking speed), not a teleport straight to a possibly-distant steer corner,
        # so we clamp the desired displacement to STEP_SIZE before calling it.
        dx = steer.position[0] - position[0]
        dy = steer.position[1] - position[1]
        dz = steer.position[2] - position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        scale = min(1.0, STEP_SIZE / dist) if dist > 1e-6 else 1.0
        desired = (position[0] + dx * scale, position[1] + dy * scale, position[2] + dz * scale)

        moved = q.move_along_surface(position, desired, current_poly)
        position = moved.position
        if moved.visited:
            current_poly = moved.visited[-1]  # poly the new position actually landed in

        if step % 10 == 0:
            print(f"  step {step}: position={position} steer_dist={steer.distance:.2f}")
    else:
        print(f"  stopped after {max_steps} steps without reaching the end")

    print("\n=== raycast (straight line start -> end) ===")
    ray = q.raycast(start, end)
    if ray.hit:
        print(f"hit=True t={ray.t:.3f} position={ray.position} normal={ray.normal}")
    else:
        print("hit=False (clear line of sight along the navmesh surface)")

    print("\n=== find_distance_to_wall ===")
    max_radius = 20.0
    wall = q.find_distance_to_wall(start, max_radius=max_radius)
    if wall.distance < 0:
        print("no polygon found near this position")
    elif wall.distance >= max_radius:
        print(f"no wall within {max_radius} units")
    else:
        print(f"distance={wall.distance:.2f} position={wall.position} normal={wall.normal}")

    nm.free_map()


if __name__ == "__main__":
    main()
