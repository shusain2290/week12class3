"""
CDS215 — TREASURE HUNT (Game 3)

Drive the BLUE Husky to collect FIVE gold treasures for 10 points each.
Avoid up to THREE red/orange guards: each patrols independently until it
gets close enough to chase you. Collect all 50 points before time runs out.
Set GUARD_COUNT = 1, 2, or 3 at the top to choose the difficulty.

Requires: pip install pybullet
Run: python CDS215_Treasure_Hunt_Student.py
Controls: W/Up = forward, S/Down = reverse, A/Left = steer left,
          D/Right = steer right, R = restart, Q = quit.

Students independently complete TASKS 1-4 in update_mission() (Part 5).
All robot driving, guard AI, physics, obstacles, and graphics are provided.
Optional no-GUI rule check: python filename.py --smoke-test
"""

import math
import sys
import time
import pybullet as p
import pybullet_data


# =====================================================================
# PART 1 — GAME SETTINGS: STUDENTS MAY EXPERIMENT WITH THESE
# =====================================================================
MISSION_SECONDS = 110.0            # Collect all treasures in simulated seconds.
PLAYER_SPEED = 10.0                # Intentionally harder than a balanced game.
PLAYER_TURN_SPEED = 3.0
HUNTER_SPEED = 11.5               # The RED guard is faster initially.
HUNTER_TURN_SPEED = 4.5
PLAYER_START = (-7.1, -6.8)
HUNTER_START = (0.4, 1.5)       # Original guard's starting location.
GUARD_COUNT = 3                   # 1 = normal, 2 = difficult, 3 = CHAOS!
# New guards begin in separate parts of the arena (away from walls and boxes).
GUARD_STARTS = [HUNTER_START, (-6.2, 3.2), (6.3, -2.3)]
GUARD_COLORS = [(1.0, 0.16, 0.18), (1.0, 0.42, 0.12), (0.93, 0.14, 0.45)]
GUARD_PATROL_STARTS = [0, 2, 1]   # Different patrol phases, not a robot convoy.

# Five GOLD treasures: the student must visit each location once.
TREASURE_SPOTS = [
    (-5.8, 5.6),
    (-0.4, 6.3),
    (5.8, 5.5),
    (6.35, -6.05),
    (-1.7, -5.9),
]
TREASURE_POINTS = 10
TREASURE_RADIUS = 1.0           # Close enough to pick up a coin.
GUARD_ALERT_RADIUS = 3.7        # Guard starts chasing when player is this close.
GUARD_ALERT_SECONDS = 4.0       # Continues chasing briefly after losing contact.
PATROL_POINTS = [(-1.4, 2.8), (3.0, 2.6), (3.2, -2.9), (-1.8, -2.8)]
PATROL_RADIUS = 0.85
GUARD_PATROL_SPEED_FACTOR = 0.65

# These settings affect PERFORMANCE and PLAYBACK, not motor speed ratios.
PHYSICS_HZ = 120                # Increasing Hz increases physics work.
RAY_DRAW_HZ = 10                # Visual drawing only; sensor checks run every step.
MAX_PLAYBACK_SPEED = 1.0       # 1x is easiest for keyboard controls.
SHOW_HUNTER_RAYS = True
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 750

# Prewritten guard sensor and obstacle-avoidance settings.
ARENA_HALF_SIZE = 9.0
WALL_THICKNESS = 0.6
HUNTER_SENSOR_RANGE = 2.6
HUNTER_SENSOR_ANGLE = math.radians(12)
HUNTER_STOP_DISTANCE = 1.5
HUNTER_SIDE_DISTANCE = 1.15
HUNTER_SENSOR_OFFSET = 0.72
HUNTER_CORNER_OFFSET = 0.36
HUNTER_AVOID_TURN_SECONDS = 0.55
HUNTER_ADVANCE_SECONDS = 1.2
HUNTER_ADVANCE_SPEED_FACTOR = 0.70
HUNTER_ADVANCE_FRONT_CLEARANCE = 0.70
HUNTER_ADVANCE_SIDE_CLEARANCE = 0.35
HUNTER_STUCK_CHECK_SECONDS = 2.0
HUNTER_STUCK_MIN_MOVEMENT = 0.30
HUNTER_REVERSE_SECONDS = 0.95
HUNTER_RECOVERY_SPIN_SECONDS = 1.10
HUNTER_RECOVERY_ADVANCE_SECONDS = 1.5
SENSOR_HEIGHT = 0.38
LEFT_WHEELS = (2, 4)
RIGHT_WHEELS = (3, 5)

# Each obstacle: (x, y, full width, full depth, RGB color).
# Obstacles deliberately cover different portions of the arena.
OBSTACLES = [
    (-3.0, 1.2, 2.6, 1.2, (0.15, 0.75, 0.95)),
    (1.1, -2.1, 3.0, 1.25, (0.95, 0.62, 0.15)),
    (3.5, 2.8, 1.3, 2.8, (0.68, 0.34, 0.91)),
    (-4.1, -3.3, 1.6, 1.5, (0.94, 0.74, 0.18)),
    (-0.6, 4.1, 2.5, 1.2, (0.40, 0.88, 0.54)),
    (4.3, -4.1, 1.7, 1.7, (0.91, 0.41, 0.61)),
    (-6.3, 0.1, 1.2, 2.2, (0.25, 0.65, 0.85)),
    (6.0, -0.5, 1.2, 2.1, (0.95, 0.47, 0.52)),
]

SMOKE_TEST = "--smoke-test" in sys.argv
if type(GUARD_COUNT) is not int or GUARD_COUNT not in (1, 2, 3):
    raise ValueError("GUARD_COUNT must be 1, 2, or 3")
if not isinstance(PHYSICS_HZ, int) or PHYSICS_HZ < 30:
    raise ValueError("PHYSICS_HZ must be an integer of at least 30")
if not isinstance(RAY_DRAW_HZ, int) or RAY_DRAW_HZ < 1:
    raise ValueError("RAY_DRAW_HZ must be an integer of at least 1")
if MAX_PLAYBACK_SPEED < 0:
    raise ValueError("MAX_PLAYBACK_SPEED cannot be negative")
if MISSION_SECONDS <= 0 or TREASURE_POINTS <= 0 or TREASURE_RADIUS <= 0:
    raise ValueError("Time, treasure points and pickup radius must be positive")
if len(TREASURE_SPOTS) != 5:
    raise ValueError("The Treasure Hunt needs exactly five treasures")


# =====================================================================
# PART 2 — REUSABLE ROBOT COMMANDS (FROM OUR PREVIOUS HUSKY LABS)
# =====================================================================
_last_commands = {}


def set_wheels(robot_id, left_speed, right_speed):
    """Only send a command if it changes (our motor optimization)."""
    speeds = (round(left_speed, 4), round(right_speed, 4))
    if _last_commands.get(robot_id) == speeds:
        return
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left_speed, force=100)
    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right_speed, force=100)
    _last_commands[robot_id] = speeds


def stop_robot(robot_id):
    set_wheels(robot_id, 0, 0)


def robot_xy_heading(robot_id):
    position, rotation = p.getBasePositionAndOrientation(robot_id)
    return position[0], position[1], p.getEulerFromQuaternion(rotation)[2]


def angle_difference(target_angle, current_angle):
    return math.atan2(math.sin(target_angle - current_angle),
                      math.cos(target_angle - current_angle))


def pressed(keys, *codes):
    return any(keys.get(code, 0) & p.KEY_IS_DOWN for code in codes)


def player_teleop(robot_id, keys):
    """WASD / arrow controls; holding forward and turn at once makes an arc."""
    up = getattr(p, "B3G_UP_ARROW", -100)
    down = getattr(p, "B3G_DOWN_ARROW", -101)
    left_arrow = getattr(p, "B3G_LEFT_ARROW", -102)
    right_arrow = getattr(p, "B3G_RIGHT_ARROW", -103)

    forward = pressed(keys, ord('w'), ord('W'), up)
    backward = pressed(keys, ord('s'), ord('S'), down)
    left = pressed(keys, ord('a'), ord('A'), left_arrow)
    right = pressed(keys, ord('d'), ord('D'), right_arrow)

    drive = (int(forward) - int(backward)) * PLAYER_SPEED
    turn = (int(left) - int(right)) * PLAYER_TURN_SPEED
    set_wheels(robot_id, drive - turn, drive + turn)


# =====================================================================
# PART 3 — ROBOT COLORS, ARENA, AND SCENE SETUP
# =====================================================================
def make_box(x, y, width, depth, color, height=1.2):
    half = [width / 2, depth / 2, height / 2]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                 rgbaColor=[*color, 1])
    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, height / 2])


def tint_husky(robot_id, rgb):
    """Give the two otherwise-identical Husky models distinct team colors."""
    for link in [-1] + list(range(p.getNumJoints(robot_id))):
        try:
            p.changeVisualShape(robot_id, link, rgbaColor=[*rgb, 1])
        except p.error:
            pass  # Some URDF links have no visible shape.


def create_arena():
    size = ARENA_HALF_SIZE
    thick = WALL_THICKNESS
    blue_gray = (0.28, 0.35, 0.48)
    make_box(0, size, size * 2 + thick, thick, blue_gray)
    make_box(0, -size, size * 2 + thick, thick, blue_gray)
    make_box(size, 0, thick, size * 2 + thick, blue_gray)
    make_box(-size, 0, thick, size * 2 + thick, blue_gray)
    for x, y, w, d, color in OBSTACLES:
        make_box(x, y, w, d, color)


def make_husky(start, rgb):
    rid = p.loadURDF("husky/husky.urdf", [start[0], start[1], 0.22])
    tint_husky(rid, rgb)
    return rid


# =====================================================================
# PART 4 — INDEPENDENT RED GUARDS: PATROL, CHASE, AVOID
# =====================================================================
# Each guard patrols fixed points until it sees the student.
# Guards then chase the student's LIVE coordinates independently.
# Students do not modify the navigation or obstacle avoidance below.

_ray_ids = {}  # One set of three reusable debug lines PER guard.


def hunter_sensor(hunter_id, player_id, x, y, heading, angle, step):
    """Return the distance to the nearest wall/box using a front-facing ray.

    The center ray originates at the nose. Left/right rays originate near
    the front corners and fan slightly outward to protect the full width.
    The player is not an obstacle to avoid because catching it is the goal.
    Detection runs EVERY physics step; visuals refresh less frequently.
    """
    # The center ray starts at the nose. The two side rays start at the
    # FRONT CORNERS of the Husky instead of on the 35-degree diagonal.
    # "side" is signed: + means left, - means right, 0 means center.
    side = HUNTER_CORNER_OFFSET if angle > 0 else (
        -HUNTER_CORNER_OFFSET if angle < 0 else 0.0
    )
    start = [x + HUNTER_SENSOR_OFFSET * math.cos(heading) - side * math.sin(heading),
             y + HUNTER_SENSOR_OFFSET * math.sin(heading) + side * math.cos(heading),
             SENSOR_HEIGHT]
    direction = heading + angle
    end = [start[0] + HUNTER_SENSOR_RANGE * math.cos(direction),
           start[1] + HUNTER_SENSOR_RANGE * math.sin(direction), SENSOR_HEIGHT]
    hit_id, _, fraction, _, _ = p.rayTest(start, end)[0]
    # Treat the player as clear so the enemy runs into the player to win.
    is_wall = hit_id >= 0 and hit_id not in (hunter_id, player_id)
    distance = max(0.0, fraction * HUNTER_SENSOR_RANGE) if is_wall else HUNTER_SENSOR_RANGE

    draw_interval = max(1, round(PHYSICS_HZ / RAY_DRAW_HZ))
    if SHOW_HUNTER_RAYS and not SMOKE_TEST and step % draw_interval == 0:
        label = "center" if angle == 0 else ("left" if angle > 0 else "right")
        rgb = [1, 0.15, 0.1] if is_wall else [0.1, 0.9, 0.2]
        ray_key = (hunter_id, label)
        _ray_ids[ray_key] = p.addUserDebugLine(
            start, end, rgb, lineWidth=2,
            replaceItemUniqueId=_ray_ids.get(ray_key, -1))
    return distance


def update_hunter(hunter_id, player_id, game, guard, target_xy=None, speed_factor=1.0):
    """Navigate independently; each guard has its own state dictionary."""
    hunter_x, hunter_y, hunter_heading = robot_xy_heading(hunter_id)
    if target_xy is None:
        target_x, target_y, _ = robot_xy_heading(player_id)  # CHASE moving player!
    else:
        target_x, target_y = target_xy                 # PATROL fixed waypoint.
    target_angle = math.atan2(target_y - hunter_y, target_x - hunter_x)
    error = angle_difference(target_angle, hunter_heading)
    step = game["step"]

    # Detect a physically trapped hunter, including when it switches between
    # AVOID and ADVANCE without actually moving around the box. Previously
    # this check only ran during CHASE, so that loop could last indefinitely.
    if step - guard["anchor_step"] >= round(HUNTER_STUCK_CHECK_SECONDS * PHYSICS_HZ):
        anchor_x, anchor_y = guard["anchor"]
        moved = math.hypot(hunter_x - anchor_x, hunter_y - anchor_y)
        to_target = math.hypot(target_x - hunter_x, target_y - hunter_y)
        if (moved < HUNTER_STUCK_MIN_MOVEMENT
                and to_target > 0.75
                and guard["mode"] in ("chase", "avoid", "advance")):
            old_mode = guard["mode"]
            guard["mode"] = "reverse"
            guard["mode_until"] = step + round(HUNTER_REVERSE_SECONDS * PHYSICS_HZ)
            guard["recover_direction"] *= -1  # Alternate to escape dead ends.
            if not SMOKE_TEST:
                print(f"GUARD {guard['number']} STUCK in {old_mode.upper()}: moved {moved:.2f} m "
                      f"in {HUNTER_STUCK_CHECK_SECONDS:.1f}s. Reversing and turning.")
        guard["anchor"] = (hunter_x, hunter_y)
        guard["anchor_step"] = step

    if guard["mode"] == "reverse":
        if step < guard["mode_until"]:
            set_wheels(hunter_id, -HUNTER_SPEED * 0.6, -HUNTER_SPEED * 0.6)
            return
        guard["mode"] = "spin"
        guard["mode_until"] = step + round(HUNTER_RECOVERY_SPIN_SECONDS * PHYSICS_HZ)

    if guard["mode"] == "spin":
        if step < guard["mode_until"]:
            d = guard["recover_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do not point straight at the player again after getting unstuck;
        # commit to moving away from the box before returning to CHASE.
        guard["mode"] = "advance"
        guard["mode_until"] = step + round(HUNTER_RECOVERY_ADVANCE_SECONDS * PHYSICS_HZ)
        guard["avoid_direction"] = guard["recover_direction"]
        guard["anchor"] = (hunter_x, hunter_y)
        guard["anchor_step"] = step

    if guard["mode"] == "avoid":
        if step < guard["mode_until"]:
            d = guard["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        # Do NOT immediately aim back at the player. First get past the box.
        guard["mode"] = "advance"
        guard["mode_until"] = step + round(HUNTER_ADVANCE_SECONDS * PHYSICS_HZ)

    # Sensors monitor the path every step. Drawing the rays is optional.
    front = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, 0, step)
    left = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                         hunter_heading, HUNTER_SENSOR_ANGLE, step)
    right = hunter_sensor(hunter_id, player_id, hunter_x, hunter_y,
                          hunter_heading, -HUNTER_SENSOR_ANGLE, step)

    # Finish the maneuver with a committed forward move. We intentionally
    # ignore the usual EARLY-warning distances during this short stage: they
    # may still see the box alongside the Husky. But all three rays remain
    # active, and emergency clearances can interrupt forward travel.
    if guard["mode"] == "advance":
        too_close = (front < HUNTER_ADVANCE_FRONT_CLEARANCE
                     or left < HUNTER_ADVANCE_SIDE_CLEARANCE
                     or right < HUNTER_ADVANCE_SIDE_CLEARANCE)
        if too_close:
            guard["mode"] = "avoid"
            # Maintain the chosen detour direction rather than oscillating.
            guard["mode_until"] = step + round(0.32 * PHYSICS_HZ)
            d = guard["avoid_direction"] * HUNTER_TURN_SPEED
            set_wheels(hunter_id, -d, d)
            return
        if step < guard["mode_until"]:
            speed = HUNTER_SPEED * HUNTER_ADVANCE_SPEED_FACTOR * speed_factor
            set_wheels(hunter_id, speed, speed)
            return
        guard["mode"] = "chase"

    # During the normal chase, ANY ray can trigger a full avoidance maneuver.
    if (front < HUNTER_STOP_DISTANCE
            or left < HUNTER_SIDE_DISTANCE
            or right < HUNTER_SIDE_DISTANCE):
        guard["mode"] = "avoid"
        guard["avoid_direction"] = 1 if left >= right else -1
        guard["mode_until"] = step + round(HUNTER_AVOID_TURN_SECONDS * PHYSICS_HZ)
        d = guard["avoid_direction"] * HUNTER_TURN_SPEED
        set_wheels(hunter_id, -d, d)
        return

    # Pure pursuit: steer toward the player's current x/y coordinates.
    # Slower turning creates opportunities for the student to outmaneuver it.
    if abs(error) > 0.72:
        turn = HUNTER_TURN_SPEED if error > 0 else -HUNTER_TURN_SPEED
        set_wheels(hunter_id, -turn, turn)
    else:
        turn = max(-HUNTER_TURN_SPEED, min(HUNTER_TURN_SPEED, error * 5.0))
        drive = HUNTER_SPEED * speed_factor * (0.78 if abs(error) > 0.38 else 1.0)
        set_wheels(hunter_id, drive - turn, drive + turn)



# =====================================================================
# PART 5 — TREASURE PICKUP RULES: THE STUDENT CODING CHALLENGE
# =====================================================================
# Markers are visual only (no collision), so the Husky can drive over them.
def make_treasure(position, number):
    x, y = position
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.44,
                                 length=0.065, rgbaColor=[1.0, 0.78, 0.05, 1.0])
    body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, 0.07])
    label = p.addUserDebugText(f"${number}", [x - 0.32, y + 0.35, 0.75],
                               textColorRGB=[1.0, 0.9, 0.0], textSize=1.35)
    return body, label


def celebrate_treasure(game, treasure_index):
    """Prewritten helper: remove the collected coin and announce the score."""
    body = game["treasure_bodies"][treasure_index]
    label = game["treasure_labels"][treasure_index]
    if body is not None:
        p.removeBody(body)
        game["treasure_bodies"][treasure_index] = None
    if label is not None:
        if label >= 0:
            p.removeUserDebugItem(label)
        game["treasure_labels"][treasure_index] = None
    print(f"TREASURE {treasure_index + 1} collected! Score: {game['score']} "
          f"/ {len(TREASURE_SPOTS) * TREASURE_POINTS}")


def update_mission(player_id, game):
    """Student challenge: complete four tasks without changing other functions."""
    if game["result"] != "PLAYING":
        return  # The mission has already ended.
    # Your Husky's current X and Y coordinates are supplied here.
    # The third value is heading, which we do not need for treasure pickup.
    player_x, player_y, _ = robot_xy_heading(player_id)

    # ==================================================================
    # INDEPENDENT CODING CHALLENGE — COMPLETE TASKS 1–4
    # ==================================================================
    # The movement controls, guard behavior, collision detection,
    # and treasure graphics are already programmed.
    #
    # Your job is to complete the Python statements below.
    # Use your previous notebooks and Python notes for help.
    #
    # IMPORTANT:
    # Some statements contain temporary values such as False, 0,
    # or pass. Replace these with working Python code.


    # ------------------------------------------------------------------
    # TASK 1 — CALCULATE DISTANCE TO EACH TREASURE
    # ------------------------------------------------------------------
    # The loop below examines each treasure.
    #
    # Each treasure contains two coordinates:
    #   treasure[0] = X coordinate
    #   treasure[1] = Y coordinate
    #
    # Your job:
    #   1. Calculate the difference between the player's X coordinate
    #      and the treasure's X coordinate.
    #   2. Calculate the difference between their Y coordinates.
    #   3. Use math.hypot() to calculate the distance.
    #
    # Hint: math.hypot() requires two arguments.


    for index, treasure in enumerate(TREASURE_SPOTS):

        # Skip treasures that have already been collected.
        if index in game["collected"]:
            continue

        
        x_difference = player_x - treasure[0]  # TODO 1D: Calculate X difference.
        
        y_difference = player_y - treasure[1]  # TODO 1E: Calculate Y difference.
        
            #distance_to_home = float("inf")  # TODO 1F: Use math.hypot().
        distance = float(math.hypot(x_difference, y_difference))  # Initialize distance to home.
        

        # --------------------------------------------------------------
        # TASK 2 — COLLECT A TREASURE AND AWARD POINTS
        # --------------------------------------------------------------
        # Replace False with a condition that checks TWO things:
        #
        #   1. Is the player within TREASURE_RADIUS?
        #   2. Has this treasure NOT already been collected?
        #
        # Both conditions must be true.
        #
        # If they are true:
        #   - Add the treasure index to game["collected"].
        #   - Increase game["score"] by TREASURE_POINTS.
        #   - Call celebrate_treasure(game, index).
        #
        # Hints: and, not in, .add(), +=

        if index not in game["collected"] and distance < TREASURE_RADIUS:
            game["collected"].add(index)
            game["score"] += TREASURE_POINTS
            celebrate_treasure(game, index)




    # ------------------------------------------------------------------
    # TASK 3 — DETERMINE WHEN THE PLAYER WINS
    # ------------------------------------------------------------------
    # This code is OUTSIDE the for loop.
    #
    # Your job:
    #   1. Calculate the maximum possible score.
    #   2. Check whether the player has earned that score.
    #   3. If so, change game["result"] to "WIN".
    #   4. Print the victory message.
    #
    # Hints:
    #   - len(TREASURE_SPOTS) tells you how many treasures exist.
    #   - Each treasure is worth TREASURE_POINTS.
    #   - Compare the player's score with the maximum score.

    maximum_score = len(TREASURE_SPOTS) * TREASURE_POINTS  # TODO 3A: Calculate maximum possible score.

    if game["score"] >= maximum_score:  # TODO 3B: Replace False with your winning condition.

        game["result"] = "WIN"  # TODO 3C: Update game["result"].

        print("Congratulations! You have won the game!")  # TODO 3D: Print the victory message.


        # --------------------------------------------------------------
        # TASK 4 — CALCULATE AND DISPLAY COMPLETION TIME
        # --------------------------------------------------------------
        # This task belongs INSIDE the winning condition above.
        #
        # Your job:
        #   1. Calculate how many simulated seconds have passed.
        #   2. Store the result in completion_time.
        #   3. Display the time using an f-string.
        #   4. Format the result to one decimal place.
        #
        # Hints:
        #   - game["step"] contains the total simulation steps.
        #   - PHYSICS_HZ contains the steps per simulated second.
        #   - Use :.1f to display one decimal place.

        completion_time = game["step"] / PHYSICS_HZ  # TODO 4A: Calculate elapsed seconds.
    
        print(f"Completion time: {completion_time:.1f} seconds")  # TODO 4B: Print completion_time using an f-string.

def update_guard(player_id, guard, game):
    """Each guard has a personal patrol index, alert timer, and navigation state."""
    guard_id = guard["id"]
    gx, gy, _ = robot_xy_heading(guard_id)
    px, py, _ = robot_xy_heading(player_id)
    to_player = math.hypot(px - gx, py - gy)
    # Each guard can see and chase the player independently of other guards.
    if to_player < GUARD_ALERT_RADIUS:
        guard["guard_alert_until"] = game["step"] + round(GUARD_ALERT_SECONDS * PHYSICS_HZ)

    if game["step"] < guard["guard_alert_until"]:
        guard["guard_behavior"] = "CHASE"
        update_hunter(guard_id, player_id, game, guard)
        return

    guard["guard_behavior"] = "PATROL"
    waypoint = PATROL_POINTS[guard["patrol_index"]]
    if math.hypot(gx - waypoint[0], gy - waypoint[1]) < PATROL_RADIUS:
        guard["patrol_index"] = (guard["patrol_index"] + 1) % len(PATROL_POINTS)
        waypoint = PATROL_POINTS[guard["patrol_index"]]
    update_hunter(guard_id, player_id, game, guard, target_xy=waypoint,
                  speed_factor=GUARD_PATROL_SPEED_FACTOR)


# =====================================================================
# PART 6 — GAME STATUS, RESET, AND MAIN LOOP (ALL PREWRITTEN)
# =====================================================================
_hud_id = -1
_mode_hud_id = -1


def status_hud(game, player_id, guards, force=False):
    """Display score, countdown, nearest guard, and the number chasing."""
    global _hud_id, _mode_hud_id
    if SMOKE_TEST:
        return
    interval = max(1, PHYSICS_HZ // 2)
    if not force and game["step"] % interval != 0:
        return
    remaining = max(0.0, MISSION_SECONDS - game["step"] / PHYSICS_HZ)
    px, py, _ = robot_xy_heading(player_id)
    distances = []
    for guard in guards:
        gx, gy, _ = robot_xy_heading(guard["id"])
        distances.append(math.hypot(px - gx, py - gy))
    closest = min(distances)
    chasing = sum(guard["guard_behavior"] == "CHASE" for guard in guards)
    total = len(TREASURE_SPOTS) * TREASURE_POINTS
    if game["result"] == "WIN":
        label = f"YOU WIN! ALL TREASURES! SCORE {game['score']} | R = REPLAY | Q = QUIT"
        rgb = [0.22, 1, 0.35]
    elif game["result"] == "CAUGHT":
        label = f"CAUGHT! SCORE {game['score']}/{total} | R = RETRY | Q = QUIT"
        rgb = [1, 0.27, 0.26]
    elif game["result"] == "TIMEOUT":
        label = f"TIME UP! SCORE {game['score']}/{total} | R = RETRY | Q = QUIT"
        rgb = [1, 0.27, 0.26]
    else:
        label = (f"TREASURE HUNT | {game['score']:02d}/{total} POINTS | "
                 f"{remaining:05.1f}s | GUARDS {len(guards)} | CLOSEST {closest:.1f}m")
        rgb = [1, 0.97, 0.18]
    _hud_id = p.addUserDebugText(label, [-8.3, 8.08, 1.7],
                                 textColorRGB=rgb, textSize=1.07,
                                 replaceItemUniqueId=_hud_id)
    modes = "  ".join(f"G{g['number']}:{g['mode'].upper()}" for g in guards)
    _mode_hud_id = p.addUserDebugText(
        f"CHASING: {chasing}/{len(guards)} | {modes} "
        f"| COINS LEFT: {len(TREASURE_SPOTS) - len(game['collected'])}",
        [-8.3, 7.40, 1.7], textColorRGB=[0.8, 0.95, 1.0], textSize=0.88,
        replaceItemUniqueId=_mode_hud_id)


def reset_game():
    """Create an arena, treasures, and 1–3 independently moving guards."""
    global _hud_id, _mode_hud_id
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / PHYSICS_HZ)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")
    create_arena()
    bodies, labels = [], []
    for index, spot in enumerate(TREASURE_SPOTS):
        body, label = make_treasure(spot, index + 1)
        bodies.append(body)
        labels.append(label)
    player_id = make_husky(PLAYER_START, (0.16, 0.49, 1.0))

    # Each guard has a separate PyBullet robot and a separate state dictionary.
    # In particular, no guard shares a navigation mode or recovery timer.
    guards = []
    for number in range(GUARD_COUNT):
        start = GUARD_STARTS[number]
        guard_id = make_husky(start, GUARD_COLORS[number])
        guards.append({
            "id": guard_id,
            "number": number + 1,
            "guard_behavior": "PATROL",
            "guard_alert_until": 0,
            "patrol_index": GUARD_PATROL_STARTS[number],
            "mode": "chase",  # Navigation state, independent for each guard.
            "mode_until": 0,
            "avoid_direction": 1,
            "recover_direction": 1 if number % 2 == 0 else -1,
            "anchor": start,
            "anchor_step": 0,
        })

    p.resetDebugVisualizerCamera(cameraDistance=25, cameraYaw=0,
                                  cameraPitch=-88, cameraTargetPosition=[0, 0, 0])
    _last_commands.clear()
    _ray_ids.clear()
    _hud_id = -1
    _mode_hud_id = -1
    game = {
        "step": 0,
        "result": "PLAYING",
        "score": 0,
        "collected": set(),
        "treasure_bodies": bodies,
        "treasure_labels": labels,
        "wall_start": time.monotonic(),
        "report_start": time.monotonic(),
        "report_step": 0,
    }
    status_hud(game, player_id, guards, force=True)
    print("\nTREASURE HUNT — NEW GAME")
    print(f"GUARD COUNT: {GUARD_COUNT} (change GUARD_COUNT at the top: 1, 2, or 3)")
    print("Collect the FIVE numbered GOLD treasures for 10 points each.")
    print("Each guard patrols independently and CHASES when close. Avoid ALL of them!")
    print("WASD / arrows = drive | R = restart | Q = quit")
    print("Click the PyBullet window to give keyboard controls focus.")
    return player_id, guards, game


def smoke_test_mission():
    """Check mission rules without a GUI; student starter is unfinished."""
    player_id, guards, game = reset_game()
    if len(guards) != GUARD_COUNT or len({g["id"] for g in guards}) != GUARD_COUNT:
        print("SMOKE TEST FAIL: guard count or IDs wrong")
        return 1
    # Test no pickup at an irrelevant location.
    p.resetBasePositionAndOrientation(player_id, [0, -7.0, 0.22], [0, 0, 0, 1])
    update_mission(player_id, game)
    if game["score"] != 0:
        print("SMOKE TEST FAIL: awarded treasure away from coins")
        return 1
    for x, y in TREASURE_SPOTS:
        p.resetBasePositionAndOrientation(player_id, [x, y, 0.22], [0, 0, 0, 1])
        update_mission(player_id, game)
        update_mission(player_id, game)  # Revisit must not double-score.
    max_score = len(TREASURE_SPOTS) * TREASURE_POINTS
    if game["score"] == max_score and game["result"] == "WIN":
        print(f"SMOKE TEST PASS: all treasures give {max_score} points, win once")
        return 0
    if game["score"] == 0 and game["result"] == "PLAYING":
        print("SMOKE TEST PASS: student starter is correctly unfinished (TASKS 1-4)")
        return 0
    print(f"SMOKE TEST FAIL: unexpected score {game['score']} and result {game['result']}")
    return 1


def run():
    if SMOKE_TEST:
        client = p.connect(p.DIRECT)
    else:
        client = p.connect(p.GUI, options=f"--width={WINDOW_WIDTH} --height={WINDOW_HEIGHT}")
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    if client < 0:
        raise RuntimeError("Could not start PyBullet: install pybullet and use a desktop GUI.")
    if not SMOKE_TEST:
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    try:
        if SMOKE_TEST:
            return smoke_test_mission()
        player_id, guards, game = reset_game()
        while p.isConnected():
            keys = p.getKeyboardEvents()
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('q'), ord('Q'))):
                break
            if any(keys.get(k, 0) & p.KEY_WAS_TRIGGERED for k in (ord('r'), ord('R'))):
                player_id, guards, game = reset_game()
                continue
            if game["result"] != "PLAYING":
                time.sleep(0.04)
                continue
            player_teleop(player_id, keys)
            for guard in guards:
                update_guard(player_id, guard, game)
            p.stepSimulation()
            game["step"] += 1
            elapsed = game["step"] / PHYSICS_HZ
            # Touching ANY guard means you lose; guard-to-guard bumps are harmless.
            caught_by = next(
                (guard["number"] for guard in guards
                 if p.getContactPoints(bodyA=player_id, bodyB=guard["id"])),
                None
            )
            # Tagging and timeout take priority over collecting a coin.
            if caught_by is not None:
                game["result"] = "CAUGHT"
                print(f"CAUGHT by Guard {caught_by}! Collected {game['score']} "
                      f"points in {elapsed:.1f}s.")
            elif elapsed >= MISSION_SECONDS:
                game["result"] = "TIMEOUT"
                print(f"TIME UP! Final score {game['score']}.")
            else:
                update_mission(player_id, game)
            if game["result"] != "PLAYING":
                stop_robot(player_id)
                for guard in guards:
                    stop_robot(guard["id"])
                status_hud(game, player_id, guards, force=True)
            status_hud(game, player_id, guards)
            now = time.monotonic()
            if now - game["report_start"] >= 5.0:
                sim_time = (game["step"] - game["report_step"]) / PHYSICS_HZ
                rate = sim_time / (now - game["report_start"])
                print(f"SIM SPEED: {rate:.2f}x | {max(0.0, MISSION_SECONDS - elapsed):.1f}s left")
                game["report_start"] = now
                game["report_step"] = game["step"]
            if MAX_PLAYBACK_SPEED > 0:
                block = max(1, PHYSICS_HZ // 20)
                if game["step"] % block == 0:
                    target_wall = game["step"] / (PHYSICS_HZ * MAX_PLAYBACK_SPEED)
                    delay = target_wall - (now - game["wall_start"])
                    if delay > 0:
                        time.sleep(delay)
        return 0
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.exit(run())
