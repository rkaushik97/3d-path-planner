#!/bin/bash
# Populate /world/planner_world with a path-planning test scene.
# Buildings = colored primitive boxes; trees = real Fuel models;
# 2 people for context; floating green/red spheres mark start + goal.

set -e

FUEL="https://fuel.gazebosim.org/1.0/OpenRobotics/models"

spawn_fuel() {
  local name=$1 pose=$2 uri=$3
  echo "Spawning ${name}..."
  gz service -s /world/planner_world/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><include><name>${name}</name><pose>${pose}</pose><uri>${uri}</uri></include></sdf>'"
}

# Box origin is at its center — caller sets z = Lz/2 to sit on the ground.
spawn_box() {
  local name=$1 pose=$2 size=$3 r=$4 g=$5 b=$6
  echo "Spawning ${name}..."
  gz service -s /world/planner_world/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"${name}\"><static>true</static><link name=\"link\"><collision name=\"col\"><geometry><box><size>${size}</size></box></geometry></collision><visual name=\"vis\"><geometry><box><size>${size}</size></box></geometry><material><ambient>${r} ${g} ${b} 1</ambient><diffuse>${r} ${g} ${b} 1</diffuse></material></visual></link><pose>${pose}</pose></model></sdf>'"
}

# Visual-only sphere (no collision, drone passes through). For start/goal markers.
spawn_marker() {
  local name=$1 pose=$2 radius=$3 r=$4 g=$5 b=$6
  echo "Spawning ${name}..."
  gz service -s /world/planner_world/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf: '<?xml version=\"1.0\"?><sdf version=\"1.6\"><model name=\"${name}\"><static>true</static><link name=\"link\"><visual name=\"vis\"><geometry><sphere><radius>${radius}</radius></sphere></geometry><material><ambient>${r} ${g} ${b} 1</ambient><diffuse>${r} ${g} ${b} 1</diffuse><emissive>${r} ${g} ${b} 1</emissive></material></visual></link><pose>${pose}</pose></model></sdf>'"
}

# --- Buildings (boxes) ---
spawn_box "tall_blocker" "30  0 7 0 0 0" "6 6 14" "0.85" "0.20" "0.20"
spawn_box "mid_n1"       "12  10 4 0 0 0" "5 5 8"  "0.20" "0.40" "0.85"
spawn_box "mid_s1"       "12 -10 4 0 0 0" "5 5 8"  "0.20" "0.40" "0.85"
spawn_box "low_n2"       "45  10 2 0 0 0" "4 4 4"  "0.95" "0.55" "0.20"
spawn_box "low_s2"       "45 -10 2 0 0 0" "4 4 4"  "0.95" "0.55" "0.20"

# --- Trees (Fuel) ---
spawn_fuel "tree_n" "20  8 0 0 0 0" "${FUEL}/Pine Tree"
spawn_fuel "tree_s" "20 -8 0 0 0 0" "${FUEL}/Oak tree"

# --- People (Fuel — may render dark under software GL but readable as silhouettes) ---
spawn_fuel "person_start" " 8  6 0 0 0  1.0" "${FUEL}/Standing person"
spawn_fuel "person_goal"  "47 -4 0 0 0 -1.0" "${FUEL}/Walking person"

# --- Start (green) and goal (red) markers — emissive spheres at flight altitude ---
spawn_marker "start_marker" " 0 0 5 0 0 0" "0.7" "0.10" "0.85" "0.10"
spawn_marker "goal_marker"  "50 0 5 0 0 0" "0.7" "0.85" "0.10" "0.10"

echo
echo "Spawn complete. 11 entities in /world/planner_world."
echo "Start (green sphere): (0,0,5)  |  Goal (red sphere): (50,0,5)"
echo "Heights: tall=14m | mid=8m | low=4m"
