#!/bin/bash
# Remove every entity name ever spawned by spawn_world.sh.
# Names not present in the world are silently skipped.

remove_model() {
  gz service -s /world/planner_world/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "name: \"$1\" type: MODEL" > /dev/null 2>&1
}

NAMES=(
  # Current scene
  tall_blocker mid_n1 mid_s1 low_n2 low_s2
  tree_n tree_s
  person_start person_goal
  start_marker goal_marker
  # Earlier Fuel-only layout
  apartment_n1 apartment_n2 house_s1 house_s2 school1
  house_n1 house_n2
  office1
  suv1
  person1 person2
  # Legacy names from earlier spawns — kept so leftovers get cleaned
  asphalt1 asphalt2 asphalt3 asphalt4
  grass1 grass2 grass3 grass4
  house1 house2 house3
  apartment1 apartment2
  office1 office2
  tree_n1 tree_n2 tree_n3 tree_s1 tree_s2 tree_s3
  lamp1 lamp2 lamp3 lamp4
  fountain
  person3
  hatchback ambulance
  hydrant mailbox stopsign
)

for name in "${NAMES[@]}"; do
  echo "Removing ${name}..."
  remove_model "${name}"
done

echo "Done."
