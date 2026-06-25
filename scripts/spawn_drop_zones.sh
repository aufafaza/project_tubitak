#!/bin/bash

CENTER_X=${1:-0.0}
CENTER_Y=${2:-0.0}
HALF_W=${3:-30.0}
HALF_H=${4:-15.0}

X_MIN=$(python3 -c "print($CENTER_X - $HALF_W)")
X_MAX=$(python3 -c "print($CENTER_X + $HALF_W)")
Y_MIN=$(python3 -c "print($CENTER_Y - $HALF_H)")
Y_MAX=$(python3 -c "print($CENTER_Y + $HALF_H)")

rand() {
  python3 -c "import random; print(round(random.uniform($1, $2), 2))"
}

X_RED=85.53;   Y_RED=164.33
X_BLUE=80.0;   Y_BLUE=20.0

echo "Spawn red drop zone at ($X_RED, $Y_RED)"
echo "Spawn blue drop zone at ($X_BLUE, $Y_BLUE)"

gz service -s /world/runway/remove \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "name: \"dropzone_red\", type: 2" 2>/dev/null

gz service -s /world/runway/remove \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req "name: \"dropzone_blue\", type: 2" 2>/dev/null

gz service -s /world/runway/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"model://dropzone_red\", name: \"dropzone_red\", pose: {position: {x: $X_RED, y: $Y_RED, z: 0.0}}"

gz service -s /world/runway/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"model://dropzone_blue\", name: \"dropzone_blue\", pose: {position: {x: $X_BLUE, y: $Y_BLUE, z: 0.0}}"
