#!/usr/bin/env bash
# Link Rescue's Go2 locomotion task into an Isaac Lab checkout.
#
# WHY A SYMLINK RATHER THAN A COPY. Isaac Lab is a third-party checkout that gets pulled and
# rebased; a task copied into its tree is untracked by this repo, invisible to review, and
# gone the first time the checkout is replaced. Linking keeps the single source of truth in
# Rescue while still landing inside the package tree that `isaaclab_tasks`' `import_packages`
# walks -- which is what makes the task self-register with no edit to any Isaac Lab file.
#
# Coexists with P2Dingo's `go2_p2dingo` link: different package name, different task ids,
# different experiment directory. Both can be installed at once.
#
# Idempotent: re-running relinks. Refuses to clobber a real directory, because that would
# mean Isaac Lab ships a task of the same name and the collision needs a human.
#
#   ./install_task.sh                     # links into $HOME/IsaacLab
#   ISAACLAB_PATH=/opt/IsaacLab ./install_task.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/go2_rescue"
ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/IsaacLab}"
DEST_DIR="$ISAACLAB_PATH/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config"
DEST="$DEST_DIR/go2_rescue"

if [ ! -d "$SRC" ]; then
  echo "error: task source not found at $SRC" >&2
  exit 1
fi

if [ ! -d "$DEST_DIR" ]; then
  echo "error: Isaac Lab velocity config dir not found:" >&2
  echo "       $DEST_DIR" >&2
  echo "       Set ISAACLAB_PATH to your Isaac Lab checkout." >&2
  exit 1
fi

if [ -e "$DEST" ] && [ ! -L "$DEST" ]; then
  echo "error: $DEST exists and is NOT a symlink -- refusing to replace it." >&2
  echo "       Move it aside if it is a stale copy of this task." >&2
  exit 1
fi

ln -sfn "$SRC" "$DEST"
echo "linked  $DEST"
echo "     -> $SRC"

# The stale-bytecode trap: Python caches a package's compiled modules next to the source,
# and a __pycache__ left by a PREVIOUS link (or a copy that was moved aside) can shadow an
# edit made here. Cheap to clear, confusing to debug.
find "$SRC" -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true

cat <<'MSG'

Registered task ids (after the next Isaac Lab start):
  Isaac-Velocity-Rescue-Unitree-Go2-v0            train
  Isaac-Velocity-Rescue-Unitree-Go2-Play-v0       watch it on the stair terrain
  Isaac-Velocity-Rescue-Unitree-Go2-Flat-v0       reward-set checks on a plane
  Isaac-Velocity-Rescue-Unitree-Go2-Flat-Play-v0  posture / gait / turn probes

Verify without training:
  cd "$ISAACLAB_PATH" && ./isaaclab.sh -p scripts/environments/list_envs.py | grep Rescue
MSG
