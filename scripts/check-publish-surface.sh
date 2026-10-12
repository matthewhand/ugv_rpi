#!/usr/bin/env bash
# Publish-surface guard. Dependency-free; safe to run from a git hook.
#
# Blocks two classes of accidental leak in a repo destined for a public fork:
#   1. High-signal credentials in staged content.
#   2. Tracked raw camera captures (fisheye/jpg/mp4 debug frames) and
#      prebuilt binaries.
#
# Exits non-zero (blocking the commit) on a hit. Set SKIP_PUBLISH_GUARD=1 to bypass.

set -uo pipefail
[ "${SKIP_PUBLISH_GUARD:-0}" = "1" ] && exit 0

# Scan staged content (what would be committed), not the whole tree.
if git rev-parse --verify HEAD >/dev/null 2>&1; then
  STAGED=$(git diff --cached --name-only --diff-filter=ACMR)
else
  STAGED=$(git ls-files)
fi

fail=0

# --- 1. credential patterns -------------------------------------------------
SECRET_RE='BEGIN (RSA |OPENSSH |EC |PGP )?PRIVATE KEY|AKIA[0-9A-Z]{16}|ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,}|sk-ant-[A-Za-z0-9-]{20,}|sk-[A-Za-z0-9]{32,}|xox[baprs]-[A-Za-z0-9-]{20,}|AIza[0-9A-Za-z_-]{30,}'
if [ -n "$STAGED" ]; then
  # shellcheck disable=SC2086
  if git diff --cached -U0 2>/dev/null | grep -nE "$SECRET_RE" | grep -vE '^\+.*(RE=|_RE|REGEX|example|placeholder|sk-\.\.\.)' | head; then
    echo "BLOCKED: possible credential in staged changes." >&2
    fail=1
  fi
fi

# --- 2. tracked camera captures / binaries ---------------------------------
CAPTURE_RE='(ugv_wall_check|capture_|frame_|snapshot).*\.(jpg|jpeg|png|mp4|avi|mov|mkv)$'
BIN_RE='(^|/)sunny$|\.bin$|\.hef$'
# shellcheck disable=SC2086
bad=$(echo "$STAGED" | grep -E "$CAPTURE_RE|$BIN_RE" || true)
if [ -n "$bad" ]; then
  echo "BLOCKED: camera capture or prebuilt binary staged:" >&2
  echo "$bad" >&2
  fail=1
fi

exit $fail
