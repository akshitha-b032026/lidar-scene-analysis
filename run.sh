#!/usr/bin/env bash
# Usage: ./run.sh [/path/to/nuscenes]
cd "$(dirname "$0")"
[ -d .venv ] || python3 -m venv .venv
source .venv/bin/activate
pip install -q -r requirements.txt
[ -n "$1" ] && export NUSCENES_DATAROOT="$1"
uvicorn backend.app:app --reload
