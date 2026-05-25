#!/usr/bin/env bash
# Life-Link — End-to-End Demo Runner
# Usage: bash run_demo.sh [--headless] [--analysis] [--test]

set -e
cd "$(dirname "$0")"

echo "============================================================"
echo "  LIFE-LINK Smart Traffic System — Demo Runner"
echo "============================================================"

# Check Python
python3 --version || { echo "Python 3 required"; exit 1; }

# Install deps if needed
if ! python3 -c "import pygame" 2>/dev/null; then
    echo "[Setup] Installing dependencies..."
    pip install -r requirements.txt
fi

MODE=${1:-"--headless"}

if [[ "$MODE" == "--test" ]]; then
    echo "[Step 1] Running pytest suite..."
    python3 -m pytest tests/ -v
    echo "[PASS] All tests passed!"

elif [[ "$MODE" == "--analysis" ]]; then
    echo "[Step 1] Generating charts..."
    python3 scripts/generate_charts.py
    echo "[Step 2] Running adaptive vs fixed analysis..."
    python3 scripts/run_analysis.py
    echo "[Done] Charts in output/"

elif [[ "$MODE" == "--headless" ]]; then
    echo "[Step 1] Running headless simulation..."
    python3 scripts/run_demo_headless.py
    echo "[Step 2] Generating sample CSV..."
    python3 scripts/generate_sample_csv.py
    echo "[Step 3] Generating charts..."
    python3 scripts/generate_charts.py
    echo ""
    echo "[All steps complete]"
    echo "  Logs:    logs/"
    echo "  Charts:  output/"

elif [[ "$MODE" == "--pygame" ]]; then
    echo "[Launching Pygame simulation...]"
    python3 -m src.ui.pygame_ui

else
    echo "Usage: bash run_demo.sh [--headless|--analysis|--test|--pygame]"
fi
