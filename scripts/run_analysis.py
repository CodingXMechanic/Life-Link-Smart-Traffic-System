"""Run the fixed-timer vs adaptive comparison."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from src.analysis.compare import generate_comparison_charts
generate_comparison_charts(duration=60.0, n_vehicles=5, out_dir="output")
