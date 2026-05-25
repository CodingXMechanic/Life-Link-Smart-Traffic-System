"""Generate a sample CSV log file for demonstration."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from src.logging.logger import generate_sample_csv
generate_sample_csv("logs/sample_log.csv", n_rows=100)
