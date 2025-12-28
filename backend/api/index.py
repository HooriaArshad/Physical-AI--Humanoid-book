# Vercel serverless function entry point
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from app_rag import app

# Export for Vercel
handler = app
