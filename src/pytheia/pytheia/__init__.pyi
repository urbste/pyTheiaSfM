"""Type stubs for the pytheia package."""
from pytheia.pytheia import io, math, matching, mvs, sfm, solvers
import pytheia.pytheia as pytheia

# Re-export all submodules for convenience
__all__ = ['io', 'math', 'matching', 'mvs', 'sfm', 'solvers', 'pytheia']