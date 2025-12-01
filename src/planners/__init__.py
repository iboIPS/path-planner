from .a_star import run as run_a_star
from .d_star import run as run_d_star
from .dijkstra import run as run_dijkstra
from .dwa import run as run_dwa
from .multi_paths import run as run_multi_paths
from .potential_field import run as run_potential_field
from .teb import run as run_teb
from .vfh import run as run_vfh

from .rrt.rrt import run as run_rrt
from .rrt.rrt_star import run as run_rrt_star
from .rrt.rrt_connect import run as run_rrt_connect
from .rrt.dynamic_rrt import run as run_dynamic_rrt
from .rrt.informed_rrt_start import run as run_informed_rrt_start
from .rrt.goal_biased_rrt import run as run_goal_biased_rrt

from .prm.prm import run as run_prm
from .prm.prm_star import run as run_prm_star
from .prm.lazy_prm import run as run_lazy_prm
from .prm.gaussian_prm import run as run_gaussian_prm
from .prm.bridge_prm import run as run_bridge_prm

PLANNERS = {
    "A*": run_a_star,
    "Dijkstra": run_dijkstra,
    "D*": run_d_star,
    "RRT": run_rrt,
    "RRT*": run_rrt_star,
    "RRTConnect": run_rrt_connect,
    "RRTDynamic": run_dynamic_rrt,
    "RRTInformed*": run_informed_rrt_start,
    "RRTGoalBiased": run_goal_biased_rrt,
    "PRM": run_prm,
    "PRM*": run_prm_star,
    "PRMLazy": run_lazy_prm,
    "PRMGuassian": run_gaussian_prm,
    "PRMBridge": run_bridge_prm,
    "DWA": run_dwa,
    "TEB": run_teb,
    "VFH": run_vfh,
    "PotentialField": run_potential_field,
    "MultiPaths": run_multi_paths,
}

__all__ = [
    "PLANNERS",
    "run_a_star",
    "run_d_star",
    "run_dijkstra",
    "run_rrt",
    "run_rrt_star",
    "run_rrt_connect",
    "run_dynamic_rrt",
    "run_informed_rrt_start",
    "run_goal_biased_rrt",
    "run_prm",
    "run_prm_star",
    "run_lazy_prm",
    "run_gaussian_prm",
    "run_bridge_prm",
    "run_dwa",
    "run_teb",
    "run_vfh",
    "run_potential_field",
    "run_multi_paths",
]
