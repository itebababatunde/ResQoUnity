
Traceback (most recent call last):
  File "/home/kishi/ResQoUnity/main.py", line 25, in <module>
    from omniverse_sim import run_sim
  File "/home/kishi/ResQoUnity/omniverse_sim.py", line 67, in <module>
    from omni.isaac.orbit_tasks.utils import get_checkpoint_path
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit_tasks/omni/isaac/orbit_tasks/__init__.py", line 25, in <module>
    from .utils import import_packages
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit_tasks/omni/isaac/orbit_tasks/utils/__init__.py", line 9, in <module>
    from .parse_cfg import get_checkpoint_path, load_cfg_from_registry, parse_env_cfg
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit_tasks/omni/isaac/orbit_tasks/utils/parse_cfg.py", line 16, in <module>
    from omni.isaac.orbit.envs import RLTaskEnvCfg
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/envs/__init__.py", line 24, in <module>
    from . import mdp, ui
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/envs/mdp/__init__.py", line 18, in <module>
    from .actions import *  # noqa: F401, F403
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/envs/mdp/actions/__init__.py", line 8, in <module>
    from .actions_cfg import *
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/envs/mdp/actions/actions_cfg.py", line 9, in <module>
    from omni.isaac.orbit.managers.action_manager import ActionTerm, ActionTermCfg
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/managers/__init__.py", line 13, in <module>
    from .action_manager import ActionManager, ActionTerm
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/managers/action_manager.py", line 16, in <module>
    from omni.isaac.orbit.assets import AssetBase
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/assets/__init__.py", line 41, in <module>
    from .articulation import Articulation, ArticulationCfg, ArticulationData
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/assets/articulation/__init__.py", line 8, in <module>
    from .articulation import Articulation
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/assets/articulation/articulation.py", line 22, in <module>
    import omni.isaac.orbit.sim as sim_utils
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/sim/__init__.py", line 29, in <module>
    from .converters import *  # noqa: F401, F403
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/sim/converters/__init__.py", line 19, in <module>
    from .asset_converter_base import AssetConverterBase
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/sim/converters/asset_converter_base.py", line 15, in <module>
    from omni.isaac.orbit.utils.assets import check_file_path
  File "/home/kishi/IsaacLab_v0.3.1/source/extensions/omni.isaac.orbit/omni/isaac/orbit/utils/assets.py", line 43
    return os.path.abspath(target_path)
IndentationError: unexpected indent
2025-11-14 21:45:04 [12,006ms] [Warning] [carb] [Plugin: omni.spectree.delegate.plugin] Module /home/kishi/.local/share/ov/pkg/isaac-sim-2023.1.1/kit/exts/omni.usd_resolver/bin/libomni.spectree.delegate.plugin.so remained loaded after unload request
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.stageupdate.plugin] Deprecated: direct use of IStageUpdate callbacks is deprecated. Use IStageUpdate::getStageUpdate instead.
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,009ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,010ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Annotators' for removal
2025-11-14 21:45:04 [12,010ms] [Warning] [omni.graph.core.plugin] Could not find category 'Replicator:Core' for removal
2025-11-14 21:45:04 [12,012ms] [Warning] [carb.audio.context] 1 contexts were leaked
2025-11-14 21:45:04 [12,047ms] [Warning] [carb] Recursive unloadAllPlugins() detected!
2025-11-14 21:45:04 [12,053ms] [Warning] [omni.core.ITypeFactory] Module /home/kishi/.local/share/ov/pkg/isaac-sim-2023.1.1/kit/exts/omni.activity.core/bin/libomni.activity.core.plugin.so remained loaded after unload request.

========================================
Simulation ended
========================================
kishi@jeffry-ThinkStation-P360-Tower:~/ResQoUnity$ ^C
kishi@jeffry-ThinkStation-P360-Tower:~/ResQoUnity$ 