"""The active-perception paper's sweep protocol, shared by the sweep tools in ``scripts/``.

``paper_sweep.py`` runs it and ``compare_paper_sweep.py`` reads its results. Stdlib only, like
``episode_summary``. ``run_experiments.sh`` keeps its own shell copy of the scene lists.
"""

LARGE_SCENES = ["102344280", "102344049", "102816009", "103997445", "102816036"]
SMALL_SCENES = ["102815835", "102816852", "102816216"]
PLANNERS = ["nbs", "dbs", "spt", "tsp", "rrtstar", "rrt"]
TASKS = ["explore", "recon", "points"]

# cost_budget [s of trajectory time] of each scene set.
LARGE_BUDGET = 600
SMALL_BUDGET = 300

# Wall-clock cap per (scene, task), in minutes, as recorded in the paper's
# mission_coordinator_node/time_limit. It is part of the protocol, not a safety net: by
# compare_paper_sweep.PAPER_COST it truncated TSP in every point-collection cell and in most
# large-scene reconstruction and exploration cells, and RRT* in large-scene point collection
# and most large-scene exploration cells; those truncated runs are in the paper's means.
# Modal value over the paper's planners and runs for that cell.
PAPER_WALL_CAPS = {
    ("102344280", "explore"): 11,
    ("102344280", "recon"): 11,
    ("102344280", "points"): 12,
    ("102344049", "explore"): 11,
    ("102344049", "recon"): 11,
    ("102344049", "points"): 12,
    ("102816009", "explore"): 11,
    ("102816009", "recon"): 11,
    ("102816009", "points"): 12,
    ("103997445", "explore"): 11,
    ("103997445", "recon"): 11,
    ("103997445", "points"): 18,
    ("102816036", "explore"): 15,
    ("102816036", "recon"): 15,
    ("102816036", "points"): 15,
    ("102815835", "explore"): 7,
    ("102815835", "recon"): 7,
    ("102815835", "points"): 7,
    ("102816852", "explore"): 7,
    ("102816852", "recon"): 7,
    ("102816852", "points"): 7,
    ("102816216", "explore"): 7,
    ("102816216", "recon"): 7,
    ("102816216", "points"): 7,
}
