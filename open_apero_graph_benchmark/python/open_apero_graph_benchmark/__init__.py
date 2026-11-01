"""The graph-benchmark tools: one module per command-line tool, plus the code they share.

Each tool in ``scripts/`` is the entry point of the module of the same name. The shared modules
are :mod:`.benchmark_ranges` (the range aggregation) and :mod:`.paths`, which says where the inputs
(``data/``) and the outputs (below the data root, ``$OPEN_APERO_DATA_ROOT`` or
``~/open_apero_data``) live.
"""
