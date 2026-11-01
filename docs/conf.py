"""Sphinx configuration for the Open Apero documentation (Doxygen + Breathe + MyST)."""

import os

# -- Project information -----------------------------------------------------
project = "Open Apero"
copyright = "2024-2026, ETH Zürich"
author = "Kaixian Qu, Han Wang"
version = "1.0"
release = "1.0.0"

# -- General configuration ---------------------------------------------------
extensions = [
    "breathe",  # Doxygen XML → Sphinx bridge
    "myst_parser",  # Markdown support (instead of RST)
    "sphinxcontrib.mermaid",  # Mermaid diagrams (also rendered natively by GitHub)
]

# MyST-Parser settings
myst_enable_extensions = [
    "colon_fence",
    "fieldlist",
    "deflist",
]
myst_heading_anchors = 4  # anchors for headings down to H4, which the pages link
myst_all_links_external = False

# Render plain ```mermaid and ```math fenced code blocks (which GitHub renders natively) as
# the Sphinx mermaid and math directives, so the same source renders in both places.
myst_fence_as_directive = ["mermaid", "math"]

# Diagrams render at their natural size (custom.css keeps them within the column): mermaid gives
# each flowchart its pixel size, and the container's height follows the diagram, not a fixed 500 px.
mermaid_init_config = {
    "startOnLoad": False,
    "themeVariables": {"fontSize": "14px"},
    "flowchart": {"useMaxWidth": False, "rankSpacing": 35, "nodeSpacing": 30},
}
mermaid_height = "auto"

# Source file suffixes
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# -- Breathe configuration --------------------------------------------------
breathe_projects = {
    "open_apero": os.path.join(os.path.abspath(os.path.dirname(__file__)), "generated", "xml"),
}
breathe_default_project = "open_apero"
breathe_default_members = ("members", "undoc-members")

# -- HTML output -------------------------------------------------------------
# sphinx-book-theme (https://sphinx-book-theme.readthedocs.io), built on the PyData Sphinx theme.
html_theme = "sphinx_book_theme"
html_title = "Open Apero"
html_static_path = ["_static"]
html_logo = "../open_apero_logo.png"
html_favicon = "_static/favicon.ico"
html_css_files = ["custom.css"]

# Light/dark follows the reader's system setting until they pick one with the header toggle.
html_context = {"default_mode": "auto"}

# The theme's sidebar minus its search field: the page header already has one (an icon on a phone).
html_sidebars = {"**": ["navbar-logo.html", "icon-links.html", "sbt-sidebar-nav.html"]}

html_theme_options = {
    "repository_url": "https://github.com/OpenApero/openapero",
    "repository_branch": "main",
    "path_to_docs": "docs",
    "use_repository_button": True,
    "use_issues_button": True,
    "use_edit_page_button": False,
    # Readers print a page to PDF from the browser instead.
    "use_download_button": False,
    "home_page_in_toc": True,
    "show_toc_level": 2,
    "logo": {"alt_text": "Open Apero logo"},
}

# -- Exclude patterns --------------------------------------------------------
exclude_patterns = ["_build", "generated", "Thumbs.db"]


# -- On-page table of contents ------------------------------------------------
# Breathe names every C++ object for the ToC itself, bypassing Sphinx's toc_object_entries,
# so "On this page" would list every member. Keep only the top-level objects (the classes).
def _drop_member_toc_entries(app, doctree):
    from sphinx import addnodes

    for sig in doctree.findall(addnodes.desc_signature):
        node = sig.parent.parent
        while node is not None and not isinstance(node, addnodes.desc):
            node = node.parent
        if node is not None:  # nested in another object: a member
            sig.attributes.pop("_toc_name", None)


def setup(app):
    # Before the ToC collector, which reads _toc_name on doctree-read at the default priority 500.
    app.connect("doctree-read", _drop_member_toc_entries, priority=400)
