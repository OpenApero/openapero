#!/usr/bin/env bash
# Build the Doxygen/Breathe/Sphinx documentation.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Doxyfile resolves source paths through REPO_ROOT.
export REPO_ROOT="$(cd .. && pwd)"

# Dependencies

if ! command -v doxygen &> /dev/null; then
    echo "ERROR: doxygen is not installed."
    echo "Install with: sudo apt-get install doxygen"
    exit 1
fi

if ! command -v sphinx-build &> /dev/null; then
    echo "ERROR: sphinx-build is not installed."
    echo "Install with: pip install -r ${SCRIPT_DIR}/requirements.txt"
    exit 1
fi

# The Python packages of requirements.txt, by import name.
missing=()
for module in sphinx breathe myst_parser sphinx_book_theme sphinxcontrib.mermaid; do
    python3 -c "import ${module}" 2>/dev/null || missing+=("${module}")
done
if [ ${#missing[@]} -gt 0 ]; then
    echo "ERROR: not installed: ${missing[*]}"
    echo "Install with: pip install -r ${SCRIPT_DIR}/requirements.txt"
    exit 1
fi

echo "=== Generating Open Apero Documentation ==="
echo "Repo root: ${REPO_ROOT}"

# Doxygen XML

echo ""
echo "--- Step 1/2: Running Doxygen (C++ → XML) ---"
rm -rf "${SCRIPT_DIR}/generated"
doxygen Doxyfile
echo "Doxygen XML written to: ${SCRIPT_DIR}/generated/xml/"

# Sphinx HTML

echo ""
echo "--- Step 2/2: Running Sphinx (XML + Markdown → HTML) ---"
rm -rf "${SCRIPT_DIR}/_build"
# -W --keep-going: any warning (a broken include, link or cross-reference) fails the build, after
# all of them are reported.
sphinx-build -W --keep-going -b html "${SCRIPT_DIR}" "${SCRIPT_DIR}/_build/html"

echo ""
echo "=== Documentation generated successfully! ==="
echo "Open: ${SCRIPT_DIR}/_build/html/index.html"
