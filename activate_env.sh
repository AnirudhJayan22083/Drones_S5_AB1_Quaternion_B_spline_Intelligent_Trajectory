# Source this to use the project's .venv in the current bash session:
#   source ./activate_env.sh
# (Puts .venv's python.exe first on PATH, plus .venv/Library/bin, which
# carries the conda-forge BLAS/LAPACK DLLs numpy needs -- without it,
# np.linalg.* crashes the interpreter hard instead of raising a Python error.)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_ROOT="$SCRIPT_DIR/.venv"

if [ ! -f "$ENV_ROOT/python.exe" ]; then
    echo "No .venv found at $ENV_ROOT. Run: conda create -p ./.venv python=3.12 -y" >&2
    return 1 2>/dev/null || exit 1
fi

export PATH="$ENV_ROOT/Library/bin:$ENV_ROOT:$ENV_ROOT/Scripts:$PATH"
echo "Activated .venv ($ENV_ROOT). 'python' now points to it."
