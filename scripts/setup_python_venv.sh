#!/bin/bash

# setup a Python virtual environment for Python development.

# ensure script runs from script location
cd "$(dirname "${BASH_SOURCE[0]}")" || exit

venv_name="oae_venv"

# check for existing venv and install python and depdencies via poetry if it
# does not exist
if [ ! -d ../${venv_name} ]; then
    pyenv install 3.11 --skip-existing
    pyenv local 3.11
    python3 -m venv ${venv_name}
    poetry install
fi

# shellcheck source=./oae_venv/bin/activate
source ./${venv_name}/bin/activate
# move python cache out of project directory
export PYTHONPYCACHEPREFIX=/tmp/pycache
export PYTHONPATH="$(pwd)"
