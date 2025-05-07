#!/bin/bash

# setup a Python virtual environment for Python development.

# ensure script runs from script location
cd "$(dirname "${BASH_SOURCE[0]}")" || exit

venv_name="oae_venv"

# determine OS
uname_out="$(uname -s)"
case "${uname_out}" in
Linux*) machine=Linux ;;
CYGWIN* | MINGW* | MSYS*) machine=Windows ;;
*) machine="UKNOWN:${uname_out}" ;;
esac

# check for existing venv and install python and depdencies via poetry if it
# does not exist
if [ ! -d ../${venv_name} ]; then
    pyenv install 3.11 --skip-existing
    pyenv local 3.11

    # install venv in project root
    if [ "$machine" = "Windows" ]; then
        python3 -m venv "../${venv_name}"
    else
        python3 -m venv "${venv_name}"
    fi
    poetry install
fi

# activate venv depending on OS
if [ "$machine" = "Windows" ]; then
    # shellcheck source=../oae_venv/bin/activate
    source "../${venv_name}/Scripts/activate"
else
    # shellcheck source=../oae_venv/bin/activate
    source "${venv_name}/bin/activate"
fi

# move python cache out of project directory
export PYTHONPYCACHEPREFIX=/tmp/pycache
export PYTHONPATH="$(pwd)"
