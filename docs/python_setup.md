# Python Setup

This document outlines the setup required for development of Python-based tools
and scripts, including the serial protocol for device USB control.

Development on linux is recommended, though all tools for management of the
Python virtual environment can be installed on Windows. As such, separate
instructions are available for linux and Windows.

The following tools are required:

- Poetry
- Pyenv

## Linux

1. Install pipx based on your distribution:
   https://github.com/pypa/pipx?tab=readme-ov-file#on-linux
2. Using pipx, install Poetry: https://python-poetry.org/docs/#installation
3. Follow install instructions for Pyenv:
   https://github.com/pyenv/pyenv?tab=readme-ov-file#installation
4. With Poetry and Pyenv installed, source the setup script
   `setup_python_venv.sh` in `scripts/`

```
# bash, from project root
source ./scripts/setup_python_venv.sh
```

## Windows

1. Install scoop with scoop's quickstart guide: https://scoop.sh/
2. Using scoop, install pipx:
   https://github.com/pypa/pipx?tab=readme-ov-file#on-windows
3. Using pipx, install Poetry: https://python-poetry.org/docs/#installation
4. Follow install instructions for Pyenv-win:
   https://github.com/pyenv/pyenv?tab=readme-ov-file#installation
5. todo
