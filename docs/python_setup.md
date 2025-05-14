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

5. Whenever developing Python scripts as a part of `oae_firmware`, always source
   the setup script to source the Python virtual environment.

## Windows

1. Install scoop with scoop's quickstart guide: https://scoop.sh/
2. Using scoop, install pipx:
   https://github.com/pypa/pipx?tab=readme-ov-file#on-windows
3. Using pipx, install Poetry: https://python-poetry.org/docs/#installation
4. Follow install instructions for Pyenv-win:
   https://github.com/pyenv/pyenv?tab=readme-ov-file#installation
5. Install git bash, as a part of git: https://git-scm.com/downloads
6. With Poetry and Pyenv installed, source the setup script
   `setup_python_venv.sh` in `scripts/`.

```
# git bash, from project root
source ./scripts/setup_python_venv.sh
```

7. The previous step will install Python and required dependencies with Poetry.
   The same script can be used to source the Python virtual environment every
   time, but if use of the default Windows command prompt is preferred, the
   Python environment can be sourced with:

```
# windows command prompt, from project root
oae_venv\Scripts\activate
```

## Using Poetry

The follow are a few useful commands using Poetry. For further documentation,
consult the Poetry docs.

### Install Dependencies

```
poetry install
```

### Update Dependencies

```
poetry update
```

### Adding Dependencies

```
# example, adding the pandas package
poetry add pandas
```

### Removing Dependencies

```
# example, removing the pandas package
poetry remove pandas
```
