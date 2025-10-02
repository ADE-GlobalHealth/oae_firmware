# Python Setup

This document outlines the setup required for development of Python-based tools
and scripts, including the serial protocol for device USB control.

Development on linux is recommended, though all tools for management of the
Python virtual environment can be installed on Windows. 

The following tools are required:

- uv 

## uv Install

Install UV for your choice of OS via the [UV install instructions](https://docs.astral.sh/uv/getting-started/installation/).

## Using uv 

The follow are a few useful commands using UV. For further documentation,
consult the UV docs.

### Add Dependencies

```
uv add numpy
```

### Update Dependencies

```
uv lock --upgrade
```

### Removing Dependencies

```
uv remove numpy 
```
