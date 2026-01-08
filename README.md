# X26_RobotCode
This repo holds 3484's 2026 robot code, written in Python.

- [General docs](https://github.com/FRC-Team3484/docs)

## Installation

Clone this repository:
```bash
git clone https://github.com/FRC-Team3484/X26_RobotCode
```
Or, use `Git: Clone` in the VSCode command pallate (`Ctrl+Shift+P`)

Create a new virtual enviroment and activate it:
```bash
python -m venv .venv
```
Or, with the Python VSCode extension, click the Python version number in the bottom right, and select `Create Virtual Enviroment`

Install the dependencies:
```bash
pip install -r requirements.txt
```

And sync robotpy libraries:
```bash
robotpy sync
```

## Usage
Test your code using the following command: 

```bash
robotpy test
```