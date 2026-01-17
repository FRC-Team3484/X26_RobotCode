# X26_RobotCode
This repo holds 3484's 2026 robot code, written in Python.

- [General docs](https://github.com/FRC-Team3484/docs)
- [FRC3484_Lib_Python](https://github.com/FRC-Team3484/FRC3484_Lib_Python)

## Installation

Clone this repository:
```bash
git clone https://github.com/FRC-Team3484/X26_RobotCode
```
Or, use `Git: Clone` in the VSCode command palate (`Ctrl+Shift+P`)

Create a new virtual environment and activate it:
```bash
python -m venv .venv
```
Or, with the Python VSCode extension, click the Python version number in the bottom right, and select `Create Virtual Environment`

Install the dependencies:
```bash
pip install -r requirements.txt
```
Or, check `Install dependencies` when creating your virtual environment.

Finally, sync robotpy libraries:
```bash
robotpy sync
```

## Usage
Test your code using the following command: 

```bash
robotpy test
```