# Setup Guide for main_loop.py Control System

## Prerequisites
- **Python 3.13.12** (exact version used in development)
- **numpy 1.26.4** (specific version required - see note below)
- National Instruments DAQmx hardware driver (for nidaqmx)

**Note on numpy version:** This project uses numpy 1.26.4 due to a dependency conflict:
- `moteus` requires numpy < 2.0
- `nidaqmx 1.3.0` requests numpy >= 2.1 for Python 3.13, but works fine with 1.26.4

## Installation Steps

### 1. Copy Required Files
Transfer the following files to the target machine:

**Main Control File:**
- `main_loop.py` - Main control loop implementation

**Core Modules:**
- `ACTUATOR.py` - Actuator control class with TTL DAQ integration
- `Controllers.py` - Controller implementations
- `filters.py` - Signal filtering utilities

**Configuration:**
- `requirements.txt` - Python dependencies

### 2. Create a virtual environment
```powershell
py -3.13 -m venv .venv
```

### 3. Activate the virtual environment
**Windows (PowerShell):**
```powershell
.\.venv\Scripts\Activate.ps1
```

### 4. Install dependencies
**Important:** Install packages in the following order to avoid dependency conflicts:

```powershell
pip install nidaqmx==1.3.0
pip install moteus==0.3.76
```

**Note:** Installing nidaqmx first, then moteus avoids pip's dependency resolver conflicts. You may see warnings about numpy version incompatibility - these can be safely ignored as the code has been tested and works correctly with numpy 1.26.4.

## Hardware Dependencies

### NIDAQmx Installation
The `nidaqmx` package requires NI-DAQmx driver to be installed **before** running pip install:
1. Download NI-DAQmx from: https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html
2. Install the driver on your system
3. Then install Python package via requirements.txt

### Moteus Installation
The moteus motor controller library will be installed via requirements.txt.
If you encounter issues, refer to: https://github.com/mjbots/moteus

## Module Dependencies for main_loop.py

```
main_loop.py
├── ACTUATOR.py
│   ├── filters.py (signal processing)
│   ├── moteus (motor control)
│   ├── nidaqmx (DAQ interface)
│   ├── scipy (interpolation)
│   └── numpy (numerical operations)
│
└── Controllers.py
    ├── ACTUATOR.py
    ├── filters.py
    ├── scipy (interpolation)
    └── numpy (numerical operations)
```

## Running the Code
```powershell
python main_loop.py
```

## Verification
After installation, verify all dependencies:
```powershell
python -c "import numpy, scipy, moteus, nidaqmx; print('All dependencies installed successfully!')"
```

## Troubleshooting
- **Python Version**: Python 3.13.12 was used in testing
- **Dependency conflict error**: If `pip install -r requirements.txt` fails with numpy version conflicts, install packages individually in order: first nidaqmx, then moteus (see step 4)
- **Numpy version conflict**: If you see warnings about nidaqmx 1.3.0 requiring numpy>=2.1, ignore them. The packages were tested and work with numpy 1.26.4 (moteus requires numpy<2)
- **nidaqmx fails**: Ensure NI-DAQmx driver is installed first (system-level)
