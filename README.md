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

### Test Mode (No Protocol)
```powershell
python main_loop.py
```

### With Experimental Protocol
```powershell
# Training protocol for 70kg participant
python main_loop.py --protocol training --bodyweight 70

# Training protocol for 154lb participant
python main_loop.py --protocol training --bodyweight-lbs 154

# Real trial protocol for 65kg participant
python main_loop.py --protocol real_trial --bodyweight 65

# Real trial with reproducible randomization (seed)
python main_loop.py --protocol real_trial --bodyweight 65 --seed 42

# With custom data file name
python main_loop.py --protocol training --bodyweight 70 --datafile participant01
```

### Command-Line Arguments
- `--protocol {training,real_trial}` - Protocol type for force perturbations
- `--bodyweight WEIGHT` - Participant bodyweight in kg
- `--bodyweight-lbs WEIGHT` - Participant bodyweight in lbs (alternative to --bodyweight)
- `--seed SEED` - Random seed for real_trial protocol (optional)
- `--datafile NAME` - Data file name (default: test0)

## Force Protocol Generation

The system supports two experimental protocols for force perturbation sequences:

### Protocol Types

#### 1. Training Protocol (40 trials)
- **15 trials**: Control (0 force)
- **25 trials**: Incrementing perturbations in blocks of 5 trials each
  - Trials 16-20: 2% of bodyweight
  - Trials 21-25: 4% of bodyweight
  - Trials 26-30: 6% of bodyweight
  - Trials 31-35: 8% of bodyweight
  - Trials 36-40: 10% of bodyweight

#### 2. Real Trial Protocol (50 trials)
- **25 trials**: Control (0 force)
- **25 trials**: Randomized perturbations (5 trials at each level, randomized order)
  - 5 trials each at: 2%, 4%, 6%, 8%, 10% of bodyweight

### Generating Protocols

#### Method 1: Direct Command-Line (Recommended)
Simply run main_loop.py with protocol arguments:
```powershell
# Training protocol
python main_loop.py --protocol training --bodyweight 70

# Real trial protocol
python main_loop.py --protocol real_trial --bodyweight 65 --seed 42
```

The protocol will be automatically generated and saved to `force_maps/` when the program starts.

#### Method 2: Pre-generate Protocol Files
Use the helper script to generate and inspect protocols before running:
```powershell
# Interactive mode (will prompt for inputs)
python generate_protocol.py

# Command-line mode
python generate_protocol.py --protocol training --bodyweight 70
python generate_protocol.py --protocol real_trial --bodyweight 65 --seed 42
```

#### Method 3: Programmatic generation (Advanced)
Edit the `TTLController.__init__()` method in [Controllers.py](Controllers.py) (not recommended - use command-line instead).

### Protocol Files
- Generated protocols are saved in `force_maps/` directory as JSON files
- Files include metadata: protocol type, bodyweight, force levels, timestamps
- View existing protocols: check `force_maps/force_map_*.json` files

### Viewing Protocol Details
Generated JSON files contain:
- Trial-by-trial force mapping
- Metadata (bodyweight, force levels, timestamps)
- Randomization details (for real_trial protocol)

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
