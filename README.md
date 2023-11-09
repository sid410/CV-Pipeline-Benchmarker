# CV-Pipeline-Benchmarker
For monitoring CPU usage of each processes spawned in the script and total usage of each core.

### Run the virtual environment

If running on windows:
```
python -m venv venv
.\venv\Scripts\activate
pip install -r requirements.txt
```

If running on linux:
```
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### Run the benchmarker

Run `benchmark_cpu.py` to see the CPU usage in real-time.

Play around with how many process you want to create, and other arguments by reading:
```
python benchmark_cpu.py --help
```

### Basic purpose of the script
Ideally, you would set the processes between 2 and total logical cores.
Run `get_cpu_count.py` to get the number of logical and physical cores.
Spawning too much process however creates too much overhead making it inefficient.
For different processing power, number of cores, and how heavy the CV pipeline is, we need to do trial-and-error to estimate how many process we need.