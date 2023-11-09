import psutil

if __name__ == "__main__":
    print(f"Number of logical CPU: {psutil.cpu_count(logical=True)}")
    print(f"Number of physical CPU: {psutil.cpu_count(logical=False)}")
