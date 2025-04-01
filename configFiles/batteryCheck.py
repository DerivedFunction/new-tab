import time
import os

INTERVAL = 1  # Target time interval in seconds

# Record the start time
start_time = time.time()

while True:
    time.sleep(INTERVAL)
    
    # Calculate total elapsed time based on current time
    total_sec = int(time.time() - start_time)
    
    # Calculate days, hours, minutes, seconds
    days = total_sec // 86400
    remaining_sec = total_sec % 86400
    hours = remaining_sec // 3600
    remaining_sec %= 3600
    minutes = remaining_sec // 60
    seconds = remaining_sec % 60
    
    # Write to a temporary file
    with open("battery_test_temp.log", "w") as f:
        f.write(f"{days} {hours:02d}:{minutes:02d}:{seconds:02d}\n")
        f.flush()  # Ensure data is written
        os.fsync(f.fileno())  # Sync to disk
    
    # Atomically replace the main file with the temporary file
    os.replace("battery_test_temp.log", "battery_test.log")