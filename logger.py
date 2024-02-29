import os
import csv
from datetime import datetime

class Logger:
    def __init__(self, filename="logs/logfile.csv"):
        self.default = filename
        self.filename = filename
        self._check_and_create_file()
        self.log_count = 0

    def _check_and_create_file(self):
        i = 0
        while os.path.exists(self.filename):
            name, ext = os.path.splitext(self.default)
            i += 1
            self.filename = f"{name}{str(i).zfill(3)}{ext}"
        
        with open(self.filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "count", "param1", "param2", "param3", "error"])

    def log(self, param1, param2, param3, error=""):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(self.filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, self.log_count, param1, param2, param3, error])
            self.log_count += 1
