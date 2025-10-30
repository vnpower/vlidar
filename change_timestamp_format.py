import time
from datetime import datetime

# 1. Get the current timestamp (float)
raw_timestamp = 1761236134.9473908
print(f"Raw timestamp: {raw_timestamp}")

# 2. Convert to a datetime object (local time)
dt_object = datetime.fromtimestamp(raw_timestamp)
print(f"Datetime object: {dt_object}")

# 3. Format the datetime object into a specific string format
# Example format: YYYY-MM-DD HH:MM:SS
formatted_time = dt_object.strftime("%Y-%m-%d %H:%M:%S")
print(f"Formatted time: {formatted_time}")

# Example format: Month Day, Year, HH:MM AM/PM
custom_format = dt_object.strftime("%B %d, %Y, %I:%M %p")
print(f"Custom format: {custom_format}")