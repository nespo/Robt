from math import sin, cos, sqrt, atan2, radians

# Function to calculate the distance between two points on Earth
def calculate_distance(lat1, lon1, lat2, lon2):
  """
  This function calculates the distance between two points on Earth in kilometers.

  Args:
      lat1 (float): Latitude of the first point in degrees.
      lon1 (float): Longitude of the first point in degrees.
      lat2 (float): Latitude of the second point in degrees.
      lon2 (float): Longitude of the second point in degrees.

  Returns:
      float: The distance between the two points in kilometers.
  """
  # Convert decimal degrees to radians
  lat1 = radians(lat1)
  lon1 = radians(lon1)
  lat2 = radians(lat2)
  lon2 = radians(lon2)

  # Earth radius in kilometers
  R = 6371

  # Calculate the difference in latitude and longitude
  dlon = lon2 - lon1
  dlat = lat2 - lat1

  # Haversine formula
  a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2)
  c = 2 * atan2(sqrt(a), sqrt(1 - a))

  # Distance in kilometers
  distance = R * c

  return distance

# Provided coordinates
lat1, lon1 = 62.8788833618, 27.6375541687
lat2, lon2 = 62.878815, 27.637536

# Calculate the distance
distance = calculate_distance(lat1, lon1, lat2, lon2)

# Convert kilometers to centimeters and round to two decimal places
distance_in_centimeters = distance * 100000

print(f"The distance between the two points is approximately {distance_in_centimeters:.2f} centimeters.")
