import math

def calculate_bearing(lat1, lon1, lat2, lon2):

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    delta_lon = lon2_rad - lon1_rad

    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    bearing_rad = math.atan2(y, x)

    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360

    return bearing_deg


# latA, lonA = -7.241604106065979, 112.72989220580334
# latB, lonB = -7.241982128999947, 112.73133411522784

latA, lonA = -7.2028193, 112.7422065 # ECW Utara
latB, lonB = -7.2036017, 112.7423198 # ECW Selatan

# latA, lonA = -7.240692510259271, 112.72929799800684
# latB, lonB = -7.241182100784971, 112.72278559484337

bearing = round(calculate_bearing(latA, lonA, latB, lonB), 2)
print(f"The bearing is: {bearing} degrees ")

bearing = round(calculate_bearing(latB, lonB, latA, lonA), 2)
print(f"The bearing is: {bearing} degrees ")
