import folium
import pandas as pd
from geopy.distance import geodesic
import math

def generate_map(filename: str, output_filename: str):
    filepath = f'results/{filename}.jsonl'
    df = pd.read_json(filepath, lines=True)

    lat_col = 'filtered_latitude'
    lon_col = 'filtered_longitude'

    acc_x_col = 'acceleration_x'
    acc_y_col = 'acceleration_y'
    acc_z_col = 'acceleration_z'

    vel_x_col = 'angular_velocity_x'
    vel_y_col = 'angular_velocity_z'
    vel_z_col = 'angular_velocity_z'

    start_coords = [df[lat_col].iloc[0], df[lon_col].iloc[0]]
    mymap = folium.Map(location=start_coords, zoom_start=20)

    folium.Marker(
        start_coords,
        popup=f"Start: {df['timestamp'].iloc[0]}",
        icon=folium.Icon(color='green')
    ).add_to(mymap)

    coordinates = []
    total_distance_m = 0

    aggregate_velocity = 0.0

    for i in range(len(df)):
        coord = [df[lat_col].iloc[i], df[lon_col].iloc[i]]
        coordinates.append(coord)

        if i > 0:
            prev_coord = [df[lat_col].iloc[i-1], df[lon_col].iloc[i-1]]
            segment_distance = geodesic(prev_coord, coord).meters
            total_distance_m += segment_distance

        # single_velocity = math.sqrt(df[vel_x_col].iloc[i]**2 + df[vel_y_col].iloc[i]**2 + df[vel_z_col].iloc[i]**2)
        # aggregate_velocity += single_velocity

    folium.PolyLine(locations=coordinates, color='blue', weight=3, opacity=0.7).add_to(mymap)

    end_coords = [df[lat_col].iloc[-1], df[lon_col].iloc[-1]]
    folium.Marker(
        end_coords,
        popup=f"End: {df['timestamp'].iloc[-1]}",
        icon=folium.Icon(color='red')
    ).add_to(mymap)

    mymap.save(output_filename)

    print(f"Total movement distance: {total_distance_m:.2f} meters")
    # print(f"Peta telah disimpan sebagai {output_filename}.")

    start_point = (df[lat_col].iloc[0], df[lon_col].iloc[0])
    end_point = (df[lat_col].iloc[-1], df[lon_col].iloc[-1])
    start_end_distance_m = geodesic(start_point, end_point).meters

    # print(f"Jarak langsung dari titik awal ke akhir: {start_end_distance_m:.2f} meter")
    # print(f"aggregate velocity adalah : {round(aggregate_velocity/len(df),2)}")


