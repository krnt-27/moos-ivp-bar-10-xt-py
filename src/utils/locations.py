import yaml

def load_locations(file_path: str, location_key:str):
    with open(file_path, "rb") as f:
        locs = yaml.safe_load(f)
    return locs[location_key]