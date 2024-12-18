import os
import yaml

def load_yaml(file_path: str):
    """
    Loads a YAML file from the specified path.

    :param file_path: Path to the YAML file.
    :return: Content of the YAML file as a dictionary.
    :raises FileNotFoundError: If the file is not found.
    :raises yaml.YAMLError: If there is an error reading the YAML file.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File {file_path} not found.")

    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"Error reading YAML file {file_path}: {e}")