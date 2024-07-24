import json
from typing import List
from CamExtris import CamExtris

class ExtriJsonCreator:
    def __init__(self, all_extris: list[CamExtris], path):
        self.all_extris = all_extris
        cam_extris_dict_list = [cam_extris.to_dict() for cam_extris in self.all_extris]   
        file_path = path + "extris.json"
        with open(file_path, 'w') as json_file:
            json.dump(cam_extris_dict_list, json_file, indent=4)
        
class ExtriJsonLoader:
    def __init__(self, path: str):
        # Lesen Sie die JSON-Datei ein
        with open(path, 'r') as json_file:
            data = json.load(json_file)
        # Konvertieren Sie die JSON-Daten in eine Liste von CamExtris Objekten
        self.all_extris = [CamExtris(item['name'], item['extri']) for item in data]
    
    def get_all_extris(self)->list[CamExtris]:
        return self.all_extris