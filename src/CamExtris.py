class CamExtris:
    def __init__(self, name, extri):
        self.name = name
        self.extri = extri
    def to_dict(self):
        print(self.extri)
        return {'name': self.name, 'extri': self.extri.tolist()}