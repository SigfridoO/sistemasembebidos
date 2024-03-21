from PyQt5.QtWidgets import QLabel

class Indicador(QLabel):
    def __init__(self, tag:str="", nombre:str=""):
        super().__init__()
        self.tag = tag
        self.nombre = nombre
        self.estado = False

        self.setStyleSheet(f"""
                           background-color: white; 
                           border: 1px solid black""")
        
    def get_tag(self):
        return self.tag
    
    def set_tag(self, tag:str=""):
        self.tag = tag

        
    def get_nombre(self):
        return self.nombre
    
    def set_nombre(self, nombre:str=""):
        self.nombre = nombre

        
    def get_estado(self):
        return self.estado
    
    def set_estado(self, estado:bool=""):
        self.estado = estado