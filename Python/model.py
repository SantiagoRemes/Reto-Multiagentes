from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
import random

import numpy as np

import networkx as nx 

import math as math

import json

JSONFile = {
    "numRobots": 0,
    "robots": [],
    "numEstantes": 0,
    "posEstantes": [],
    "numPilas": 0,
    "posPilas": [],
    "numBandasEntrada": 0,
    "posBandasEntrada": [],
    "numBandasSalida": 0,
    "posBandasSalida": [],
    "numCrates": 0,
    "crates": []
}


class Celda(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Paquete(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.lugar = "Estanteria"

class BandaEntrada(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.paquete = None
        self.x = -1
        self.y = 0

class BandaSalida(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.x = 1
        self.y = 0

class Estanteria(Agent):
    def __init__(self, unique_id, model, y_offset):
        super().__init__(unique_id, model)
        self.paquete = None
        self.x = 0
        self.y = y_offset

class Recarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.ocupado = False
        self.x = 0
        self.y = 1

class RobotCarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 300
        self.needs_charge = False
        self.paquete = None
        self.orden_asignada = None

    @staticmethod

    def cargar(self, cargador):
        cargador.ocupado = True
        self.carga = self.carga + 25
        if(self.carga >= 300):
            self.carga = 300
            cargador.ocupado = False
            self.needs_charge = False

    def dejar_paquete_estanteria(self, estanteria):
        self.sig_pos = estanteria.pos
        paquete = self.paquete
        self.paquete = None
        estanteria.paquete = paquete
        paquete.lugar = ""
        paquete.model.grid.move_agent(paquete, estanteria.pos)
    
    def dejar_paquete_banda(self, banda):
        self.model.lista_paquetes.remove(self.paquete)
        self.model.grid.remove_agent(self.paquete)
        self.model.lista_ordenes.remove(self.orden_asignada)
        self.orden_asignada = None
        self.paquete = None

    def recoger_paquete_banda(self, banda):
        paquete = banda.paquete
        banda.paquete = None
        self.paquete = paquete
        paquete.model.grid.move_agent(paquete, self.pos)
    
    def recoger_paquete_estanteria(self, estanteria):
        self.sig_pos = estanteria.pos
        paquete = estanteria.paquete
        estanteria.paquete = None
        self.paquete = paquete
        paquete.model.grid.move_agent(paquete, self.pos)
    
    def find_paquete_banda(self):
        lista_entrada = []
        for banda in self.model.lista_entrada:
            if banda.paquete != None:
                lista_entrada.append(banda)
    
        if len(lista_entrada) > 0:
            closest_paquete = min(lista_entrada, key=lambda c: self.distance(self.pos, (c.pos[0] + c.x, c.pos[1] + c.y)))
            
            return closest_paquete
        else:
            return None
    
    def find_cargador(self):
        lista_cargadores = []
        for cargador in self.model.lista_cargadores:
            if not(cargador.ocupado) or (cargador.pos[0] + cargador.x, cargador.pos[1] + cargador.y) == self.pos:
                lista_cargadores.append(cargador)
    
        if len(lista_cargadores) > 0:
            closest_cargador = min(lista_cargadores, key=lambda c: self.distance(self.pos, (c.pos[0] + c.x, c.pos[1] + c.y)))
            
            return closest_cargador
        else:
            return None
    
    def find_estanteria(self):
        lista_estanteria = []
        for estanteria in self.model.lista_estanterias:
            if estanteria.paquete == None:
                lista_estanteria.append(estanteria)
    
        if len(lista_estanteria) > 0:
            closest_estanteria = min(lista_estanteria, key=lambda c: self.distance(self.pos, (c.pos[0] + c.x, c.pos[1] + c.y)))
            
            return closest_estanteria
        else:
            return None
        
    def asignar_orden(self):
        for orden in self.model.lista_ordenes:
            if(orden[2] == "Libre"):
                orden[2] = "En Proceso"
                self.orden_asignada = orden
                
                return
        else:
            return
        
    def encontrar_camino(self, target):
        sig_pos_robots = [robot.sig_pos for robot in self.model.lista_robots if robot != self and robot.sig_pos is not None]

        graph = self.model.graph

        graph.add_node(self.pos)

        # Add neighbors of the current position to the graph
        neighbors = self.model.grid.get_neighborhood(self.pos, moore=False, include_center=True)
        for neighbor in neighbors:
            try:
                graph.add_edge(self.pos, neighbor)
            except:
                pass

        for sig_pos in sig_pos_robots:
            try:
                graph.remove_node(sig_pos)
            except:
                pass


        current_pos = self.pos
        target_pos = (target.pos[0] + target.x, target.pos[1] + target.y)

        contents = self.model.grid.get_cell_list_contents(self.pos)
        for content in contents:
            if isinstance(content, (Estanteria)):
                return [self.pos, (content.pos[0] + content.x, content.pos[1] + content.y)]

        if current_pos not in graph.nodes or target_pos not in graph.nodes:
            contents = self.model.grid.get_cell_list_contents(self.pos)
            for content in contents:
                if isinstance(content, (Estanteria)):
                    return [self.pos, (content.pos[0] + content.x, content.pos[1] + content.y)]
            return []

        try:
            path = nx.shortest_path(graph, current_pos, target_pos)
            return path
        except nx.NetworkXNoPath:
            # input("SOTP")
            return []
    
    def distance(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return abs(x1 - x2) + abs(y1 - y2)

    def step(self):
        # Sin paquete
        if(self.paquete == None):
            # Falta Bateria
            if(self.needs_charge):
                cargador = self.find_cargador()
                if(cargador != None):
                    camino = self.encontrar_camino(cargador)
                    if(len(camino) > 1):
                        self.sig_pos = camino[1]
                    elif(len(camino) != 0):
                        self.cargar(self, cargador)
                        self.sig_pos = self.pos
                    else:
                        self.sig_pos = self.pos
                else:
                    self.sig_pos = self.pos
            else:
                if(self.carga < 100):
                    self.needs_charge = True
                self.asignar_orden()
            # Tiene orden
                if(self.orden_asignada != None):
                    contents = self.model.grid.get_cell_list_contents(self.orden_asignada[1].pos)
                    for content in contents:
                        if isinstance(content, (Estanteria)):
                            camino = self.encontrar_camino(content)
                            if(len(camino) > 1):
                                self.sig_pos = camino[1]
                            elif(len(camino) != 0):
                                self.recoger_paquete_estanteria(content)
                            else:
                                self.sig_pos = self.pos
                    
            # No tiene orden
                else:
                    banda = self.find_paquete_banda()
                    if(banda != None):
                        camino = self.encontrar_camino(banda)
                        if(len(camino) > 1):
                            self.sig_pos = camino[1]
                        elif(len(camino) != 0):
                            self.recoger_paquete_banda(banda)
                            self.sig_pos = self.pos
                        else:
                            self.sig_pos = self.pos
                    else:
                        self.sig_pos = self.pos
        # Con paquete         
        else:
            # Llevar paquete a estanteria
            if(self.paquete.lugar == "Estanteria"):
                estanteria = self.find_estanteria()
                if(estanteria != None):
                    camino = self.encontrar_camino(estanteria)
                    if(len(camino) > 1):
                        self.sig_pos = camino[1]
                    elif(len(camino) != 0):
                        self.dejar_paquete_estanteria(estanteria)
                    else:
                        self.sig_pos = self.pos
            # Completar orden
            elif(self.paquete.lugar == "Salida"):
                camino = self.encontrar_camino(self.orden_asignada[0])
                if(len(camino) > 1):
                        self.sig_pos = camino[1]
                elif(len(camino) != 0):
                    self.dejar_paquete_banda(self.orden_asignada[0])
                    self.sig_pos = self.pos
                else:
                    self.sig_pos = self.pos

            
    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1


        if self.carga > 0:
            self.model.grid.move_agent(self, self.sig_pos)
            if(self.paquete != None):
                self.paquete.model.grid.move_agent(self.paquete, self.sig_pos)
            self.carga -= 1

        self.sig_pos = None


class Bodega(Model):
    def __init__(self, M, N, 
                 steps_per_package_generation,
                 steps_create_order,
                 num_agentes,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes

        self.lista_robots = []
        self.lista_cargadores = []
        self.lista_estanterias = []
        self.lista_entrada = []
        self.lista_salida = []
        self.lista_paquetes = []
        self.lista_ordenes = []
        self.steps_per_package = steps_per_package_generation
        self.steps_package = 0
        self.steps_leave_package = steps_create_order
        self.steps_package_leave = 0
        self.paquete_counter = 0
        self.graph = None

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de Cargadores
        posiciones_cargadores = []
        for x in range(math.floor(M/2)-2, math.floor(M/2)+3, 1):
            posiciones_cargadores.append((x, 0))
        JSONFile["posPilas"] = []
        for id, pos in enumerate(posiciones_cargadores):
            bateria = Recarga(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(bateria, pos)
            posiciones_disponibles.remove(pos)
            self.lista_cargadores.append(bateria)
            JSONFile["posPilas"].append({"x": pos[0], "y": 0, "z": pos[1]})
        JSONFile["numPilas"] = len(self.lista_cargadores)

        # Posicionamiento de Banejas de Salida
        posiciones_salida = []
        for y in range(2, N-1, 2):
            posiciones_salida.append((0, y))

        JSONFile["posBandasSalida"] = []
        for id, pos in enumerate(posiciones_salida):
            banda = BandaSalida(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(banda, pos)
            posiciones_disponibles.remove(pos)
            self.lista_salida.append(banda)
            JSONFile["posBandasSalida"].append({"x": pos[0], "y": 0, "z": pos[1]})
        JSONFile["numBandasSalida"] = len(self.lista_salida)

        # Posicionamiento de Banejas de Entradas
        posiciones_entrada = []
        for y in range(2, N-1, 2):
            posiciones_entrada.append((M-1, y))
        JSONFile["posBandasEntrada"] = []
        for id, pos in enumerate(posiciones_entrada):
            banda = BandaEntrada(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(banda, pos)
            posiciones_disponibles.remove(pos)
            self.lista_entrada.append(banda)
            JSONFile["posBandasEntrada"].append({"x": pos[0], "y": 0, "z": pos[1]})
        JSONFile["numBandasEntrada"] = len(self.lista_entrada)

        # Posicionamiento de Estanterias
        posiciones_estantes = []
        for x in range(math.floor(M/2) - 7, math.floor(M/2) + 7, 1):
                if(x != math.floor(M/2) - 1 and x != math.floor(M/2) and x != math.floor(M/2) + 1):
                    posiciones_estantes.append((x, N-1))
        for y in range(5, math.floor(N/2) + 5, 5):
            for x in range(math.floor(M/2) - 7, math.floor(M/2) + 7, 1):
                if(x != math.floor(M/2) - 1 and x != math.floor(M/2) and x != math.floor(M/2) + 1):
                    posiciones_estantes.append((x,N - y))
            for x in range(math.floor(M/2) - 7, math.floor(M/2) + 7, 1):
                if(x != math.floor(M/2) - 1 and x != math.floor(M/2) and x != math.floor(M/2) + 1):
                    posiciones_estantes.append((x,N - y - 1))
        
        # JSONFile["estanterias"] = []
        posEstantes = []
        for id, pos in enumerate(posiciones_estantes):
            y_offset = 1
            if((N- 1 - pos[1])  % 5 == 0):
                y_offset = -1
            elif(pos[1] == 47):
                y_offset = -1
            estanteria = Estanteria(int(f"{num_agentes}0{id}") + 1, self, y_offset)
            self.grid.place_agent(estanteria, pos)
            posiciones_disponibles.remove(pos)
            self.lista_estanterias.append(estanteria)
            JSONFile["posEstantes"].append({"x": pos[0], "y": 0, "z": pos[1]})
        JSONFile["numEstantes"] = len(self.lista_estanterias)

        JSONFile["robots"] = []
        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotCarga(id, self)
            self.lista_robots.append(robot)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)
            #JSONFile["robots"].append({"id": id, "targetPositions": [{"x": robot.pos[0], "y": 0, "z": robot.pos[1]}]})
            JSONFile["robots"].append({
                "id": id,
                "initialPosition": {"x": robot.pos[0], "y": 0, "z": robot.pos[1]},
                "targetPositions": []  # This will be filled during the simulation
            })
        JSONFile["numRobots"] = len(self.lista_robots)

        self.datacollector = DataCollector(
        )

        # Initial Packages
        JSONFile["crates"] = []
        for id, pos in enumerate(posiciones_entrada):
            if len(self.lista_paquetes) < len(self.lista_estanterias):
                paquete = Paquete(self.paquete_counter, self)
                self.grid.place_agent(paquete, pos)
                self.lista_entrada[id].paquete = paquete
                self.lista_paquetes.append(paquete)
                self.paquete_counter = self.paquete_counter + 1
                #JSONFile["crates"].append({"id": id, "targetPositions": [{"x": pos[0], "y": 1, "z": pos[1]}]})
                JSONFile["crates"].append({
                    "id": id,
                    "initialPosition": {"x": pos[0], "y": 1, "z": pos[1]},
                    "targetPositions": []
                })
        JSONFile["numCrates"] = len(self.lista_paquetes)

    def build_graph(self):
        graph = nx.Graph()

        for cell in self.grid.coord_iter():
            cell_content, pos = cell
            if not (any(isinstance(obj, (Estanteria, BandaEntrada, BandaSalida, Recarga, RobotCarga)) for obj in cell_content)):
                neighbors = self.grid.get_neighborhood(
                    pos,
                    moore=False,
                    include_center=True
                )
                for neighbor in neighbors:
                    neighbor_content = self.grid.get_cell_list_contents(neighbor)
                    if not (any(isinstance(obj, (Estanteria, BandaEntrada, BandaSalida, Recarga, RobotCarga)) for obj in neighbor_content)):
                        graph.add_edge(pos, neighbor)
        
        return graph
    
    # def generate_json(self):
    #     with open("output.json", 'w') as json_file:
    #         json.dump(JSONFile, json_file, indent=2)

    def generate_json(self):
        #with open("output.json", 'w') as json_file:
            #json.dump(JSONFile, json_file, indent=2)
        return JSONFile
    
    def step(self):
        self.steps_package = self.steps_package + 1
        self.steps_package_leave = self.steps_package_leave + 1
        self.graph = self.build_graph()
        if(self.steps_package >= self.steps_per_package and len(self.lista_paquetes) < len(self.lista_estanterias)):
            lista_entrada_vacia = []
            for BandaEntrada in self.lista_entrada:
                if(BandaEntrada.paquete == None):
                    lista_entrada_vacia.append(BandaEntrada)
            if(len(lista_entrada_vacia) != 0):
                rand_int = random.randint(0, len(lista_entrada_vacia) - 1)
                BandaEntrada = lista_entrada_vacia[rand_int]
                paquete = Paquete(self.paquete_counter, self)
                BandaEntrada.paquete = paquete
                self.grid.place_agent(paquete, BandaEntrada.pos)
                self.lista_paquetes.append(paquete)
                self.steps_package = 0
                self.paquete_counter = self.paquete_counter + 1
                targetPositionsNull = []
                for step in range(self.schedule.steps):
                    targetPositionsNull.append({"x": -1, "y": -1, "z": -1})
                JSONFile["crates"].append({"id": paquete.unique_id, "targetPositions": targetPositionsNull})
                
        if(self.steps_package_leave >= self.steps_leave_package):
            lista_paquetes_libres = []
            for paquete in self.lista_paquetes:
                if(paquete.lugar != "Estanteria" and paquete.lugar != "Salida"):
                    lista_paquetes_libres.append(paquete)
            if(len(lista_paquetes_libres) != 0):
                rand_int = random.randint(0, len(self.lista_salida) - 1)
                BandaSalida = self.lista_salida[rand_int]
                rand_int = random.randint(0, len(lista_paquetes_libres) - 1)
                paquete = lista_paquetes_libres[rand_int]
                paquete.lugar = "Salida"
                self.lista_ordenes.append([BandaSalida, paquete, "Libre"])
                self.steps_package_leave = 0


        self.datacollector.collect(self)

        self.schedule.step()

        
        # for robot in self.lista_robots:
        #     JSONFile["robots"][robot.unique_id]["targetPositions"].append({"x": robot.pos[0], "y": 0, "z": robot.pos[1]})

        # for paquete in self.lista_paquetes:
        #     JSONFile["paquetes"][paquete.unique_id]["targetPositions"].append({"x": paquete.pos[0], "y": 1, "z": paquete.pos[1]})
        for robot in self.lista_robots:
            JSONFile["robots"][robot.unique_id]["targetPositions"].append(
                {"x": robot.pos[0], "y": 0, "z": robot.pos[1]}
            )

        for paquete in self.lista_paquetes:
            JSONFile["crates"][paquete.unique_id]["targetPositions"].append(
                {"x": paquete.pos[0], "y": 1, "z": paquete.pos[1]}
            )