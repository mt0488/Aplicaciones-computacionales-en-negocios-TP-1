import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any
from datetime import datetime, timedelta, time
from tqdm import tqdm
import random
import bisect
import sys
import pygame

def update_speed(v, low, high):
    return max(low, min(high, v))

class Plane:
    def __init__(self,
    MAX_RADAR_DISTANCE: int = 100,
    MIN_THRESHOLD: int = 4,
    MIN_BUF: int = 5,
    DEVIATION_SPEED: int = 200,
    DT: float = 1 / 60,
    DATE: Tuple = (2025, 9, 10, 6, 0, 0),     
    PROXIMITY_RANGE: List[Tuple[Tuple[int, int], Tuple[int, int]]] = [
                    ((100, 10_000_000), (300, 500)),
                    ((50, 100), (250, 300)),
                    ((15, 50), (200, 250)),
                    ((5, 15), (150, 200)),
                    ((0, 5), (120, 150)),
                    ],
    LOWER_BOUNDS: List[int] = [100, 50, 15, 5, 0],
    UPPER_BOUNDS: List[int] = [10_000_000,100,50,15,5],
    MAX_SPEEDS: List[int]  = [500, 300, 250, 200, 150],
    MIN_SPEEDS: List[int] = [300, 250, 200, 150, 120],
    PROP_BOUNCE: float = 0, # Probability to interrupt landing and go back to plane queue
    ):
        self.MAX_RADAR_DISTANCE = MAX_RADAR_DISTANCE
        self.MIN_THRESHOLD = MIN_THRESHOLD
        self.MIN_BUF = MIN_BUF
        self.DEVIATION_SPEED = DEVIATION_SPEED
        self.DT = DT
        self.DATE = DATE
        self.PROXIMITY_RANGE = PROXIMITY_RANGE
        self.LOWER_BOUNDS = LOWER_BOUNDS
        self.UPPER_BOUNDS = UPPER_BOUNDS
        self.MAX_SPEEDS = MAX_SPEEDS
        self.MIN_SPEEDS = MIN_SPEEDS
        self.PROP_BOUNCE = PROP_BOUNCE

        self.pos: int = MAX_RADAR_DISTANCE  # MN a AEP
        self.range_idx: int = 0 # PROXIMITY_RANGE idx
        self.dist_range: Tuple[int, int] = PROXIMITY_RANGE[0][0]
        self.speed_range: Tuple[int, int] = PROXIMITY_RANGE[0][1]
        self.speed: float = PROXIMITY_RANGE[0][1][1]  # knots
        self.dir: int = -1 # -1 towards AEP, 1 away from AEP
        self.id: Optional[int] = None
        self.status: str = "on-time" # on-time | delayed | diverted | landed
        self.landed: bool = False
        self.sta: datetime = None  # Scheduled Time of Arrival
        self.eta: datetime = None  # Estimated Time of Arrival
        self.ata: datetime = None  # Actual Time of Arrival
        self.dt: float = DT  # step size
        self.adjusting_speed: bool = False
        self.history: List[Tuple[float, float, int, int, str]] = []

    def find_range_idx(self, x: float) -> int:
        """Returns current distance range index"""
        for i, (dist_range, _) in enumerate(self.PROXIMITY_RANGE):
            dmin, dmax = dist_range
            if dmin <= x <= dmax:
                return i
            
        if x < 0:
            return len(self.PROXIMITY_RANGE) - 1
        return 0
    
    def update_ranges(self):
        """Updates ranges based on current position"""
        idx = self.find_range_idx(self.pos)
        self.range_idx = idx
        self.dist_range = self.PROXIMITY_RANGE[idx][0]
        self.speed_range = self.PROXIMITY_RANGE[idx][1]

        self.speed = update_speed(self.speed, self.speed_range[0], self.speed_range[1])


    def find_gap(self, plane_list: List['Plane']) -> int:
        """
        Returns index where self should be inserted in the plane queue.
        Return value respects 5 min buffer with next and previous plane.
        Returns -1 if no gap was found, -10 if plane is out of bounds
        """
        if self.pos > self.MAX_RADAR_DISTANCE:
            return -10

        if not plane_list or len(plane_list) == 0:
            # If plane_list is empty and plane is in radar distance, it can be inserted at first pos.
            return 0 if self.pos <= self.MAX_RADAR_DISTANCE else -1

        positions = [p.pos for p in plane_list]

        # If there are planes with no position, something went wrong previously
        if any(e is None for e in positions): return -1
        # If for some reason positions are not ordered, something went wrong previously
        if not all(positions[i] <= positions[i + 1] for i in range(len(positions) - 1)): return -1

        idx = bisect.bisect_left(positions, self.pos)

        delta = timedelta(minutes=self.MIN_BUF)

        # Check if next plane is 5 mins ahead
        if idx - 1 >= 0:
            if timedelta(minutes=(abs(self.pos - positions[idx - 1]) / self.speed) * 60) < delta:
                return -1
        # Check if previous plane is 5 mins behind
        if idx < len(positions):
            if timedelta(minutes=((positions[idx] - self.pos) / self.speed) * 60) < delta:
                return -1

        return idx

    def tick(self, now: datetime, airport_open: bool):
        """
        Avanza un tick:
          - Actualiza posición y estado (aterrizado si pos<=0)
          - Recalcula rangos y ETA
          - Actualiza status on-time/delayed según STA si está disponible
        """
        
        # Move
        if self.pos == 100 and not airport_open: # Handle airport closed and new airplane arriving
            self.status = "diverted"
            return
        
        self.pos += self.speed * self.dt * self.dir
        t0 = datetime(*self.DATE)  # 2025-09-10 06:00:00
        minutes_since_start = (now - t0).total_seconds() / 60.0
        if self.pos <= 0:
            self.pos = 0
            will_bounce = np.random.uniform(0, 1) # deal with landing interruptions
            if not airport_open: will_bounce = -1 # will always be < prop_bounce so arrving plane will bounce and go back to queue
            
            if will_bounce < self.PROP_BOUNCE:
                self.dir = 1
                self.speed = self.DEVIATION_SPEED
                self.status = "bounced"
            else:    
                self.status = "landed"
                self.landed = True
                self.ata = now
                self.eta = now
            self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.status))
            return

        # Update ranges and eta
        if self.dir == -1:
            self.update_ranges()

        self.status = "delayed" if now > self.sta else "on-time"
        self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.status))

    def update(self, plane_list: List['Plane'], plane_idx: int) -> Dict[str, Any]:
        """
        Lógica de espaciado/velocidad/desvío.
        NO modifica plane_list. Devuelve una acción para aplicar afuera:
          - {"action": "none"|"insert"|"divert", ...}
        """
        if self.status == "diverted":
            return {"action": "divert", "status": self.status, "idx": -10}
        
        if self.status == "bounced":
            self.status = "delayed"
            return {"action": "reposition", "status": self.status, "idx": -1}
        if self.landed:
            return {"action": "none", "status": self.status, "idx": -1}

        # Plane is moving away from AEP
        if self.dir == 1:
            if self.pos <= 5:
                return {"action":"none", "status": self.status, "idx": -1} 
            idx = self.find_gap(plane_list)
            if idx == -10:
                self.status = "diverted"
                return {"action": "divert", "status": self.status, "idx": -10}
            elif idx == -1:
                return {"action":"none", "status": self.status, "idx": -1} 
            else:
                self.dir = -1
                self.adjusting_speed = False
                self.speed = self.speed_range[1]
                return {"action": "insert", "status": self.status, "idx": idx}

        # Moving towards AEP and self is not first plane
        if 0 < plane_idx < len(plane_list) and plane_list[plane_idx - 1].eta is not None and self.eta is not None:
            next_plane = plane_list[plane_idx - 1]
            time_gap = ((self.pos - next_plane.pos) / self.speed) * 60

            if time_gap < self.MIN_THRESHOLD:
                self.adjusting_speed = True

            if self.adjusting_speed:
                if time_gap >= self.MIN_BUF:
                    self.adjusting_speed = False
                    self.speed = self.speed_range[1]
                else:
                    # keep safety speed until MIN_BUF is achieved
                    self.speed = next_plane.speed - 20
            else:
                # Updte speed to max permitted speed
                self.speed = self.speed_range[1]

            # If self fell out of min speed, 
            if self.speed < self.speed_range[0]:
                self.dir = 1
                self.adjusting_speed = False
                self.speed = self.DEVIATION_SPEED
                return {"action": "reposition", "status": self.status}

        return {"action": "none", "status": self.status}

class Handler:
    def __init__(self, 
    LAMBDA: float = 1/60,
    SIMULATION_TIME: int = 18 * 60,
    DATE: Tuple = (2025, 9, 10, 6, 0, 0),
    CLOSING_TIME: int = 18 * 60,
    N_ITERS: int = 1,
    AIRPORT_OPEN: bool = True, 
    plane_params: Dict = {
        "MAX_RADAR_DISTANCE": 100,
        "MIN_THRESHOLD": 4,
        "MIN_BUF": 5,
        "DEVIATION_SPEED": 200,
        "DT": 1 / 60,
        "DATE": (2025, 9, 10, 6, 0, 0),    
        "PROXIMITY_RANGE": [((100, 10_000_000), (300, 500)),
                            ((50, 100), (250, 300)),
                            ((15, 50), (200, 250)),
                            ((5, 15), (150, 200)),
                            ((0, 5), (120, 150)),],
        "LOWER_BOUNDS": [100, 50, 15, 5, 0],
        "UPPER_BOUNDS": [10_000_000,100,50,15,5],
        "MAX_SPEEDS": [500, 300, 250, 200, 150],
        "MIN_SPEEDS": [300, 250, 200, 150, 120],
        "PROP_BOUNCE": 0
        }
    ):
        self.LAMBDA = LAMBDA
        self.SIMULATION_TIME = SIMULATION_TIME
        self.DATE = DATE
        self.CLOSING_TIME = CLOSING_TIME
        self.N_ITERS = N_ITERS
        self.AIRPORT_OPEN = AIRPORT_OPEN
        self.plane_params = plane_params
        
    def create_plane(self, now: datetime) -> 'Plane':
        p = Plane(**self.plane_params)
        p.sta = now + timedelta(minutes= ((50/300 + 35/250 + 10/200 + 5/150) * 60))
        p.eta = p.sta
        return p

    def sort_incoming(self, incoming_planes: List['Plane']) -> None:
        old_incoming = incoming_planes.copy()
        incoming_planes.sort(key=lambda x: (x.pos is None, x.pos))
        # Sanity check, nunca puede haber un avion "mas adelante" pero atras en la fila
        # assert all(old_incoming[i] == incoming_planes[i] for i in range(len(old_incoming)))

    def simulate(self) -> Dict[str, Any]:
        results: Dict[str, Any] = {
            "simulations": []
        }
        tracker: Dict[str, Any] = {
            "simulations": []
        }
            
        for sim in range(self.N_ITERS):
            now = datetime(*self.DATE)
            all_planes: List[Plane] = []
            incoming_planes: List[Plane] = []
            repositioning_planes: List[Plane] = []

            landed_count = 0
            diverted_count = 0
            reposition_count = 0

            for t in range(self.SIMULATION_TIME):
                if self.CLOSING_TIME != 0:
                    if self.CLOSING_TIME < t <= self.CLOSING_TIME + 30:
                        self.AIRPORT_OPEN = False
                    else:
                        self.AIRPORT_OPEN = True
                        
                if np.random.uniform(0, 1) <= self.LAMBDA:
                    p = self.create_plane(now)
                    all_planes.append(p)
                    incoming_planes.append(p)

                for p in all_planes:
                    if (not p.landed) and (p.status != "diverted"): # only on-air planes are updated
                        p.tick(now, self.AIRPORT_OPEN)

                # Sanity check
                self.sort_incoming(incoming_planes)

                i = 0
                while i < len(incoming_planes):
                    p = incoming_planes[i]

                    if p.landed or p.status == "landed":
                        landed_count += 1
                        incoming_planes.pop(i)
                        # no update i bc removed element
                        continue

                    res = p.update(incoming_planes, i)
                    action = res.get("action", "none")

                    if action == "reposition": # plane has moved to repositioning queue
                        reposition_count += 1
                        repositioning_planes.append(p)
                        incoming_planes.pop(i)
                        # no update i bc removed element
                        continue

                    if action == "divert":
                        diverted_count += 1
                        incoming_planes.pop(i)
                        # no update i bc removed element
                        continue

                    i += 1

                j = 0
                while j < len(repositioning_planes):
                    p = repositioning_planes[j]
                    res = p.update(incoming_planes, -1)
                    action = res.get("action", "none")

                    if action == "insert":
                        idx = res.get("idx", None)
                        if idx is not None and idx >= 0:
                            if idx > len(incoming_planes):
                                idx = len(incoming_planes)
                            incoming_planes.insert(idx, p)
                            repositioning_planes.pop(j)
                        self.sort_incoming(incoming_planes) # Sanity check
                        # no update j bc removed element
                        continue

                    if action == "divert":
                        diverted_count += 1
                        repositioning_planes.pop(j)
                        # no update j bc removed element
                        continue

                    j += 1

                now += timedelta(minutes=1)

            results["simulations"].append({
                "simulation_id": sim,
                "landed": landed_count,
                "diverted": diverted_count,
                "reposition_count": reposition_count,
                "total_planes": len(all_planes)
            })
            tracker["simulations"].append({
                "simulation_id": sim,
                "plane_history": [p.history for p in all_planes]
            })

        return results, tracker
    