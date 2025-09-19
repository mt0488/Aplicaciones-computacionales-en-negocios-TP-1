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

def get_simulation_averages(sh, sim_id):
  x = [{"plane_id": i, 
      "start_time": sh[i][0][0],
      "end_time": sh[i][-1][0],
      "minutes_congested": sh[i][-1][4],
      "reposition_count": sh[i][-1][5],
      "diverted": int(sh[i][-1][6] == "diverted"),
      "status": sh[i][-1][6],
      } for i in range(len(sh))]

  df = pd.DataFrame(x)
  df["flight_time"] = df["end_time"] - df["start_time"]
  mask = df["status"].isin(["landed"])
  return {
    "simulation_id": sim_id,
    "mean_reposition": df["reposition_count"].mean(),
    "mean_congestion_all": df["minutes_congested"].mean(),
    "mean_congestion_landed": (df.loc[mask, "minutes_congested"]).mean(),
    "mean_flight_time_all": df["flight_time"].mean(),
    "mean_flight_time_landed": (df.loc[mask, "flight_time"]).mean(),
    "mean_diverted": df["diverted"].mean()
  }

def generate_simulation_dataframe(simulation_history):
  ls = []
  for i, s in enumerate(simulation_history):
    ls.append(get_simulation_averages(s, i))
  return pd.DataFrame(ls)

def update_speed(v, low, high):
    return max(low, min(high, v))

class Plane:
    def __init__(self,
    MAX_RADAR_DISTANCE: int = 100,
    MIN_THRESHOLD: int = 4,
    MIN_BUF: int = 5,
    MAX_DEVIATION_SPEED: int = 200,
    MIN_DEVIATION_SPEED: int = 150,
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
    minutes_from_start: int = 0
    ):
        self.MAX_RADAR_DISTANCE = MAX_RADAR_DISTANCE
        self.MIN_THRESHOLD = MIN_THRESHOLD
        self.MIN_BUF = MIN_BUF
        self.MAX_DEVIATION_SPEED = MAX_DEVIATION_SPEED
        self.MIN_DEVIATION_SPEED = MIN_DEVIATION_SPEED
        self.DT = DT
        self.DATE = DATE
        self.PROXIMITY_RANGE = PROXIMITY_RANGE
        self.LOWER_BOUNDS = LOWER_BOUNDS
        self.UPPER_BOUNDS = UPPER_BOUNDS
        self.MAX_SPEEDS = MAX_SPEEDS
        self.MIN_SPEEDS = MIN_SPEEDS
        self.PROP_BOUNCE = PROP_BOUNCE

        self.pos: int = MAX_RADAR_DISTANCE  # MN a AEP
        self.range_idx: int = 1 # PROXIMITY_RANGE idx
        self.dist_range: Tuple[int, int] = PROXIMITY_RANGE[1][0]
        self.speed_range: Tuple[int, int] = PROXIMITY_RANGE[1][1]
        self.speed: float = PROXIMITY_RANGE[1][1][1]  # knots
        self.dir: int = -1 # -1 towards AEP, 1 away from AEP
        self.id: Optional[int] = None
        self.status: str = "on-schedule" # on-schedule | delayed | diverted | landed
        self.landed: bool = False
        self.sta: datetime = None  # Scheduled Time of Arrival
        self.eta: datetime = None  # Estimated Time of Arrival
        self.ata: datetime = None  # Actual Time of Arrival
        self.dt: float = DT  # step size
        self.adjusting_speed: bool = False
        self.minutes_congested: int = 0
        self.reposition_count: int = 0
        self.history: List[Tuple[float, float, int, int, int, int, str]] = [(minutes_from_start, MAX_RADAR_DISTANCE, self.speed, -1, 0, 0, "on-schedule")]


    def minutes_from_MVD(self):
        """
        Returns minutes from MVD from current position assuming max speed can be achieved at every range.
        """
        minutes = 0
        x = self.pos
        if x < self.MAX_RADAR_DISTANCE:
            minutes += ((self.MAX_RADAR_DISTANCE - self.pos)/ self.MAX_DEVIATION_SPEED) * 60
            x = self.MAX_RADAR_DISTANCE
        minutes += ((31/200 + 5/150) * 60)
        return minutes
    
    def find_range_idx(self, x: float) -> int:
        """Returns current distance range index"""
        for i, (dist_range, _) in enumerate(self.PROXIMITY_RANGE):
            dmin, dmax = dist_range
            if dmin <= x < dmax:
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

    def calc_gap(self, x: float) -> float:
        s = self.speed
        if self.dir == -1 and not (self.range_idx == len(self.MAX_SPEEDS) - 1):
            if x < self.LOWER_BOUNDS[self.range_idx]:
                curr_to_lower = self.pos - self.LOWER_BOUNDS[self.range_idx]
                lower_to_next = self.LOWER_BOUNDS[self.range_idx] - x
                return (abs(curr_to_lower/self.speed + lower_to_next/self.MAX_SPEEDS[self.range_idx + 1])) * 60
        return (abs(self.pos - x)/s) * 60

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
            if timedelta(minutes=(self.calc_gap(positions[idx - 1]))) < delta:
                return -1
        # Check if previous plane is 5 mins behind
        if idx < len(positions):
            if timedelta(minutes=(self.calc_gap(positions[idx]))) < delta:
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
        self.pos += self.speed * self.dt * self.dir
        t0 = datetime(*self.DATE)  # 2025-09-10 06:00:00
        minutes_since_start = (now - t0).total_seconds() / 60.0
        if self.pos <= 0:
            self.pos = 0
            will_bounce = np.random.uniform(0, 1) # deal with landing interruptions
            if not airport_open:
                self.status = "diverted"
                self.minutes_congested += self.minutes_from_MVD()
                self.dir = 1
            else:
                if will_bounce < self.PROP_BOUNCE:
                    self.dir = 1
                    self.speed = self.MAX_DEVIATION_SPEED
                    self.status = "bounced"
                else:    
                    self.status = "landed"
                    self.landed = True
                    self.ata = now
                    self.eta = now
            self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.minutes_congested, self.reposition_count, self.status))
            return

        # Update ranges
        if self.dir == -1:
            self.update_ranges()

        self.minutes_congested += int((self.speed < self.MAX_SPEEDS[self.range_idx]) or (self.dir == 1))
        self.status = "delayed" if now > self.sta else "on-schedule"

        if self.pos > self.MAX_RADAR_DISTANCE:
            self.status = "diverted"
            self.minutes_congested += self.minutes_from_MVD()

        self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.minutes_congested, self.reposition_count, self.status))

    def update(self, now: datetime, plane_list: List['Plane'], reposition_list: List['Plane'], plane_idx: int) -> Dict[str, Any]:

        t0 = datetime(*self.DATE)  # 2025-09-10 06:00:00
        minutes_since_start = (now - t0).total_seconds() / 60.0
        if self.status == "diverted":
            return {"action": "divert", "status": self.status, "idx": -10}
        if self.status == "bounced":
            self.status = "repositioning"
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
                # Agregamos el tiempo estimado de llegada a MVD como minutos donde el avion sufre congestion/retraso
                # dado que no formaba parte de su plan de vuelo
                self.minutes_congested += self.minutes_from_MVD()
                self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.minutes_congested, self.reposition_count, self.status))
                return {"action": "divert", "status": self.status, "idx": -10}
            elif idx == -1:
                # Handle cases when plane doesn't find a gap and has to remain in the reposition queue
                action = "none"
                if plane_idx > 0:
                    time_gap = self.calc_gap(reposition_list[plane_idx - 1].pos)
                    if time_gap < self.MIN_THRESHOLD: self.speed = reposition_list[plane_idx - 1].speed - 20
                    if self.speed < self.MIN_DEVIATION_SPEED:
                        self.status = "diverted"
                        # Idem if idx == -10
                        self.minutes_congested += self.minutes_from_MVD()
                        self.history.append((minutes_since_start, self.pos, self.speed, self.dir, self.minutes_congested, self.reposition_count, self.status))
                        action = "divert"
                return {"action": action, "status": self.status, "idx": -1} 
            else:
                self.dir = -1
                self.adjusting_speed = False
                self.update_ranges()
                self.speed = self.speed_range[1]
                self.status = "on-schedule"
                return {"action": "insert", "status": self.status, "idx": idx}

        # Moving towards AEP and self is not first plane
        if 0 < plane_idx < len(plane_list):
            next_plane = plane_list[plane_idx - 1]
            time_gap = self.calc_gap(next_plane.pos)

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
                self.speed = self.MAX_DEVIATION_SPEED
                self.reposition_count += 1
                self.status = "repositioning"
                return {"action": "reposition", "status": self.status, "idx": -1}

        return {"action": "none", "status": self.status, "idx": -1}

class Handler:
    def __init__(self, 
    LAMBDA: float = 0.1,
    SIMULATION_TIME: int = 18 * 60,
    DATE: Tuple = (2025, 9, 10, 6, 0, 0),
    CLOSING_TIME: bool = False,
    N_ITERS: int = 1,
    AIRPORT_OPEN: bool = True,
    SAVE_HISTORY: bool = False, 
    MAX_RADAR_DISTANCE: int = 100,
    MIN_THRESHOLD: int = 4,
    MIN_BUF:int = 5,
    MAX_DEVIATION_SPEED:int = 200,
    MIN_DEVIATION_SPEED:int = 150,
    DT: float = 1 / 60, 
    PROXIMITY_RANGE: List = [((100, 10_000_000), (300, 500)),
                            ((50, 100), (250, 300)),
                            ((15, 50), (200, 250)),
                            ((5, 15), (150, 200)),
                            ((0, 5), (120, 150)),],
    LOWER_BOUNDS: List = [100, 50, 15, 5, 0],
    UPPER_BOUNDS: List = [10_000_000,100,50,15,5],
    MAX_SPEEDS: List = [500, 300, 250, 200, 150],
    MIN_SPEEDS: List = [300, 250, 200, 150, 120],
    PROP_BOUNCE: float = 0
    ):
        plane_params = {
                    "MAX_RADAR_DISTANCE": MAX_RADAR_DISTANCE,
                    "MIN_THRESHOLD": MIN_THRESHOLD,
                    "MIN_BUF": MIN_BUF,
                    "MAX_DEVIATION_SPEED": MAX_DEVIATION_SPEED,
                    "MIN_DEVIATION_SPEED": MIN_DEVIATION_SPEED,
                    "DT": DT, 
                    "PROXIMITY_RANGE": PROXIMITY_RANGE,
                    "LOWER_BOUNDS": LOWER_BOUNDS,
                    "UPPER_BOUNDS": UPPER_BOUNDS,
                    "MAX_SPEEDS": MAX_SPEEDS,
                    "MIN_SPEEDS": MIN_SPEEDS,
                    "PROP_BOUNCE": PROP_BOUNCE 
        }
        self.SAVE_HISTORY = SAVE_HISTORY
        self.LAMBDA = LAMBDA
        self.SIMULATION_TIME = SIMULATION_TIME
        self.DATE = DATE
        self.CLOSING_TIME = CLOSING_TIME
        self.N_ITERS = N_ITERS
        self.AIRPORT_OPEN = AIRPORT_OPEN
        self.plane_params = plane_params
        
    def create_plane(self, now: datetime) -> 'Plane':
        min_from_start = (now - datetime(*self.DATE)).total_seconds() // 60
        p = Plane(**self.plane_params, minutes_from_start=min_from_start)
        p.sta = now + timedelta(minutes= ((50/300 + 35/250 + 10/200 + 5/150) * 60))
        p.eta = p.sta
        return p

    def sort_incoming(self, incoming_planes: List['Plane']) -> None:
        incoming_planes.sort(key=lambda x: (x.pos is None, x.pos))

    def simulate(self) -> Dict[str, Any]:
        results: List[Dict] = []
        tracker: List[Dict[str, Any]] = [] if self.SAVE_HISTORY else None
        simulation_stats: List[Dict[str, Any]] = []
            
        for sim in tqdm(range(self.N_ITERS), desc="Simulation:"):
            now = datetime(*self.DATE)
            all_planes: List[Plane] = []
            incoming_planes: List[Plane] = []
            repositioning_planes: List[Plane] = []

            landed_count = 0
            diverted_count = 0
            reposition_count = 0

            closed_interval = 18 * 60
            if self.CLOSING_TIME:
                times = [i for i in range(1080)]
                closed_interval = random.choice(times)
                
            for t in range(self.SIMULATION_TIME):
                if closed_interval < self.SIMULATION_TIME:
                    if closed_interval < t <= closed_interval + 30:
                        self.AIRPORT_OPEN = False
                    else:
                        self.AIRPORT_OPEN = True
                        
                for p in all_planes:
                    if (not p.landed) and (p.status != "diverted"): # only on-air planes are updated
                        p.tick(now, self.AIRPORT_OPEN)

                if np.random.uniform(0, 1) <= self.LAMBDA:
                    p = self.create_plane(now)
                    all_planes.append(p)
                    incoming_planes.append(p)


                # Sanity check
                # self.sort_incoming(incoming_planes)

                i = 0
                while i < len(incoming_planes):
                    p = incoming_planes[i]

                    if p.landed or p.status == "landed":
                        landed_count += 1
                        incoming_planes.pop(i)
                        # no update i bc removed element
                        continue

                    res = p.update(now, incoming_planes, repositioning_planes, i)
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
                    res = p.update(now, incoming_planes, repositioning_planes, j)
                    action = res.get("action", "none")

                    if action == "insert":
                        idx = res.get("idx", None)
                        if idx is not None and idx >= 0:
                            if idx > len(incoming_planes):
                                idx = len(incoming_planes)
                            incoming_planes.insert(idx, p)
                            repositioning_planes.pop(j)
                        # self.sort_incoming(incoming_planes) # Sanity check
                        # no update j bc removed element
                        continue

                    if action == "divert":
                        diverted_count += 1
                        repositioning_planes.pop(j)
                        # no update j bc removed element
                        continue

                    j += 1

                now += timedelta(minutes=1)

            hist = [p.history for p in all_planes]

            if self.SAVE_HISTORY:
                tracker.append({
                    "simulation_id": sim,
                    "plane_history": hist
                })
            results.append({
                "simulation_id": sim,
                "landed": landed_count,
                "diverted": diverted_count,
                "reposition_count": reposition_count,
                "congestion_count": sum([(hist[k][-1][4] > 0) for k in range(len(hist))]),
                "total_planes": len(all_planes)
            })

            simulation_stats.append(get_simulation_averages(hist, sim))
                

        return pd.DataFrame(results), simulation_stats, tracker
    