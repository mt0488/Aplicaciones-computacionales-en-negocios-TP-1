from __future__ import annotations

"""
Infraestructura base para simular arribos y aproximación en AEP (TP1 ACN)
-----------------------------------------------------------------------------
Objetivo: brindar una arquitectura clara y extensible para:
 - Generar arribos estocásticos (Bernoulli por minuto, parámetro lambda)
 - Simular la aproximación de 100 mn a 0 mn con rangos de velocidad por tramo
 - Aplicar reglas de separación temporal (mínimo 4', objetivo 5')
 - Reducir velocidad (20 nudos por debajo del líder) para mantener separación
 - Ejecutar desvíos (go-around/hold) y desvío a Montevideo según reglas
 - Medir métricas (atraso, throughput, % congestión, desvíos, etc.)
 - Visualizar el sistema (trayectorias en distancia-tiempo y timeline de eventos)

Decisiones de diseño:
 - Simulación discretizada por minuto (dt=1 min). Si se necesita más precisión,
   se puede pasar a dt=0.5 min sin romper la API.
 - Estados por avión: NEW, APPROACH, HOLDING, DIVERTED, LANDED.
 - Orquestador: Simulation que contiene Scheduler (arribos), ATC (control de
   separación/velocidad), World (tiempo, clima/eventos), Metrics, Visualizer.
 - Sin dependencias pesadas; solo numpy y matplotlib opcionales.

Este archivo funciona como módulo y script. Ver bloque __main__ al final.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
import numpy as np

# ======================== Configuración del dominio ========================= #

# Conversión de unidades
KTS_TO_NM_PER_MIN = 1.0 / 60.0  # 1 nudo = 1 mn/h = 1/60 mn/min

# Rango de velocidad permitido por tramo de distancia (mn)
# (lim_sup_exclusivo, (v_min, v_max))
SPEED_BRACKETS = [
    (100.0, (250, 300)),   # 100mn a 50mn -> luego se castea por función get_speed_range
    (50.0, (200, 250)),    # 50mn a 15mn
    (15.0, (150, 200)),    # 15mn a 5mn
    (5.0, (120, 150)),     # 5mn a 0
]
# Nota: Para >100 mn usamos (300, 500) solo si se inicia fuera, pero el enunciado
# fija el inicio a 100 mn. Mantenemos helper por completitud.
OUTER_SPEED_RANGE = (300, 500)

# Separación
MIN_SEP_MIN = 4  # mínimo de 4 minutos
TARGET_SEP_MIN = 5  # objetivo 5 minutos (buffer)

# Hold / desvío
HOLD_SPEED_KTS = 200  # velocidad a la que vuelan “hacia atrás/paralelo”
REJOIN_GAP_MIN = 10   # gap necesario para reinsertarse
REJOIN_MIN_DIST_MN = 5  # debe estar a > 5 mn de AEP para reinsertar

# Jornada operativa (minutos)
DAY_START_MIN = 6 * 60
DAY_END_MIN = 24 * 60  # medianoche
SIM_WINDOW_MIN = DAY_END_MIN - DAY_START_MIN

# ============================ Modelos de datos ============================== #

class PlaneState:
    NEW = "NEW"
    APPROACH = "APPROACH"
    HOLDING = "HOLDING"
    DIVERTED = "DIVERTED"
    LANDED = "LANDED"

@dataclass
class Plane:
    pid: int
    spawn_minute: int  # minuto absoluto del día en que aparece en 100 mn
    dist_nm: float = 100.0
    gs_kts: float = 0.0  # ground speed elegida
    state: str = PlaneState.NEW
    last_update_minute: int = field(default_factory=lambda: DAY_START_MIN)

    # Métricas
    congested_minutes: int = 0
    extra_delay_min: int = 0  # atraso vs. trayectoria sin congestión
    diverted: bool = False
    interrupted: bool = False  # toque frustrado / episodio ventoso

    # Tiempos relevantes
    t_approach_start: Optional[int] = None
    t_landing: Optional[int] = None
    t_divert: Optional[int] = None

    def __post_init__(self):
        self.state = PlaneState.APPROACH
        self.t_approach_start = self.spawn_minute
        self.last_update_minute = self.spawn_minute

# =============================== Utilidades ================================= #

def get_speed_range(dist_nm: float) -> Tuple[int, int]:
    """Devuelve (v_min, v_max) según tramo de distancia."""
    if dist_nm > 100:
        return OUTER_SPEED_RANGE
    if dist_nm > 50:
        return (250, 300)
    if dist_nm > 15:
        return (200, 250)
    if dist_nm > 5:
        return (150, 200)
    return (120, 150)

# Tiempo libre (gap) entre aterrizajes planeados en minutos

def landing_gap(planned_landings: List[int], t: int) -> int:
    """Devuelve el gap mínimo (en minutos) a t respecto de los aterrizajes planificados."""
    if not planned_landings:
        return 10**9
    diffs = [abs(t - x) for x in planned_landings]
    return min(diffs)

# ============================ Componentes core ============================== #

@dataclass
class ArrivalProcess:
    lam: float  # probabilidad de arribo por minuto (Bernoulli)
    rng: np.random.Generator

    def arrivals_at(self, minute_abs: int) -> int:
        # Bernoulli(λ) ⇒ 0/1 llegadas por minuto
        return int(self.rng.random() < self.lam)

@dataclass
class ATC:
    """Control separaciones y velocidades."""
    planned_landings: List[int] = field(default_factory=list)  # tiempos objetivo de toque

    def schedule_landing(self, t_candidate: int) -> int:
        """Ajusta el tiempo de aterrizaje para respetar MIN_SEP_MIN, apuntando a TARGET_SEP_MIN.
        Retorna el t_landing comprometido (planificado)."""
        if not self.planned_landings:
            self.planned_landings.append(t_candidate)
            return t_candidate
        # Si está muy cerca del último, empujar hacia adelante al menos a TARGET_SEP_MIN
        last = self.planned_landings[-1]
        if t_candidate - last < MIN_SEP_MIN:
            t_candidate = last + TARGET_SEP_MIN
        self.planned_landings.append(t_candidate)
        return t_candidate

    def can_rejoin(self, t_candidate: int) -> bool:
        # Debe existir gap de al menos REJOIN_GAP_MIN
        if not self.planned_landings:
            return True
        # Verificar con vecinos más cercanos
        ts = np.array(self.planned_landings)
        return np.all(np.abs(ts - t_candidate) >= REJOIN_GAP_MIN)

@dataclass
class Metrics:
    total_planes: int = 0
    landed: int = 0
    diverted: int = 0
    avg_delay_min: float = 0.0
    avg_congestion_ratio: float = 0.0

    # Para errores de estimación por replicación
    delays: List[float] = field(default_factory=list)
    congest_ratios: List[float] = field(default_factory=list)

    def consume_plane(self, p: Plane):
        self.total_planes += 1
        if p.state == PlaneState.LANDED and p.t_landing is not None:
            self.landed += 1
            self.delays.append(p.extra_delay_min)
            # proporción de minutos congestivos sobre minutos totales en tramo
            total_minutes = max(1, p.t_landing - p.spawn_minute)
            self.congest_ratios.append(p.congested_minutes / total_minutes)
        if p.diverted:
            self.diverted += 1

    def finalize(self):
        if self.delays:
            self.avg_delay_min = float(np.mean(self.delays))
        if self.congest_ratios:
            self.avg_congestion_ratio = float(np.mean(self.congest_ratios))

# =============================== Simulación ================================= #

@dataclass
class Simulation:
    lam: float
    seed: int = 123
    windy: bool = False           # p=0.1 interrupciones independientes por avión
    storm_closure: Optional[Tuple[int, int]] = None  # (t_start_abs, duration_min)
    dt_min: float = 1.0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.arrivals = ArrivalProcess(self.lam, self.rng)
        self.atc = ATC()
        self.metrics = Metrics()
        self.planes: List[Plane] = []
        self.clock = DAY_START_MIN
        self.active: List[Plane] = []

    # ------------------------- helpers de estado ---------------------------- #
    def _spawn_planes(self):
        n = self.arrivals.arrivals_at(self.clock)
        for _ in range(n):
            pid = len(self.planes) + 1
            p = Plane(pid=pid, spawn_minute=self.clock)
            self.planes.append(p)
            self.active.append(p)

    def _storm_active(self) -> bool:
        if self.storm_closure is None:
            return False
        t0, dur = self.storm_closure
        return t0 <= self.clock < (t0 + dur)

    def _apply_interruption(self, p: Plane) -> bool:
        if not self.windy:
            return False
        # 10% de chance una sola vez (cuando cruza 5 mn)
        if (not p.interrupted) and p.dist_nm <= REJOIN_MIN_DIST_MN:
            if self.rng.random() < 0.1:
                p.interrupted = True
                return True
        return False

    # --------------------------- física simplificada ------------------------ #
    def _free_flow_speed(self, p: Plane) -> int:
        vmin, vmax = get_speed_range(p.dist_nm)
        return vmax  # política base: usar la máxima permitida

    def _leader_landing_time(self) -> Optional[int]:
        # Último aterrizaje planificado (si existe)
        if not self.atc.planned_landings:
            return None
        return self.atc.planned_landings[-1]

    def _planned_t_landing_free(self, p: Plane, v_kts: int) -> int:
        # Estima tiempo de arribo a pista a velocidad constante v_kts
        minutes = int(np.ceil(p.dist_nm / (v_kts * KTS_TO_NM_PER_MIN)))
        return self.clock + minutes

    def _update_follow_rules(self, p: Plane, v_free: int) -> Tuple[int, bool]:
        """Devuelve (v_aplicada, en_congestion). Aplica 20 kts menos que líder si no
        logra respetar 4-5 min de separación respecto del aterrizaje previo.
        """
        t_free = self._planned_t_landing_free(p, v_free)
        last_land = self._leader_landing_time()
        if last_land is None:
            return v_free, False
        # Si muy cerca, bajar 20 kts hasta lograr TARGET_SEP_MIN
        if t_free - last_land < MIN_SEP_MIN:
            vmin, _ = get_speed_range(p.dist_nm)
            v_adj = max(vmin, v_free - 20)
            return v_adj, v_adj < v_free
        return v_free, False

    def _step_plane(self, p: Plane):
        if p.state in {PlaneState.DIVERTED, PlaneState.LANDED}:
            return

        if self._storm_active():
            # aeropuerto cerrado: entrar en holding a 200 kts alejándose
            p.state = PlaneState.HOLDING
            p.gs_kts = HOLD_SPEED_KTS
            p.dist_nm = min(120.0, p.dist_nm + HOLD_SPEED_KTS * KTS_TO_NM_PER_MIN * self.dt_min)
            p.congested_minutes += 1
            p.extra_delay_min += 1
            return

        # ¿interrupción ventosa cerca de pista?
        if self._apply_interruption(p):
            p.state = PlaneState.HOLDING

        if p.state == PlaneState.HOLDING:
            # volar "hacia atrás / paralelo" a 200 kts hasta hallar gap >=10'
            p.gs_kts = HOLD_SPEED_KTS
            p.dist_nm = min(120.0, p.dist_nm + HOLD_SPEED_KTS * KTS_TO_NM_PER_MIN * self.dt_min)
            # Intentar reinsertar cuando haya gap suficiente y esté a >5 nm
            t_candidate = self.clock + int(np.ceil(p.dist_nm / (self._free_flow_speed(p) * KTS_TO_NM_PER_MIN)))
            if p.dist_nm > REJOIN_MIN_DIST_MN and self.atc.can_rejoin(t_candidate):
                p.state = PlaneState.APPROACH
            else:
                # Si excede 100 mn y no encuentra gap, se va a Montevideo
                if p.dist_nm > 100.0:
                    p.state = PlaneState.DIVERTED
                    p.diverted = True
                    p.t_divert = self.clock
            p.congested_minutes += 1
            p.extra_delay_min += 1
            return

        # APPROACH: intentar volar a velocidad libre, corrigiendo por separación
        v_free = self._free_flow_speed(p)
        v_applied, congested = self._update_follow_rules(p, v_free)
        vmin, vmax = get_speed_range(p.dist_nm)

        # Si para mantener separación habría que ir por debajo del mínimo permitido,
        # entrar en holding (go-around) y buscar reinsertar con gap >=10'
        if v_applied < vmin:
            p.state = PlaneState.HOLDING
            p.congested_minutes += 1
            p.extra_delay_min += 1
            return

        # Avanzar
        p.gs_kts = v_applied
        delta_nm = v_applied * KTS_TO_NM_PER_MIN * self.dt_min
        p.dist_nm = max(0.0, p.dist_nm - delta_nm)
        if congested:
            p.congested_minutes += 1
            p.extra_delay_min += 1

        # ¿aterrizó?
        if p.dist_nm <= 0.0:
            # programar aterrizaje con ATC respetando separación
            t_candidate = self.clock
            t_land = self.atc.schedule_landing(t_candidate)
            p.t_landing = t_land
            p.state = PlaneState.LANDED

    def step(self):
        # 1) Spawns
        self._spawn_planes()
        # 2) Avanzar cada avión activo
        for p in list(self.active):
            self._step_plane(p)
            if p.state in {PlaneState.DIVERTED, PlaneState.LANDED}:
                self.metrics.consume_plane(p)
                self.active.remove(p)
        # 3) Clock
        self.clock += int(self.dt_min)

    def run(self, minutes: int = SIM_WINDOW_MIN):
        end_t = DAY_START_MIN + minutes
        while self.clock < end_t:
            self.step()
        self.metrics.finalize()
        return self.metrics

# =========================== Visualización simple =========================== #

def demo_visual(lam: float, seed: int = 1, windy: bool = False, storm: Optional[Tuple[int,int]] = None):
    """Corre una simulación y grafica distancias de cada avión en función del tiempo.
    (Esquema rápido para validar la infraestructura.)
    """
    import matplotlib.pyplot as plt

    sim = Simulation(lam=lam, seed=seed, windy=windy, storm_closure=storm)

    # Traza: registraremos (t, dist) por avión
    traces: Dict[int, List[Tuple[int, float]]] = {}

    end_t = DAY_START_MIN + SIM_WINDOW_MIN
    while sim.clock < end_t:
        # registrar estado actual antes del step
        for p in sim.active:
            traces.setdefault(p.pid, []).append((sim.clock, p.dist_nm))
        sim.step()

    # Plot
    plt.figure(figsize=(10, 6))
    for pid, td in traces.items():
        if not td:
            continue
        t, d = zip(*td)
        plt.plot(np.array(t) - DAY_START_MIN, d, linewidth=1)
    plt.axhline(5.0, linestyle='--', linewidth=1, label='5 mn')
    plt.gca().invert_yaxis()
    plt.xlabel('Minutos desde 06:00')
    plt.ylabel('Distancia a AEP (mn)')
    plt.title(f'Trayectorias en aproximación (λ={lam})')
    plt.legend()
    plt.tight_layout()
    plt.show()

# ================================ Main ===================================== #

if __name__ == "__main__":
    # Ejemplos de ejecución mínima para validar:
    # 1) Día normal, λ=0.1 (≈ 6 aviones/h en Bernoulli por minuto)
    m = Simulation(lam=0.1, seed=42).run(minutes=6*60)  # 6 horas
    print({
        'total_planes': m.total_planes,
        'landed': m.landed,
        'diverted': m.diverted,
        'avg_delay_min': round(m.avg_delay_min, 2),
        'avg_congestion_ratio': round(m.avg_congestion_ratio, 3),
    })

    # 2) Visual rápido
    demo_visual(lam=0.1, seed=7)
