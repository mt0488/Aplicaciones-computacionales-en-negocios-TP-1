import sys
import io
import pygame

# Si querés usar SVG: pip install cairosvg
try:
    import cairosvg
    HAVE_CAIROSVG = True
except Exception:
    HAVE_CAIROSVG = False


def visualize(
    history,
    sim_minutes_per_second=10.0,
    sim_time_minutes=18*60,
    fps=60,
    use_svg=True,
    AIRCRAFT_SVG_PATH="aircraft.svg",   # sprite base (facing RIGHT recomendado, pero no imprescindible)
    AIRCRAFT_PNG_PATH="aircraft.png",   # alternativa PNG
    SPRITE_PX_WIDTH=36,                 # ancho del sprite
):
    """
    history: list[list[tuple(min_from_start, pos_nm, speed_knots, dir, status)]]
    sim_minutes_per_second: minutos simulados por segundo real
    sim_time_minutes: duración total de la simulación (p. ej. 18*60)
    fps: frames por segundo
    use_svg: True->SVG con cairosvg, False->PNG
    """

    # ==================== CONSTANTES / MAPEOS ====================
    WIDTH, HEIGHT = 1500, 800
    NM_TO_PX = 10
    START_X = 100  # x(100 NM)

    def x_from_nm(pos_nm: float) -> int:
        """x = 100px en 100 NM y 'desciende' hacia la derecha (menor NM -> mayor x)."""
        return int(round(START_X + (100.0 - float(pos_nm)) * NM_TO_PX))

    # Marcas (0, 5, 15, 50, 100 NM)
    MARKS = [0, 5, 15, 50, 100]
    marks_px = {f"{nm}NM": x_from_nm(nm) for nm in MARKS}

    # ==================== PREPROCESO: ARRIBOS ====================
    # Avión "aparece" al minuto de su primer estado
    arrivals_by_minute = {m: [] for m in range(sim_time_minutes + 1)}
    for idx, hist in enumerate(history):
        if not hist:
            continue
        t0 = int(hist[0][0])
        if 0 <= t0 <= sim_time_minutes:
            arrivals_by_minute[t0].append(idx)

    # ==================== PYGAME INIT ====================
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("AEP Traffic")
    clock = pygame.time.Clock()

    pygame.font.init()
    FONT = pygame.font.SysFont(None, 18)
    CLOCK_FONT = pygame.font.SysFont(None, 28)  # ← NUEVO: fuente para la hora


    # Colores
    BACKGROUND = (26, 39, 110)
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED   = (255, 0, 0)

    # Botón pausa
    BTN_SIZE = 60
    BTN_X, BTN_Y = 10, HEIGHT - BTN_SIZE - 10
    button_rect = pygame.Rect(BTN_X, BTN_Y, BTN_SIZE, BTN_SIZE)

    # Fila única para todos los aviones
    LANE_Y = HEIGHT // 2
    BUFFER_RECT_H = 22

    # ==================== CARGA SPRITES ====================
    def surface_from_svg(path, target_w):
        if not HAVE_CAIROSVG:
            raise RuntimeError("use_svg=True pero cairosvg no está disponible.")
        png_bytes = cairosvg.svg2png(url=path, output_width=target_w)
        img = pygame.image.load(io.BytesIO(png_bytes)).convert_alpha()
        return img

    def surface_from_png(path, target_w):
        img = pygame.image.load(path).convert_alpha()
        w, h = img.get_width(), img.get_height()
        scale = target_w / float(w)
        return pygame.transform.smoothscale(img, (int(target_w), int(h * scale)))

    def tint_surface(src, color):
        out = src.copy()
        tint = pygame.Surface(out.get_size(), pygame.SRCALPHA)
        tint.fill(color + (255,))
        out.blit(tint, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        return out

    # Carga base y genera right/left + red
    if use_svg:
        base = surface_from_svg(AIRCRAFT_SVG_PATH, SPRITE_PX_WIDTH)
    else:
        base = surface_from_png(AIRCRAFT_PNG_PATH, SPRITE_PX_WIDTH)

    # No asumimos orientación del archivo; generamos ambas
    SURF_RIGHT = base
    SURF_LEFT  = pygame.transform.flip(base, True, False)
    SURF_RIGHT_RED = tint_surface(SURF_RIGHT, RED)
    SURF_LEFT_RED  = tint_surface(SURF_LEFT,  RED)

    # ==================== ESTRUCTURAS EN EJECUCIÓN ====================
    # planes[idx] -> dict con rect, tip_x flotante y cache de surfaces para el draw
    planes = {}
    # estado por avión (puntero al historial + estado actual resumido)
    plane_state = {}  # idx -> {"hist_pos": int, "active": bool}

    # tiempo de simulación (float) y control de llegadas
    sim_minutes = 0.0
    last_processed_minute = -1
    paused = False
    running = True

    # ==================== HELPERS DE DIBUJO ====================
    def draw_dashed_vline(surf, x, y0, y1, color, dash_len=12, gap_len=8, thickness=1):
        if x < 0 or x >= WIDTH:
            return
        y = y0
        while y < y1:
            y_end = min(y + dash_len, y1)
            pygame.draw.line(surf, color, (x, y), (x, y_end), thickness)
            y = y_end + gap_len

    def draw_dashed_rect(surf, rect, color, dash_len=10, gap_len=6, thickness=1):
        x, y, w, h = rect
        # Top
        cx = x
        while cx < x + w:
            seg = min(dash_len, x + w - cx)
            pygame.draw.line(surf, color, (cx, y), (cx + seg, y), thickness)
            cx += dash_len + gap_len
        # Bottom
        cx = x
        yb = y + h
        while cx < x + w:
            seg = min(dash_len, x + w - cx)
            pygame.draw.line(surf, color, (cx, yb), (cx + seg, yb), thickness)
            cx += dash_len + gap_len
        # Left
        cy = y
        while cy < y + h:
            seg = min(dash_len, y + h - cy)
            pygame.draw.line(surf, color, (x, cy), (x, cy + seg), thickness)
            cy += dash_len + gap_len
        # Right
        cy = y
        xr = x + w
        while cy < y + h:
            seg = min(dash_len, y + h - cy)
            pygame.draw.line(surf, color, (xr, cy), (xr, cy + seg), thickness)
            cy += dash_len + gap_len

    # ==================== LOOP PRINCIPAL ====================
    while running:
        # Eventos / pausa
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if button_rect.collidepoint(event.pos):
                    paused = not paused

        # Avance del tiempo simulado
        delta_minutes = 0.0
        if not paused and sim_minutes < sim_time_minutes:
            delta_minutes = sim_minutes_per_second / float(fps)
            sim_minutes = min(sim_minutes + delta_minutes, sim_time_minutes)

        current_minute = int(sim_minutes)

        # Llegadas (procesamos todos los minutos cruzados)
        while last_processed_minute < current_minute and last_processed_minute < sim_time_minutes:
            last_processed_minute += 1
            for idx in arrivals_by_minute.get(last_processed_minute, []):
                # Posición inicial según su estado en current_minute (o su primero)
                plane_state[idx] = {"hist_pos": 0, "active": True}

                # Avanzar hist_pos hasta el último estado con t ≤ current_minute
                hist = history[idx]
                while (plane_state[idx]["hist_pos"] + 1 < len(hist)
                       and hist[plane_state[idx]["hist_pos"] + 1][0] <= current_minute):
                    plane_state[idx]["hist_pos"] += 1

                # Construir rect en fila única con la nariz “anclada” a x_from_nm(pos)
                t0, pos0, speed0, dir0, min_congested0, rep_count0, _ = hist[plane_state[idx]["hist_pos"]]
                # Extrapolar minutos desde ese estado
                dt_min = max(0.0, sim_minutes - float(t0))
                # Δpos en NM: dir=-1 reduce la distancia (hacia AEP)
                pos_now = float(pos0) + (float(speed0) * (dt_min / 60.0)) * (1 if int(dir0) == 1 else -1)
                x_tip = x_from_nm(pos_now)

                # Elegir sprite (dir=-1 → RIGHT; dir=1 → LEFT)
                if int(dir0) == -1:
                    surf = SURF_RIGHT
                    rect = surf.get_rect()
                    rect.midleft = (x_tip, LANE_Y)   # nariz a la izquierda del rect
                else:
                    surf = SURF_LEFT
                    rect = surf.get_rect()
                    rect.midright = (x_tip, LANE_Y)  # nariz a la derecha del rect

                planes[idx] = {"rect": rect, "tip_x": float(x_tip)}

        # Actualización por frame de todos los activos
        if delta_minutes > 0.0:
            to_remove = []
            for idx, pdata in planes.items():
                st = plane_state.get(idx)
                if not st or not st["active"]:
                    continue

                hist = history[idx]

                # Avanzar hist_pos si hay nuevos estados ≤ current_minute
                while (st["hist_pos"] + 1 < len(hist)
                       and hist[st["hist_pos"] + 1][0] <= current_minute):
                    st["hist_pos"] += 1

                # Estado vigente
                t_now, pos_nm, speed_knots, dir_now, cong_count_now, rep_count_now, status_now = hist[st["hist_pos"]]
                dir_now = int(dir_now)
                status = (status_now or "").lower()

                # Extrapolar posición a tiempo exacto (minutos fraccionales)
                dt_min = max(0.0, sim_minutes - float(t_now))
                pos_now = float(pos_nm) + (float(speed_knots) * (dt_min / 60.0)) * (1 if dir_now == 1 else -1)
                x_tip = x_from_nm(pos_now)
                pdata["tip_x"] = x_tip

                # Aterrizado o desviado: lo removemos (si preferís dejar “fantasma”, no lo borres)
                if status in ("landed", "diverted"):
                    to_remove.append(idx)
                    continue

                # Reposicionamiento: recuadro rojo si dir==1 o status == "bounced"
                is_repositioning = (dir_now == 1) or (status == "bounced")

                # Colocar rect según dir manteniendo nariz en x_tip
                if dir_now == -1:
                    rect = SURF_RIGHT.get_rect()
                    rect.midleft = (int(round(x_tip)), LANE_Y)
                else:
                    rect = SURF_LEFT.get_rect()
                    rect.midright = (int(round(x_tip)), LANE_Y)

                pdata["rect"] = rect
                pdata["is_repo"] = is_repositioning
                pdata["speed"] = float(speed_knots)
                pdata["dir"] = dir_now

            for idx in to_remove:
                planes.pop(idx, None)
                plane_state.pop(idx, None)

        # ==================== DIBUJO ====================
        screen.fill(BACKGROUND)

        # Líneas de tramo: sólida en 0; punteadas en 5, 15, 50, 100
        pygame.draw.line(screen, WHITE, (marks_px["0NM"], 0), (marks_px["0NM"], HEIGHT), 2)
        for key in ["5NM", "15NM", "50NM", "100NM"]:
            draw_dashed_vline(screen, marks_px[f"{key}"], 0, HEIGHT, WHITE)

        # Botón pausa
        rect_color = WHITE if paused else BLACK
        tri_color  = BLACK if paused else WHITE
        pygame.draw.rect(screen, rect_color, button_rect, border_radius=8)
        # ícono "play"
        left_x   = BTN_X + int(BTN_SIZE * 0.30)
        right_x  = BTN_X + int(BTN_SIZE * 0.70)
        top_y    = BTN_Y + int(BTN_SIZE * 0.25)
        bottom_y = BTN_Y + int(BTN_SIZE * 0.75)
        mid_y    = BTN_Y + BTN_SIZE // 2
        pygame.draw.polygon(screen, tri_color, [(left_x, top_y), (left_x, bottom_y), (right_x, mid_y)])

        # Aviones
        for idx, pdata in planes.items():
            speed_knots = pdata.get("speed", 0.0)
            dir_now     = pdata.get("dir", -1)
            is_repo     = pdata.get("is_repo", False)

            # Sprite (rojo si reposiciona)
            if dir_now == -1:
                surf = SURF_RIGHT_RED if is_repo else SURF_RIGHT
                tip_x = pdata["rect"].left
            else:
                surf = SURF_LEFT_RED if is_repo else SURF_LEFT
                tip_x = pdata["rect"].right

            screen.blit(surf, pdata["rect"])

            # Buffer 4' hacia adelante (rojo si repo, negro si no)
            buf_len_px = (speed_knots * (4.0 / 60.0)) * NM_TO_PX
            if buf_len_px > 1:
                rect_h = BUFFER_RECT_H
                rect_y = int(pdata["rect"].centery - rect_h / 2)
                if dir_now == -1:  # hacia AEP → derecha
                    rect_x = int(tip_x)
                    rect_w = int(buf_len_px)
                else:              # opuesto → izquierda
                    rect_x = int(tip_x - buf_len_px)
                    rect_w = int(buf_len_px)

                # dibujar borde punteado
                color = RED if is_repo else BLACK
                # clamp ancho a pantalla para evitar trabajo extra fuera de bounds
                if rect_x < -WIDTH: 
                    pass
                else:
                    draw_dashed_rect(screen, (rect_x, rect_y, rect_w, rect_h), color, dash_len=10, gap_len=6, thickness=1)

            # Velocidad + ID sobre el avión
            txt = f"{int(round(speed_knots))} kt ({idx})"
            text_surface  = FONT.render(txt, True, WHITE)
            shadow_surface= FONT.render(txt, True, BLACK)
            txtr = text_surface.get_rect()
            txtr.centerx = pdata["rect"].centerx
            txtr.bottom  = pdata["rect"].top - 18
            sh = txtr.copy(); sh.x += 1; sh.y += 1
            screen.blit(shadow_surface, sh)
            screen.blit(text_surface, txtr)

        # ---- Reloj arriba a la derecha (HH:MM) ----
        # Arranca 06:00 y avanza minuto a minuto; a las 00:00 termina la simulación
        total_min = int(sim_minutes)
        base_min = 6 * 60
        hh = ((base_min + total_min) // 60) % 24
        mm = (base_min + total_min) % 60
        time_txt = f"{hh:02d}:{mm:02d}"

        clock_surf = CLOCK_FONT.render(time_txt, True, WHITE)  # texto en blanco
        clock_rect = clock_surf.get_rect()
        clock_rect.top = 10
        clock_rect.right = WIDTH - 10
        screen.blit(clock_surf, clock_rect)

        pygame.display.flip()
        clock.tick(fps)
    try:
        pygame.quit()
        sys.exit()
    except:
        print("Exited visualization..")
