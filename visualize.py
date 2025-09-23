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
    AIRCRAFT_SVG_PATH="aircraft.svg",   # sprite base (ideal mirando a la DERECHA)
    AIRCRAFT_PNG_PATH="aircraft.png",   # alternativa PNG
    SPRITE_PX_WIDTH=36,                 # ancho base (se reescala según ventana)
):
    """
    history: list[list[tuple(min_from_start, pos_nm, speed_knots, dir, status)]]
             NOTA: si tu historial incluye más campos en cada tupla, los extras se ignoran.
    sim_minutes_per_second: minutos simulados por segundo real
    sim_time_minutes: duración total de la simulación (p. ej. 18*60)
    fps: frames por segundo
    use_svg: True->SVG con cairosvg, False->PNG
    """

    # ==================== PYGAME INIT ====================
    pygame.init()

    # Tamaño inicial amigable a notebooks, pero adaptable a pantallas chicas
    di = pygame.display.Info()
    init_w = min(1200, max(800, di.current_w - 120))
    init_h = min(720,  max(520, di.current_h - 160))

    screen = pygame.display.set_mode((init_w, init_h), pygame.RESIZABLE)
    pygame.display.set_caption("AEP Traffic")
    clock = pygame.time.Clock()

    pygame.font.init()

    # ==================== ESTADO DE UI / ESCALA ====================
    WIDTH, HEIGHT = screen.get_size()

    def compute_layout(w, h):
        """Recalcula todo lo que depende del tamaño de ventana."""
        # Márgenes proporcionales
        left_margin  = max(50, int(w * 0.06))
        right_margin = max(40, int(w * 0.05))
        top_margin   = max(16, int(h * 0.03))
        bottom_margin= max(16, int(h * 0.05))

        # 100 NM ocupan el ancho útil
        nm_to_px = (w - left_margin - right_margin) / 100.0
        start_x  = left_margin

        # Trazas y carril
        lane_y = int(h * 0.55)

        # Tamaños UI
        btn_size = max(44, min(64, int(h * 0.09)))
        btn_x, btn_y = 10, h - btn_size - 10
        button_rect = pygame.Rect(btn_x, btn_y, btn_size, btn_size)

        # Tipos de letra escalados
        font_size       = max(14, int(h * 0.025))
        clock_font_size = max(20, int(h * 0.045))
        counter_font_size = max(16, int(h * 0.03))
        FONT       = pygame.font.SysFont(None, font_size)
        CLOCK_FONT = pygame.font.SysFont(None, clock_font_size)
        COUNTER_FONT = pygame.font.SysFont(None, counter_font_size)

        # Sprite escalado respecto a altura
        target_sprite_w = max(22, int(h * 0.045))

        # Buffer visual delante del avión
        buffer_rect_h = max(14, int(h * 0.03))

        # Panel contadores (arriba izquierda)
        counters_rect = pygame.Rect(
            10, 10 + clock_font_size + 8,
            int(w * 0.22),  # ancho relativo
            int(h * 0.12)
        )

        return {
            "LEFT_MARGIN": left_margin,
            "RIGHT_MARGIN": right_margin,
            "TOP_MARGIN": top_margin,
            "BOTTOM_MARGIN": bottom_margin,
            "NM_TO_PX": nm_to_px,
            "START_X": start_x,
            "LANE_Y": lane_y,
            "BTN_RECT": button_rect,
            "BTN_SIZE": btn_size,
            "FONT": FONT,
            "CLOCK_FONT": CLOCK_FONT,
            "COUNTER_FONT": COUNTER_FONT,
            "SPRITE_W": target_sprite_w,
            "BUFFER_RECT_H": buffer_rect_h,
            "COUNTERS_RECT": counters_rect,
        }

    L = compute_layout(WIDTH, HEIGHT)

    # ==================== COLORES ====================
    BACKGROUND = (26, 39, 110)
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED   = (255, 0, 0)
    PANEL_BG = (0, 0, 0, 120)  # semi-transparente

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

    def regenerate_surfaces():
        """Recrea los sprites según el tamaño actual."""
        target_w = L["SPRITE_W"]
        if use_svg:
            base = surface_from_svg(AIRCRAFT_SVG_PATH, target_w)
        else:
            base = surface_from_png(AIRCRAFT_PNG_PATH, target_w)

        # Horizontal
        surf_right = base
        surf_left  = pygame.transform.flip(base, True, False)
        # Vertical (90° arriba)
        surf_up_from_right = pygame.transform.rotate(surf_right, 90)   # nariz hacia ARRIBA
        # Versiones tintadas
        return {
            "RIGHT": surf_right,
            "LEFT":  surf_left,
            "RIGHT_RED": tint_surface(surf_right, RED),
            "LEFT_RED":  tint_surface(surf_left,  RED),
            "UP":        surf_up_from_right,
            "UP_RED":    tint_surface(surf_up_from_right, RED),
        }

    SURF = regenerate_surfaces()

    # ==================== HELPERS DE CONVERSIÓN ====================
    def x_from_nm(pos_nm: float) -> int:
        """x = START_X en 100 NM y 'desciende' hacia la derecha (menor NM -> mayor x)."""
        return int(round(L["START_X"] + (100.0 - float(pos_nm)) * L["NM_TO_PX"]))

    # Marcas (0, 5, 15, 50, 100 NM) → recalculadas on-the-fly
    MARKS = [0, 5, 15, 50, 100]

    # ==================== PREPROCESO: ARRIBOS ====================
    arrivals_by_minute = {m: [] for m in range(sim_time_minutes + 1)}
    for idx, hist in enumerate(history):
        if not hist:
            continue
        # Aceptamos tuplas más largas (solo tomamos los 5 primeros campos si hay extras)
        t0 = int(hist[0][0])
        if 0 <= t0 <= sim_time_minutes:
            arrivals_by_minute[t0].append(idx)

    # ==================== ESTRUCTURAS EN EJECUCIÓN ====================
    planes = {}       # idx -> dict estado runtime
    plane_state = {}  # idx -> {"hist_pos": int, "active": bool}

    # ---- Contadores ----
    landed_total = 0
    diverted_total = 0
    flying_current = 0
    seen_landed = set()
    seen_diverted = set()

    sim_minutes = 0.0
    last_processed_minute = -1
    paused = False
    running = True

    # ==================== DIBUJO: LÍNEAS PUNTEADAS ====================
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
        # Eventos / pausa / resize
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if L["BTN_RECT"].collidepoint(event.pos):
                    paused = not paused

            elif event.type == pygame.VIDEORESIZE:
                WIDTH, HEIGHT = max(640, event.w), max(480, event.h)
                screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
                L = compute_layout(WIDTH, HEIGHT)
                SURF = regenerate_surfaces()
                # Ajustar y de los que están en vuelo vertical (mantener x,y relativos)
                for pdata in planes.values():
                    if pdata.get("mode") == "divert":
                        # nada especial, su y/x ya están en píxeles de pantalla
                        pass

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
                plane_state[idx] = {"hist_pos": 0, "active": True}
                hist = history[idx]
                # Avanzar hist_pos hasta el último estado con t ≤ current_minute
                while (plane_state[idx]["hist_pos"] + 1 < len(hist)
                       and hist[plane_state[idx]["hist_pos"] + 1][0] <= current_minute):
                    plane_state[idx]["hist_pos"] += 1

                # Estado actual (ignoramos campos extra si los hay)
                rec = hist[plane_state[idx]["hist_pos"]]
                t0, pos0, speed0, dir0 = rec[0], rec[1], rec[2], int(rec[3])
                # status puede venir en 5to o 7mo campo según tu historial; tomamos defensivo
                status0 = ""
                if len(rec) >= 5:
                    status0 = rec[4]
                if len(rec) >= 7:
                    status0 = rec[6]
                dt_min = max(0.0, sim_minutes - float(t0))
                pos_now = float(pos0) + (float(speed0) * (dt_min / 60.0)) * (1 if dir0 == 1 else -1)
                x_tip = x_from_nm(pos_now)

                # Crear rect con nariz anclada a x_tip
                if dir0 == -1:
                    rect = SURF["RIGHT"].get_rect()
                    rect.midleft = (int(round(x_tip)), L["LANE_Y"])
                else:
                    rect = SURF["LEFT"].get_rect()
                    rect.midright = (int(round(x_tip)), L["LANE_Y"])

                planes[idx] = {
                    "rect": rect,
                    "tip_x": float(x_tip),
                    "speed": float(speed0),
                    "dir": dir0,
                    "mode": "normal",   # "normal" | "divert"
                    "is_repo": (dir0 == 1) or (str(status0).lower() == "bounced"),
                    "x": rect.centerx,  # para modo "divert"
                    "y": rect.centery,  # para modo "divert"
                }

        # Actualización por frame de todos los activos
        to_remove = []
        if delta_minutes > 0.0:
            for idx, pdata in list(planes.items()):
                st = plane_state.get(idx)
                if not st or not st["active"]:
                    continue

                # Modo desvío vertical: sube recta hacia arriba
                if pdata.get("mode") == "divert":
                    speed_knots = pdata.get("speed", 0.0)
                    # píxeles por minuto = NM/min * px/NM
                    dy = (speed_knots / 60.0) * L["NM_TO_PX"] * delta_minutes
                    pdata["y"] -= dy
                    # Actualizar rect centrado en (x,y)
                    rect = SURF["UP_RED"].get_rect()
                    rect.center = (int(round(pdata["x"])), int(round(pdata["y"])))
                    pdata["rect"] = rect
                    # Fuera de pantalla por arriba → eliminar
                    if rect.bottom < 0:
                        to_remove.append(idx)
                    continue

                # Si sigue en modo normal, usamos historial
                hist = history[idx]

                # Avanzar hist_pos si hay nuevos estados ≤ current_minute
                while (st["hist_pos"] + 1 < len(hist)
                       and hist[st["hist_pos"] + 1][0] <= current_minute):
                    st["hist_pos"] += 1

                rec = hist[st["hist_pos"]]
                t_now, pos_nm, speed_knots, dir_now = rec[0], rec[1], rec[2], int(rec[3])

                status_now = ""
                if len(rec) >= 5:
                    status_now = rec[4]
                if len(rec) >= 7:
                    status_now = rec[6]
                status_now = (status_now or "").lower()
                dir_now = int(dir_now)

                # Aterrizado → contar una vez y eliminar
                if status_now == "landed":
                    if idx not in seen_landed:
                        landed_total += 1
                        seen_landed.add(idx)
                    to_remove.append(idx)
                    continue

                # Desviado → cambia a modo vertical y contar una vez
                if status_now == "diverted":
                    if idx not in seen_diverted:
                        diverted_total += 1
                        seen_diverted.add(idx)
                    # Posición actual como punto de partida vertical
                    dt_min = max(0.0, sim_minutes - float(t_now))
                    pos_now = float(pos_nm) + (float(speed_knots) * (dt_min / 60.0)) * (1 if dir_now == 1 else -1)
                    x_tip = x_from_nm(pos_now)
                    # Fijamos (x,y) actuales según rect existente (si lo hubiera)
                    current_y = pdata["rect"].centery if "rect" in pdata else L["LANE_Y"]
                    pdata.update({
                        "mode": "divert",
                        "speed": float(speed_knots),
                        "x": int(round(x_tip)),
                        "y": float(current_y),
                    })
                    # Ajustamos rect al instante (ya rojo vertical)
                    rect = SURF["UP_RED"].get_rect()
                    rect.center = (pdata["x"], int(round(pdata["y"])))
                    pdata["rect"] = rect
                    continue

                # Extrapolar posición horizontal (modo normal)
                dt_min = max(0.0, sim_minutes - float(t_now))
                pos_now = float(pos_nm) + (float(speed_knots) * (dt_min / 60.0)) * (1 if dir_now == 1 else -1)
                x_tip = x_from_nm(pos_now)
                pdata["tip_x"] = x_tip

                # Reposicionamiento: recuadro rojo si dir==1 o status == "bounced"
                is_repositioning = (dir_now == 1) or (status_now == "bounced")

                # Colocar rect según dir manteniendo nariz en x_tip
                if dir_now == -1:
                    rect = SURF["RIGHT"].get_rect()
                    rect.midleft = (int(round(x_tip)), L["LANE_Y"])
                else:
                    rect = SURF["LEFT"].get_rect()
                    rect.midright = (int(round(x_tip)), L["LANE_Y"])

                pdata["rect"] = rect
                pdata["is_repo"] = is_repositioning
                pdata["speed"] = float(speed_knots)
                pdata["dir"] = dir_now

        for idx in to_remove:
            planes.pop(idx, None)
            plane_state.pop(idx, None)

        # ---- Actualizar "En vuelo" (todos los activos no en modo desvío) ----
        flying_current = sum(1 for p in planes.values() if p.get("mode") != "divert")

        # ==================== DIBUJO ====================
        WIDTH, HEIGHT = screen.get_size()
        screen.fill(BACKGROUND)

        # Recalcular marcas cada frame por si se redimensionó
        marks_px = {f"{nm}NM": x_from_nm(nm) for nm in MARKS}

        # Líneas de tramo: sólida en 0; punteadas en 5, 15, 50, 100
        pygame.draw.line(screen, WHITE, (marks_px["0NM"], 0), (marks_px["0NM"], HEIGHT), 2)
        for key in ["5NM", "15NM", "50NM", "100NM"]:
            draw_dashed_vline(screen, marks_px[f"{key}"], 0, HEIGHT, WHITE)

        # Botón pausa (triángulo tipo "play")
        rect_color = WHITE if paused else BLACK
        tri_color  = BLACK if paused else WHITE
        pygame.draw.rect(screen, rect_color, L["BTN_RECT"], border_radius=8)
        left_x   = L["BTN_RECT"].x + int(L["BTN_SIZE"] * 0.30)
        right_x  = L["BTN_RECT"].x + int(L["BTN_SIZE"] * 0.70)
        top_y    = L["BTN_RECT"].y + int(L["BTN_SIZE"] * 0.25)
        bottom_y = L["BTN_RECT"].y + int(L["BTN_SIZE"] * 0.75)
        mid_y    = L["BTN_RECT"].y + L["BTN_SIZE"] // 2
        pygame.draw.polygon(screen, tri_color, [(left_x, top_y), (left_x, bottom_y), (right_x, mid_y)])

        # Aviones
        FONT = L["FONT"]
        for idx, pdata in planes.items():
            mode = pdata.get("mode", "normal")
            speed_knots = pdata.get("speed", 0.0)
            dir_now     = pdata.get("dir", -1)
            is_repo     = pdata.get("is_repo", False)

            if mode == "divert":
                surf = SURF["UP_RED"]
                screen.blit(surf, pdata["rect"])
                # Texto encima
                txt = f"{int(round(speed_knots))} kt ({idx})"
                text_surface  = FONT.render(txt, True, WHITE)
                shadow_surface= FONT.render(txt, True, BLACK)
                txtr = text_surface.get_rect()
                txtr.centerx = pdata["rect"].centerx
                txtr.bottom  = pdata["rect"].top - 8
                sh = txtr.copy(); sh.x += 1; sh.y += 1
                screen.blit(shadow_surface, sh)
                screen.blit(text_surface, txtr)
                # No dibujamos buffer en modo vertical
            else:
                if dir_now == -1:
                    surf = SURF["RIGHT_RED"] if is_repo else SURF["RIGHT"]
                    tip_x = pdata["rect"].left
                else:
                    surf = SURF["LEFT_RED"] if is_repo else SURF["LEFT"]
                    tip_x = pdata["rect"].right

                screen.blit(surf, pdata["rect"])

                # Buffer 4' hacia adelante (rojo si repo, negro si no)
                buf_len_px = (speed_knots * (4.0 / 60.0)) * L["NM_TO_PX"]
                if buf_len_px > 1:
                    rect_h = L["BUFFER_RECT_H"]
                    rect_y = int(pdata["rect"].centery - rect_h / 2)
                    if dir_now == -1:  # hacia AEP → derecha
                        rect_x = int(tip_x)
                        rect_w = int(buf_len_px)
                    else:              # opuesto → izquierda
                        rect_x = int(tip_x - buf_len_px)
                        rect_w = int(buf_len_px)

                    color = RED if is_repo else BLACK
                    draw_dashed_rect(screen, (rect_x, rect_y, rect_w, rect_h), color,
                                     dash_len=10, gap_len=6, thickness=1)

                # Velocidad + ID sobre el avión
                txt = f"{int(round(speed_knots))} kt ({idx})"
                text_surface  = FONT.render(txt, True, WHITE)
                shadow_surface= FONT.render(txt, True, BLACK)
                txtr = text_surface.get_rect()
                txtr.centerx = pdata["rect"].centerx
                txtr.bottom  = pdata["rect"].top - 12
                sh = txtr.copy(); sh.x += 1; sh.y += 1
                screen.blit(shadow_surface, sh)
                screen.blit(text_surface, txtr)

        # ---- Reloj arriba a la derecha (HH:MM) ----
        CLOCK_FONT = L["CLOCK_FONT"]
        total_min = int(sim_minutes)
        base_min = 6 * 60
        hh = ((base_min + total_min) // 60) % 24
        mm = (base_min + total_min) % 60
        time_txt = f"{hh:02d}:{mm:02d}"

        clock_surf = CLOCK_FONT.render(time_txt, True, WHITE)
        clock_rect = clock_surf.get_rect()
        clock_rect.top = 10
        clock_rect.right = WIDTH - 10
        screen.blit(clock_surf, clock_rect)

        # ---- Panel contadores (arriba izquierda) ----
        COUNTER_FONT = L["COUNTER_FONT"]
        panel = pygame.Surface((L["COUNTERS_RECT"].w, L["COUNTERS_RECT"].h), pygame.SRCALPHA)
        panel.fill(PANEL_BG)
        line1 = COUNTER_FONT.render(f"En vuelo: {flying_current}", True, WHITE)
        line2 = COUNTER_FONT.render(f"Arribados: {landed_total}", True, WHITE)
        line3 = COUNTER_FONT.render(f"Desviados: {diverted_total}", True, WHITE)
        # margen interno
        pad = 10
        panel.blit(line1, (pad, pad))
        panel.blit(line2, (pad, pad + line1.get_height() + 4))
        panel.blit(line3, (pad, pad + line1.get_height() + line2.get_height() + 8))
        screen.blit(panel, (L["COUNTERS_RECT"].x, L["COUNTERS_RECT"].y))

        pygame.display.flip()
        clock.tick(fps)

    try:
        pygame.quit()
        sys.exit()
    except:
        print("Exited visualization..")
