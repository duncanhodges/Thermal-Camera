"""
Thermal camera enclosure v3
  Front wall : MLX90640 (located by its TO-39 can, clamped by a screw-down bar)
  Inside     : 7x9 cm protoboard (ESP32 etc), LM2596 HW-411 buck, 2x18650 enclosed holder
  Back lid   : ILI9341 2.8" (ZJY280S0800TG01) in an edge-located snap cradle, KY-040 encoder, 20 mm rocker

Run `python3 thermal_cam_case.py` -> thermal_cam_body.stl, thermal_cam_lid.stl, thermal_cam_mlx_clamp.stl
Coordinates: x right / y up as seen from the BACK (screen side). z = 0 is the outer front face,
increasing toward the lid. All dimensions mm.
"""
import manifold3d as m
from manifold3d import Manifold as M, CrossSection as CS
import numpy as np, trimesh

m.set_circular_segments(64)

# =============== PARAMETERS — the ones marked (MEASURE) are estimates ===============
WALL, FRONT_T, LID_T, CORNER_R = 2.0, 2.5, 2.0, 5.0
IW, IH  = 141.5, 108.0     # internal width / height
CAVITY  = 38.0             # internal depth front wall -> lid
M_      = 3.0              # general margin between parts and walls

# Protoboard 90 x 54, held by its outline (no reliance on the corner-hole spacing)
PB      = (90.0, 54.0, 1.6)
PB_LIFT = 4.0              # gap under the board for solder joints
PB_CLR  = 0.4

# LM2596 HW-411 buck module
BUCK    = (43.0, 21.0, 1.6)  # PCB L x W x thickness (component height ~14)
BUCK_LIFT = 2.5            # gap under PCB for through-hole terminals
BUCK_CLR = 0.4

# 2 x 18650 enclosed holder (side-by-side black box)
BATT    = (41.0, 76.0, 20.0)  # W x L x H, long side vertical (seen from back)
BATT_CLR = 0.6

# MLX90640 — located by the can, so PCB size doesn't matter (keep PCB < CLAMP_SPAN - 6 wide)
MLX_CAN_D  = 10.4          # bore for the 10 mm sensor can (+0.4 print clearance)
MLX_CAP_H  = 4.5           # (MEASURE) cap height above the flange
MLX_RING_OD = 14.0         # ring the flange / PCB face seats on
CLAMP_SPAN = 34.0          # clamp screw spacing (bar runs horizontally across the PCB back)
MLX_PCB_T  = 1.6
FLANGE_T   = 0.5

# ILI9341 2.8" module
LCD_PCB    = (86.0, 50.0, 1.6)
LCD_STACK  = 4.0           # PCB front face -> glass top (this was right in v2)
LCD_AA     = (57.6, 43.2)  # active area
LCD_AA_DX  = -6.0          # (MEASURE) active-area centre offset from PCB centre, +ve toward the pin header
LCD_AA_DY  = 0.0           # (MEASURE)
LCD_PINS_RIGHT = True      # header end on the right as seen from the back (toward the encoder)
LCD_CLR    = 0.35
LCD_MARGIN = 0.8           # window margin around the active area

# KY-040
ENC_HOLE_D = 7.4

# Round snap-in rocker switch (20 mm panel hole; printed holes shrink ~0.2 so it's cut at 20.2)
SW_HOLE_D  = 20.2
SW_DEPTH   = 28.0          # body + terminals behind the panel, for clearance checks

# Screws: M3 x 8 self-tapping for the lid, M2 x 6 self-tapping for the MLX clamp
BOSS_D, PILOT_D, SCREW_D, HEAD_D, BOSS_LEN = 6.4, 2.5, 3.4, 6.0, 9.0
M2_PILOT, M2_CLEAR = 1.7, 2.3

# ESP32 USB access slot in the left wall (set USB_SLOT = None to drop it)
USB_SLOT = (12.0, 9.0)     # width x height
USB_V    = None            # slot centre height above internal floor; None = protoboard centre
USB_Z_ABOVE_PB = 11.5      # (MEASURE) USB centre height above protoboard top (ESP32 on female headers)

# =============== DERIVED LAYOUT ===============
OX, OY = IW + 2*WALL, IH + 2*WALL
BODY_H = FRONT_T + CAVITY
CX, CY = OX/2, OY/2
zf = FRONT_T
def U(u): return WALL + u

# left column: MLX at bottom, protoboard above. right column: battery bottom, buck top.
pb_w, pb_h = PB[0] + 2*PB_CLR, PB[1] + 2*PB_CLR
pb_x0, pb_y0 = U(M_), U(IH - M_ - pb_h)
bt_w, bt_h = BATT[0] + 2*BATT_CLR, BATT[1] + 2*BATT_CLR
bt_x0, bt_y0 = U(IW - M_ - bt_w), U(M_)
bk_w, bk_h = BUCK[0] + 2*BUCK_CLR, BUCK[1] + 2*BUCK_CLR
bk_x0 = min(bt_x0 + (bt_w - bk_w)/2, U(IW - M_ - bk_w))
bk_y0 = U(IH - M_ - bk_h)
mlx_x, mlx_y = pb_x0 + pb_w/2, (U(0) + pb_y0)/2      # centred in the strip under the protoboard

BOSS_IN = BOSS_D/2 + 0.3
boss_xy = [(U(BOSS_IN), U(BOSS_IN)), (U(IW-BOSS_IN), U(BOSS_IN)),
           (U(BOSS_IN), U(IH-BOSS_IN)), (U(IW-BOSS_IN), U(IH-BOSS_IN))]

def rrect(w, h, r): return CS.square((w-2*r, h-2*r), center=True).offset(r, m.JoinType.Round)
def box(x0, y0, z0, x1, y1, z1):
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1)
    return M.cube((x1-x0, y1-y0, z1-z0)).translate((x0, y0, z0))
def cyl(x, y, z0, h, d, d2=None): return M.cylinder(h, d/2, (d2 if d2 else d)/2).translate((x, y, z0))

def snap_clip(edge_xy, side, z0, z_top, width=8.0, t=1.4, lip=0.8, lip_h=1.2):
    """Free-standing cantilever clip just outside a part edge. side = which edge of the part
    ('L','R','B','T'). z_top = top face of the retained PCB/part; the lip hooks over it."""
    x, y = edge_xy
    zt = z_top + 0.15
    if side in 'LR':
        s = -1 if side == 'L' else 1           # outward
        wall = box(x, y-width/2, z0, x+s*t, y+width/2, zt+lip_h)
        hook = M.hull(box(x, y-width/2, zt, x-s*lip, y+width/2, zt+0.3) +
                      box(x, y-width/2, zt, x-s*0.01, y+width/2, zt+lip_h))
    else:
        s = -1 if side == 'B' else 1
        wall = box(x-width/2, y, z0, x+width/2, y+s*t, zt+lip_h)
        hook = M.hull(box(x-width/2, y, zt, x+width/2, y-s*lip, zt+0.3) +
                      box(x-width/2, y, zt, x+width/2, y-s*0.01, zt+lip_h))
    return wall + hook

def corner_guides(x0, y0, w, h, z0, z1, t=1.4, arm=6.0):
    """L-shaped locating guides hugging the outside of each corner of a rectangle."""
    g = M()
    for cx, sx in ((x0, -1), (x0+w, 1)):
        for cy, sy in ((y0, -1), (y0+h, 1)):
            g += box(cx+sx*t, cy+sy*t, z0, cx-sx*arm, cy, z1)     # arm along x
            g += box(cx+sx*t, cy+sy*t, z0, cx, cy-sy*arm, z1)     # arm along y
    return g

def pads(x0, y0, w, h, z0, z1, s=5.0, mids=False):
    p = M()
    for px in (x0, x0+w-s):
        for py in (y0, y0+h-s):
            p += box(px, py, z0, px+s, py+s, z1)
    if mids:
        for py in (y0, y0+h-s):
            p += box(x0+w/2-s/2, py, z0, x0+w/2+s/2, py+s, z1)
    return p

# =============== BODY ===============
outer = rrect(OX, OY, CORNER_R).extrude(BODY_H).translate((CX, CY, 0))
inner = rrect(IW, IH, max(CORNER_R-WALL, 0.5)).extrude(CAVITY+1).translate((CX, CY, FRONT_T))
body = outer - inner

# hanging corner bosses: only the top BOSS_LEN mm, on a 45-deg gusset (support-free),
# leaving the lower corners free for parts
for x, y in boss_xy:
    sx = -1 if x < CX else 1; sy = -1 if y < CY else 1
    cxw, cyw = (U(0) if sx < 0 else U(IW)), (U(0) if sy < 0 else U(IH))   # inner corner
    top = BODY_H
    b = cyl(x, y, top-BOSS_LEN, BOSS_LEN, BOSS_D) + box(x, y, top-BOSS_LEN, cxw, cyw, top)
    reach = abs(x-cxw) + BOSS_D/2 + 0.5
    base = b.trim_by_plane((0, 0, -1), -(top-BOSS_LEN+0.01))
    tip = box(cxw, cyw, top-BOSS_LEN-reach, cxw-sx*0.01, cyw-sy*0.01, top-BOSS_LEN-reach+0.01)
    body += b + M.hull(base + tip)
    body -= cyl(x, y, top-BOSS_LEN+0.5, BOSS_LEN, PILOT_D)
BOSS_BOTTOM = BODY_H - BOSS_LEN - (BOSS_IN + BOSS_D/2 + 0.5)

# --- protoboard: corner + mid pads, corner guides, 4 snap clips ---
pb_top = zf + PB_LIFT + PB[2]
body += pads(pb_x0, pb_y0, pb_w, pb_h, zf-0.1, zf+PB_LIFT, mids=True)
body += corner_guides(pb_x0, pb_y0, pb_w, pb_h, zf-0.1, zf+PB_LIFT+PB[2]*0.7)
for (ex, ey, sd) in ((pb_x0, pb_y0+pb_h/2, 'L'), (pb_x0+pb_w, pb_y0+pb_h/2, 'R'),
                     (pb_x0+pb_w*0.3, pb_y0, 'B'), (pb_x0+pb_w*0.7, pb_y0+pb_h, 'T')):
    body += snap_clip((ex, ey), sd, zf-0.1, pb_top)

# --- LM2596 ---
bk_top = zf + BUCK_LIFT + BUCK[2]
body += pads(bk_x0, bk_y0, bk_w, bk_h, zf-0.1, zf+BUCK_LIFT, s=4.0)
body += corner_guides(bk_x0, bk_y0, bk_w, bk_h, zf-0.1, zf+BUCK_LIFT+1.0, arm=4.0)
for (ex, ey, sd) in ((bk_x0+bk_w/2, bk_y0, 'B'), (bk_x0+bk_w/2, bk_y0+bk_h, 'T')):
    body += snap_clip((ex, ey), sd, zf-0.1, bk_top, width=7.0)

# --- 2 x 18650 holder: corner guides + 4 tall flexible clips over the top rim ---
bt_top = zf + BATT[2]
body += corner_guides(bt_x0, bt_y0, bt_w, bt_h, zf-0.1, zf+8.0, arm=8.0)
for fy in (0.28, 0.72):
    body += snap_clip((bt_x0, bt_y0+bt_h*fy), 'L', zf-0.1, bt_top, width=10.0, t=1.6, lip=1.0)
    body += snap_clip((bt_x0+bt_w, bt_y0+bt_h*fy), 'R', zf-0.1, bt_top, width=10.0, t=1.6, lip=1.0)

# --- MLX90640: bore + seat ring + 2 clamp bosses ---
ring_h = max(MLX_CAP_H - FRONT_T - 0.3, 0.0)          # cap top ends 0.3 mm below the outer face
seat_z = zf + ring_h                                   # flange rests here
pcb_back_z = seat_z + FLANGE_T + MLX_PCB_T
if ring_h > 0:
    body += cyl(mlx_x, mlx_y, zf-0.1, ring_h+0.1, MLX_RING_OD)
for sx in (-1, 1):
    bx = mlx_x + sx*CLAMP_SPAN/2
    body += cyl(bx, mlx_y, zf-0.1, pcb_back_z-zf+0.1, 5.5)
    body -= cyl(bx, mlx_y, zf+0.8, pcb_back_z, M2_PILOT)
body -= cyl(mlx_x, mlx_y, -1, seat_z+1.01, MLX_CAN_D)
body -= cyl(mlx_x, mlx_y, -0.01, 1.0, MLX_CAN_D+2.0, MLX_CAN_D)  # cosmetic chamfer

# --- ESP32 USB slot in the left wall ---
if USB_SLOT:
    uv = USB_V if USB_V is not None else (pb_y0 + pb_h/2 - WALL)
    uz = pb_top + USB_Z_ABOVE_PB
    body -= rrect(USB_SLOT[0], USB_SLOT[1], 1.5).extrude(WALL+2).rotate((0, 90, 0)).translate((-1, U(uv), uz))

# =============== MLX CLAMP BAR (separate print) ===============
bar_w, bar_t = 6.0, 2.5
clamp = rrect(CLAMP_SPAN + 7, bar_w, bar_w/2 - 0.01).extrude(bar_t)
for sx in (-1, 1):
    clamp -= cyl(sx*CLAMP_SPAN/2, 0, -1, bar_t+2, M2_CLEAR)

# =============== LID (modelled in place at z = BODY_H) ===============
lz = BODY_H
lid = rrect(OX, OY, CORNER_R).extrude(LID_T).translate((CX, CY, lz))

lcd_w, lcd_h = LCD_PCB[0] + 2*LCD_CLR, LCD_PCB[1] + 2*LCD_CLR
lcd_cx = CX                                   # screen centred on the back
lcd_cy = CY + 10.0
lcd_x0, lcd_y0 = lcd_cx - lcd_w/2, lcd_cy - lcd_h/2
dx = LCD_AA_DX if LCD_PINS_RIGHT else -LCD_AA_DX
aa_cx, aa_cy = lcd_cx + dx, lcd_cy + LCD_AA_DY
vw, vh = LCD_AA[0] + 2*LCD_MARGIN, LCD_AA[1] + 2*LCD_MARGIN
lid -= box(aa_cx-vw/2, aa_cy-vh/2, lz-1, aa_cx+vw/2, aa_cy+vh/2, lz+LID_T+1)
ch = 1.2
lid -= M.hull(box(aa_cx-vw/2, aa_cy-vh/2, lz+LID_T-ch, aa_cx+vw/2, aa_cy+vh/2, lz+LID_T-ch+0.01) +
              box(aa_cx-vw/2-ch, aa_cy-vh/2-ch, lz+LID_T+0.005, aa_cx+vw/2+ch, aa_cy+vh/2+ch, lz+LID_T+0.02))
# cradle hanging from the lid: built "upright" (z from lid inner face) then mirrored into place
lcd_back = LCD_STACK + LCD_PCB[2]            # PCB back face, measured from lid inner face
cradle = corner_guides(lcd_x0, lcd_y0, lcd_w, lcd_h, -0.1, lcd_back - 0.4, arm=7.0)
for fx in (0.3, 0.7):
    cradle += snap_clip((lcd_x0 + lcd_w*fx, lcd_y0), 'B', -0.1, lcd_back, width=8.0)
    cradle += snap_clip((lcd_x0 + lcd_w*fx, lcd_y0+lcd_h), 'T', -0.1, lcd_back, width=8.0)
lid += cradle.mirror((0, 0, 1)).translate((0, 0, lz))

# encoder, lower right over the battery (thumb position)
ENC_X = U(IW) - M_ - 11.5
ENC_Y = U(0) + 22.0
lid -= cyl(ENC_X, ENC_Y, lz-1, LID_T+2, ENC_HOLE_D)

# rocker switch, left of the encoder, over the free floor area (full depth available)
SW_X = ENC_X - 44.0
SW_Y = ENC_Y
lid -= cyl(SW_X, SW_Y, lz-1, LID_T+2, SW_HOLE_D)

# locating lip
lip_t, lip_h, clr = 1.2, 2.5, 0.25
lip = rrect(IW-2*clr, IH-2*clr, 2.5).extrude(lip_h) - \
      rrect(IW-2*clr-2*lip_t, IH-2*clr-2*lip_t, 1.5).extrude(lip_h+1).translate((0, 0, -0.5))
lip = lip.translate((CX, CY, lz-lip_h))
for x, y in boss_xy:
    lip -= cyl(x, y, lz-lip_h-1, lip_h+2, BOSS_D+8)
lid += lip
for x, y in boss_xy:
    lid -= cyl(x, y, lz-1, LID_T+2, SCREW_D)
    lid -= cyl(x, y, lz+LID_T-(HEAD_D-SCREW_D)/2, (HEAD_D-SCREW_D)/2+0.01, SCREW_D, HEAD_D)

# =============== COMPONENT ENVELOPES (fit checks / renders only) ===============
def envelopes():
    e = {}
    e['protoboard'] = box(pb_x0+PB_CLR, pb_y0+PB_CLR, zf+PB_LIFT, pb_x0+PB_CLR+PB[0], pb_y0+PB_CLR+PB[1], pb_top)
    e['pb_parts_16mm'] = box(pb_x0+8, pb_y0+8, pb_top+0.01, pb_x0+pb_w-8, pb_y0+pb_h-8, pb_top+16.0)
    e['buck'] = box(bk_x0+BUCK_CLR, bk_y0+BUCK_CLR, zf+BUCK_LIFT, bk_x0+BUCK_CLR+BUCK[0], bk_y0+BUCK_CLR+BUCK[1], zf+BUCK_LIFT+14)
    e['battery'] = box(bt_x0+BATT_CLR, bt_y0+BATT_CLR, zf, bt_x0+BATT_CLR+BATT[0], bt_y0+BATT_CLR+BATT[1], bt_top)
    e['mlx_can'] = cyl(mlx_x, mlx_y, seat_z-MLX_CAP_H+0.01, MLX_CAP_H-0.02, 10.0)
    e['mlx_pcb_26x22'] = box(mlx_x-13, mlx_y-11, seat_z+FLANGE_T, mlx_x+13, mlx_y+11, pcb_back_z)
    e['mlx_clamp'] = clamp.translate((mlx_x, mlx_y, pcb_back_z))
    e['lcd'] = box(lcd_x0+LCD_CLR, lcd_y0+LCD_CLR, lz-lcd_back, lcd_x0+LCD_CLR+LCD_PCB[0], lcd_y0+LCD_CLR+LCD_PCB[1], lz)
    e['lcd_rear_8mm'] = box(lcd_x0+4, lcd_y0+4, lz-lcd_back-8, lcd_x0+lcd_w-4, lcd_y0+lcd_h-4, lz-lcd_back-0.01)
    e['rocker'] = cyl(SW_X, SW_Y, lz-SW_DEPTH, SW_DEPTH-0.01, 23.0)
    e['encoder'] = box(ENC_X-9.5, ENC_Y-12.5, lz-11, ENC_X+9.5, ENC_Y+19.5, lz-0.01)
    return e

def to_tm(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)

if __name__ == "__main__":
    b = to_tm(body)
    l = to_tm(lid.mirror((0, 0, 1))); l.apply_translation((0, 0, -l.bounds[0][2]))
    c = to_tm(clamp)
    b.export("thermal_cam_body.stl"); l.export("thermal_cam_lid.stl"); c.export("thermal_cam_mlx_clamp.stl")
    for n, t in (("body", b), ("lid", l), ("clamp", c)):
        print(f"{n:6s} watertight={t.is_watertight} extents={np.round(t.extents,1)}")
    print(f"outer {OX:.1f} x {OY:.1f} x {BODY_H+LID_T:.1f} mm; boss gusset bottom z={BOSS_BOTTOM:.1f}, battery top z={bt_top:.1f}")
