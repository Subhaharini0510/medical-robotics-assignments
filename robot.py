import time, math
import browserbotics as bb

# ════════════════════════════════════════════════════════════
#  HOSPITAL SURGICAL ROBOTICS — v8.0
#
#  Floor plan (shared walls, no gaps, no roof, no floating items):
#
#   Y=8 ┌──────────┬──────────────────────┬──────────┐
#       │  SCRUB / │   OPERATING ROOM     │ STERILE  │
#       │   PREP   │   (Robot + Table)    │  STORE   │
#   Y=2 ├──────────┴──────────┬───────────┴──────────┤
#       │        CENTRAL CORRIDOR                     │
#   Y=0 ├────────┬────────────┬──────────────┬────────┤
#       │ STAFF  │  NURSES'   │  RECOVERY    │ EQUIP. │
#       │  ROOM  │  STATION   │  ROOM        │ STORE  │
#   Y=-6└────────┴────────────┴──────────────┴────────┘
#     X=-9      X=-4          X=2           X=10    X=14
# ════════════════════════════════════════════════════════════

TEAL_DRK  = '#1A5C6A'
TEAL_MID  = '#2A8898'
STEEL     = '#A8B4BC'
STEEL_DK  = '#788890'
SCRN_BLK  = '#0A1020'
SCRN_GRN  = '#00FF88'
LAMP_WHT  = '#FFFFF0'
SKIN_TON  = '#E8C4A0'
LIGHT_BLUE= '#A8C8DC'
DADO      = '#2A7888'

W_OR    = '#C8D4D8'
W_SCRUB = '#C4D4CC'
W_REC   = '#C4CCD8'
W_STAFF = '#D4CCBC'
W_STORE = '#D0D0CC'
W_NURS  = '#C8D0D8'
W_EXT   = '#8A9AA4'

FLR_OR    = '#D0DCE0'
FLR_CORR  = '#E0E8EA'
FLR_SCRUB = '#C8DDD4'
FLR_REC   = '#C8D4E0'
FLR_STAFF = '#DDD8CC'
FLR_STORE = '#D4D4D0'
FLR_NURS  = '#C8D4D8'

WH = 2.60   # wall height — walls exist but NO ceiling added
WT = 0.12   # wall half-thickness
DW = 0.55   # door half-width

PEDESTAL_TOP_Z = 0.442
ZERO_QUAT      = [0.0, 0.0, 0.0, 1.0]
home_arm_jpos  = [0.0, -0.3, 0.0, -1.8, 0.0, 1.6, 0.8]
JOINT_NAMES = ['J1 Base','J2 Shoulder','J3 Elbow','J4 Elbow2','J5 Wrist1','J6 Wrist2','J7 Wrist3']
JOINT_LO    = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
JOINT_HI    = [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]

robot=None; ee_link=10; DOWN_QUAT=None
OBJ_IDS={}; OBJ_PICK={}; ORIGIN={}; BED_DROPS=[]
cur_j=list(home_arm_jpos); held_obj=''; drop_idx=0

# Room boundary coordinates
BLDG_X1,BLDG_X2 = -9.0, 14.0
BLDG_Y1,BLDG_Y2 = -6.0,  8.0
CORR_Y1,CORR_Y2 =  0.0,  2.0
OR_X1,OR_X2       = -6.0,  0.0
OR_Y1,OR_Y2       =  2.0,  8.0
SCRUB_X1,SCRUB_X2 = -9.0, -6.0
SCRUB_Y1,SCRUB_Y2 =  2.0,  8.0
STER_X1,STER_X2   =  0.0,  4.0
STER_Y1,STER_Y2   =  2.0,  8.0
STAFF_X1,STAFF_X2 = -9.0, -4.0
STAFF_Y1,STAFF_Y2 = -6.0,  0.0
NURS_X1,NURS_X2   = -4.0,  2.0
NURS_Y1,NURS_Y2   = -6.0,  0.0
REC_X1,REC_X2     =  2.0, 10.0
REC_Y1,REC_Y2     = -6.0,  0.0
EQUIP_X1,EQUIP_X2 = 10.0, 14.0
EQUIP_Y1,EQUIP_Y2 = -6.0,  0.0

def rcx(x1,x2): return (x1+x2)/2
def rcy(y1,y2): return (y1+y2)/2
def rhw(x1,x2): return (x2-x1)/2
def rhh(y1,y2): return (y2-y1)/2

# ── core primitive ───────────────────────────────────────────
def B(hx,hy,hz,px,py,pz,col):
    bb.createBody('box',halfExtent=[hx,hy,hz],position=[px,py,pz],color=col,mass=0)

def wall(px,py,hx,hy,col=W_EXT):
    """Full-height solid wall segment."""
    B(hx,hy,WH/2, px,py,WH/2, col)

def floor_slab(px,py,hx,hy,col,gc=None):
    B(hx,hy,0.012, px,py,0.006, col)
    if gc:
        nx=int(hx/0.40)+2
        for i in range(-nx,nx+1): B(0.004,hy,0.002, px+i*0.80,py,0.014,gc)
        ny=int(hy/0.40)+2
        for j in range(-ny,ny+1): B(hx,0.004,0.002, px,py+j*0.80,0.014,gc)

def wall_door_x(py,x1,x2,dcx,col):
    """Horizontal wall at Y=py with door gap at dcx.
       NO transom — open top, nothing floating above door."""
    gl=dcx-DW; gr=dcx+DW
    if gl>x1: wall(rcx(x1,gl),  py, rhw(x1,gl),  WT, col)
    if gr<x2: wall(rcx(gr,x2),  py, rhw(gr,x2),  WT, col)
    # Door frame posts only (ground to ~2.1 m, no top bar)
    for s in [-1,1]:
        B(0.04,WT+0.02,1.05, dcx+s*DW,py,1.05,'#B0BCC4')

def wall_door_y(px,y1,y2,dcy,col):
    """Vertical wall at X=px with door gap at dcy.
       NO transom."""
    gb=dcy-DW; gt=dcy+DW
    if gb>y1: wall(px, rcy(y1,gb), WT, rhh(y1,gb), col)
    if gt<y2: wall(px, rcy(gt,y2), WT, rhh(gt,y2), col)
    for s in [-1,1]:
        B(WT+0.02,0.04,1.05, px,dcy+s*DW,1.05,'#B0BCC4')

def dado_strip(px,py,hx,hy):
    """Wall-mounted dado rail — sits on the wall face, not floating."""
    B(hx+0.004,hy+0.004,0.022, px,py,1.220,DADO)
    B(hx+0.006,hy+0.006,0.040, px,py,0.040,TEAL_MID)

def room_name_plaque(px,py,wall_face,name_col='#1A3A5A'):
    """
    Wall-mounted room name plaque at eye level (~1.55 m).
    wall_face: 'S','N','E','W' — which wall face to mount on.
    The plaque is a flat coloured board flush against the wall.
    """
    OFF = WT+0.015   # small offset so it sits proud of wall face
    if wall_face=='S':   # mounted on south face (py is the wall Y, plaque on outside going south)
        B(0.55,0.008,0.14, px, py-OFF, 1.55, name_col)
        B(0.48,0.006,0.06, px, py-OFF-0.001, 1.55, '#FFFFFF')
    elif wall_face=='N':
        B(0.55,0.008,0.14, px, py+OFF, 1.55, name_col)
        B(0.48,0.006,0.06, px, py+OFF+0.001, 1.55, '#FFFFFF')
    elif wall_face=='W':
        B(0.008,0.55,0.14, px-OFF, py, 1.55, name_col)
        B(0.006,0.48,0.06, px-OFF-0.001, py, 1.55, '#FFFFFF')
    elif wall_face=='E':
        B(0.008,0.55,0.14, px+OFF, py, 1.55, name_col)
        B(0.006,0.48,0.06, px+OFF+0.001, py, 1.55, '#FFFFFF')

def door_side_plaque(px,py,axis,col='#1A3A5A'):
    """Small room-number plaque beside a door, mounted on the wall at 1.40 m."""
    if axis=='x':   # door in horizontal wall → plaque on wall face
        B(0.20,0.008,0.10, px+DW+0.30,py-WT-0.012,1.40,col)
        B(0.17,0.006,0.04, px+DW+0.30,py-WT-0.013,1.40,'#FFFFFF')
    else:
        B(0.008,0.20,0.10, px-WT-0.012,py+DW+0.30,1.40,col)
        B(0.006,0.17,0.04, px-WT-0.013,py+DW+0.30,1.40,'#FFFFFF')

# ════════════════════════════════════════════════════════════
# PERIMETER
# ════════════════════════════════════════════════════════════
def build_perimeter():
    BCX=rcx(BLDG_X1,BLDG_X2); BCY=rcy(BLDG_Y1,BLDG_Y2)
    BHW=rhw(BLDG_X1,BLDG_X2); BHH=rhh(BLDG_Y1,BLDG_Y2)
    wall(BCX,BLDG_Y1,BHW,WT); dado_strip(BCX,BLDG_Y1,BHW,WT)
    wall(BCX,BLDG_Y2,BHW,WT); dado_strip(BCX,BLDG_Y2,BHW,WT)
    wall(BLDG_X1,BCY,WT,BHH); dado_strip(BLDG_X1,BCY,WT,BHH)
    wall_door_y(BLDG_X2,BLDG_Y1,BLDG_Y2,rcy(CORR_Y1,CORR_Y2),W_EXT)
    dado_strip(BLDG_X2,BCY,WT,BHH)

# ════════════════════════════════════════════════════════════
# PARTITIONS
# ════════════════════════════════════════════════════════════
def build_partitions():
    # North corridor wall (Y=2)
    wall_door_x(CORR_Y2,BLDG_X1,OR_X1,  rcx(SCRUB_X1,SCRUB_X2),W_SCRUB)
    wall_door_x(CORR_Y2,OR_X1,OR_X2,    rcx(OR_X1,OR_X2),       W_OR)
    wall_door_x(CORR_Y2,STER_X1,STER_X2,rcx(STER_X1,STER_X2),   W_STORE)
    wall(rcx(STER_X2,BLDG_X2),CORR_Y2,rhw(STER_X2,BLDG_X2),WT,W_EXT)
    # South corridor wall (Y=0)
    wall_door_x(CORR_Y1,BLDG_X1,STAFF_X2,rcx(STAFF_X1,STAFF_X2),W_STAFF)
    wall_door_x(CORR_Y1,NURS_X1,NURS_X2, rcx(NURS_X1,NURS_X2),  W_NURS)
    wall_door_x(CORR_Y1,REC_X1,REC_X2,   rcx(REC_X1,REC_X2),    W_REC)
    wall_door_x(CORR_Y1,EQUIP_X1,EQUIP_X2,rcx(EQUIP_X1,EQUIP_X2),W_STORE)
    # Vertical upper partitions
    wall_door_y(OR_X1, CORR_Y2,BLDG_Y2,rcy(CORR_Y2,BLDG_Y2),W_SCRUB)
    wall_door_y(OR_X2, CORR_Y2,BLDG_Y2,rcy(CORR_Y2,BLDG_Y2),W_STORE)
    wall(STER_X2,rcy(STER_Y1,STER_Y2),WT,rhh(STER_Y1,STER_Y2),W_EXT)
    # Vertical lower partitions
    wall_door_y(STAFF_X2,BLDG_Y1,CORR_Y1,rcy(BLDG_Y1,CORR_Y1),W_STAFF)
    wall_door_y(NURS_X2, BLDG_Y1,CORR_Y1,rcy(BLDG_Y1,CORR_Y1),W_NURS)
    wall_door_y(REC_X2,  BLDG_Y1,CORR_Y1,rcy(BLDG_Y1,CORR_Y1),W_REC)
    # Dado on all partitions
    for px2,py2,hx2,hy2 in [
        (rcx(BLDG_X1,BLDG_X2),CORR_Y2,rhw(BLDG_X1,BLDG_X2),WT),
        (rcx(BLDG_X1,BLDG_X2),CORR_Y1,rhw(BLDG_X1,BLDG_X2),WT),
        (OR_X1, rcy(CORR_Y2,BLDG_Y2),WT,rhh(CORR_Y2,BLDG_Y2)),
        (OR_X2, rcy(CORR_Y2,BLDG_Y2),WT,rhh(CORR_Y2,BLDG_Y2)),
        (STAFF_X2,rcy(BLDG_Y1,CORR_Y1),WT,rhh(BLDG_Y1,CORR_Y1)),
        (NURS_X2, rcy(BLDG_Y1,CORR_Y1),WT,rhh(BLDG_Y1,CORR_Y1)),
        (REC_X2,  rcy(BLDG_Y1,CORR_Y1),WT,rhh(BLDG_Y1,CORR_Y1)),
    ]: dado_strip(px2,py2,hx2,hy2)

# ════════════════════════════════════════════════════════════
# FLOORS
# ════════════════════════════════════════════════════════════
def build_floors():
    floor_slab(rcx(BLDG_X1,BLDG_X2),rcy(CORR_Y1,CORR_Y2),
               rhw(BLDG_X1,BLDG_X2),rhh(CORR_Y1,CORR_Y2),FLR_CORR,'#C8D4D8')
    B(rhw(BLDG_X1,BLDG_X2),0.06,0.003,
      rcx(BLDG_X1,BLDG_X2),rcy(CORR_Y1,CORR_Y2),0.016,'#5090B8')
    floor_slab(rcx(OR_X1,OR_X2),    rcy(OR_Y1,OR_Y2),
               rhw(OR_X1,OR_X2),    rhh(OR_Y1,OR_Y2),    FLR_OR,  '#B8C8CE')
    floor_slab(rcx(SCRUB_X1,SCRUB_X2),rcy(SCRUB_Y1,SCRUB_Y2),
               rhw(SCRUB_X1,SCRUB_X2),rhh(SCRUB_Y1,SCRUB_Y2),FLR_SCRUB,'#A8C4BC')
    floor_slab(rcx(STER_X1,STER_X2),rcy(STER_Y1,STER_Y2),
               rhw(STER_X1,STER_X2),rhh(STER_Y1,STER_Y2),FLR_STORE,'#C0C0BC')
    floor_slab(rcx(STAFF_X1,STAFF_X2),rcy(STAFF_Y1,STAFF_Y2),
               rhw(STAFF_X1,STAFF_X2),rhh(STAFF_Y1,STAFF_Y2),FLR_STAFF,'#C8C4B8')
    floor_slab(rcx(NURS_X1,NURS_X2),rcy(NURS_Y1,NURS_Y2),
               rhw(NURS_X1,NURS_X2),rhh(NURS_Y1,NURS_Y2),FLR_NURS,'#B8C8CC')
    floor_slab(rcx(REC_X1,REC_X2),  rcy(REC_Y1,REC_Y2),
               rhw(REC_X1,REC_X2),  rhh(REC_Y1,REC_Y2),  FLR_REC, '#B0C4D0')
    floor_slab(rcx(EQUIP_X1,EQUIP_X2),rcy(EQUIP_Y1,EQUIP_Y2),
               rhw(EQUIP_X1,EQUIP_X2),rhh(EQUIP_Y1,EQUIP_Y2),FLR_STORE,'#B8B8B4')
    # OR safety zone stripes
    OCX=rcx(OR_X1,OR_X2); OCY=rcy(OR_Y1,OR_Y2)
    for sx,sy,shx,shy in [(OCX,OCY+2.2,2.4,0.04),(OCX,OCY-2.2,2.4,0.04),
                           (OCX+2.4,OCY,0.04,2.2),(OCX-2.4,OCY,0.04,2.2)]:
        B(shx,shy,0.003,sx,sy,0.016,'#D4A800')

# ════════════════════════════════════════════════════════════
# ROOM NAME PLAQUES  — all wall-mounted, nothing floating
# ════════════════════════════════════════════════════════════
def build_room_name_plaques():
    """
    Each room gets a large name board mounted flat on its south or west
    interior wall at eye-level (Z centre = 1.55 m, height = 0.28 m).
    Letters are implied by a contrasting stripe on the plaque face.
    Colour coding matches the wall colour of each room.
    """
    PLAQUE_H = 0.28   # plaque height
    PLAQUE_D = 0.016  # plaque depth (how far it sticks off wall)

    def name_board(cx,cy,wall_y,face,room_col,text_col='#FFFFFF'):
        """A wide coloured board + text stripe on a wall face."""
        off = WT + PLAQUE_D/2
        if face == 'S':
            B(0.90,PLAQUE_D,PLAQUE_H, cx,wall_y-off,1.55,room_col)
            # 3 text lines (white stripes simulating text rows)
            for ti,tw in enumerate([0.72,0.55,0.38]):
                B(tw,PLAQUE_D+0.002,0.030, cx,wall_y-off-0.001,1.65-ti*0.095,text_col)
        elif face == 'N':
            B(0.90,PLAQUE_D,PLAQUE_H, cx,wall_y+off,1.55,room_col)
            for ti,tw in enumerate([0.72,0.55,0.38]):
                B(tw,PLAQUE_D+0.002,0.030, cx,wall_y+off+0.001,1.65-ti*0.095,text_col)
        elif face == 'W':
            B(PLAQUE_D,0.90,PLAQUE_H, wall_y-off,cy,1.55,room_col)
            for ti,tw in enumerate([0.72,0.55,0.38]):
                B(PLAQUE_D+0.002,tw,0.030, wall_y-off-0.001,cy,1.65-ti*0.095,text_col)
        elif face == 'E':
            B(PLAQUE_D,0.90,PLAQUE_H, wall_y+off,cy,1.55,room_col)
            for ti,tw in enumerate([0.72,0.55,0.38]):
                B(PLAQUE_D+0.002,tw,0.030, wall_y+off+0.001,cy,1.65-ti*0.095,text_col)

    # ── 1. OPERATING ROOM — south wall interior (Y=OR_Y1=2) ──
    name_board(rcx(OR_X1,OR_X2), rcy(OR_Y1,OR_Y2), OR_Y1, 'N', '#1A4A6A')

    # ── 2. SCRUB / PREP ROOM — south wall interior ──
    name_board(rcx(SCRUB_X1,SCRUB_X2), rcy(SCRUB_Y1,SCRUB_Y2), SCRUB_Y1, 'N', '#1A5A3A')

    # ── 3. STERILE STORE — south wall interior ──
    name_board(rcx(STER_X1,STER_X2), rcy(STER_Y1,STER_Y2), STER_Y1, 'N', '#3A3A3A')

    # ── 4. STAFF ROOM — north wall interior (Y=STAFF_Y2=0) ──
    name_board(rcx(STAFF_X1,STAFF_X2), rcy(STAFF_Y1,STAFF_Y2), STAFF_Y2, 'S', '#6A4A1A')

    # ── 5. NURSES' STATION — north wall interior ──
    name_board(rcx(NURS_X1,NURS_X2), rcy(NURS_Y1,NURS_Y2), NURS_Y2, 'S', '#1A3A6A')

    # ── 6. RECOVERY ROOM — north wall interior ──
    name_board(rcx(REC_X1,REC_X2), rcy(REC_Y1,REC_Y2), REC_Y2, 'S', '#6A1A1A')

    # ── 7. EQUIPMENT STORE — north wall interior ──
    name_board(rcx(EQUIP_X1,EQUIP_X2), rcy(EQUIP_Y1,EQUIP_Y2), EQUIP_Y2, 'S', '#3A3A3A')

    # ── 8. CORRIDOR — large floor stripe label (no wall available both sides) ──
    # Instead: plaques on both corridor walls
    CCX=rcx(BLDG_X1,BLDG_X2); CCY=rcy(CORR_Y1,CORR_Y2)
    # North face of corridor (south face of upper rooms wall)
    B(4.50,PLAQUE_D,PLAQUE_H, CCX,CORR_Y2-WT-PLAQUE_D/2,1.55,'#2A5A7A')
    for ti2,tw2 in enumerate([3.60,2.80,1.80]):
        B(tw2,PLAQUE_D+0.002,0.030, CCX,CORR_Y2-WT-PLAQUE_D/2-0.001,1.65-ti2*0.095,'#FFFFFF')
    # Small door-side plaques in corridor indicating each room
    # These sit on the corridor-side of each partition wall at 1.40 m, flush against wall
    entries = [
        # (wall_Y, door_cx, label_col, face)
        (CORR_Y2, rcx(SCRUB_X1,SCRUB_X2), '#1A5A3A', 'S'),
        (CORR_Y2, rcx(OR_X1,OR_X2),       '#1A4A6A', 'S'),
        (CORR_Y2, rcx(STER_X1,STER_X2),   '#3A3A3A', 'S'),
        (CORR_Y1, rcx(STAFF_X1,STAFF_X2), '#6A4A1A', 'N'),
        (CORR_Y1, rcx(NURS_X1,NURS_X2),   '#1A3A6A', 'N'),
        (CORR_Y1, rcx(REC_X1,REC_X2),     '#6A1A1A', 'N'),
        (CORR_Y1, rcx(EQUIP_X1,EQUIP_X2), '#3A3A3A', 'N'),
    ]
    for wy,dcx2,lc,face2 in entries:
        off2 = WT + 0.010
        if face2=='S':
            B(0.45,0.012,0.12, dcx2,wy-off2,1.40,lc)
            for ti3,tw3 in enumerate([0.36,0.28]):
                B(tw3,0.010,0.025, dcx2,wy-off2-0.001,1.47-ti3*0.060,'#FFFFFF')
        else:
            B(0.45,0.012,0.12, dcx2,wy+off2,1.40,lc)
            for ti3,tw3 in enumerate([0.36,0.28]):
                B(tw3,0.010,0.025, dcx2,wy+off2+0.001,1.47-ti3*0.060,'#FFFFFF')

# ════════════════════════════════════════════════════════════
# OR CONTENTS
# ════════════════════════════════════════════════════════════
def build_or_contents():
    OCX=rcx(OR_X1,OR_X2); OCY=rcy(OR_Y1,OR_Y2)
    BX=OCX; BY=OCY+0.50

    # Lamp arm rail (wall-to-wall ceiling mount — attached to top of walls, not floating)
    B(0.030,1.50,0.026, BX,BY,WH-0.010,STEEL)
    # Lamp stem (hangs from rail down into room)
    B(0.025,0.025,0.50, BX,BY,WH-0.26,STEEL)
    B(0.22,0.22,0.022, BX,BY,WH-0.77,'#D0D8DC')
    for lx,ly,lr in [(0,0,0.12),(-0.20,-0.10,0.09),(0.20,-0.10,0.09)]:
        B(lr,lr*0.65,0.032, BX+lx,BY+ly,WH-0.81,'#C0C8CC')
        B(lr*0.65,lr,0.032, BX+lx,BY+ly,WH-0.81,'#C0C8CC')
        B(lr*0.70,lr*0.50,0.010, BX+lx,BY+ly,WH-0.845,LAMP_WHT)
        B(lr*0.50,lr*0.70,0.010, BX+lx,BY+ly,WH-0.845,LAMP_WHT)
        for dx2,dy2 in [(-0.04,0),(0.04,0),(0,-0.04),(0,0.04),(0,0)]:
            B(0.010,0.010,0.005, BX+lx+dx2,BY+ly+dy2,WH-0.858,'#FFFFF8')

    # Operating table
    LEG=0.20; BFH=0.055
    B(0.10,0.10,LEG, BX,BY,LEG,'#686E70')
    B(0.32,0.20,0.028, BX,BY,0.028,'#484E50')
    TZ=LEG*2+BFH
    B(1.00,0.44,BFH, BX,BY,TZ,'#8A9294')
    for dy in [-0.44,0.44]: B(1.00,0.016,BFH+0.020, BX,BY+dy,TZ,STEEL)
    for dx in [-0.99,0.99]: B(0.016,0.44,BFH+0.020, BX+dx,BY,TZ,STEEL)
    MZ=TZ+BFH*2+0.028
    B(0.97,0.41,0.028, BX,BY,MZ-0.014,'#1A6A78')
    B(0.96,0.40,0.035, BX,BY,MZ+0.012,'#1E7888')
    DZ=MZ+0.050
    B(0.94,0.38,0.007, BX,BY,DZ,'#2E6888')
    for dx3 in [-0.46,0,0.46]: B(0.005,0.37,0.004, BX+dx3,BY,DZ+0.008,'#265A78')
    # Patient
    PX=BX-0.76
    B(0.16,0.32,0.060, PX,BY,DZ+0.060,'#F8FAFA')
    B(0.10,0.085,0.095, PX+0.02,BY,DZ+0.150,SKIN_TON)
    B(0.045,0.060,0.022, PX+0.12,BY,DZ+0.162,'#90C8B0')
    for i in range(5): B(0.009,0.011,0.011, PX+0.16+i*0.028,BY-0.07,DZ+0.168,'#A0B8A8')
    B(0.011,0.34,0.013, PX-0.18,BY,DZ+0.190,STEEL)

    # Anaesthesia machine (NW corner)
    AMX=OR_X1+0.70; AMY=OR_Y1+0.90
    B(0.24,0.20,0.68, AMX,AMY,0.68,'#D0D8DC')
    B(0.011,0.18,0.58, AMX+0.24,AMY,0.70,'#2A3038')
    B(0.009,0.14,0.11, AMX+0.248,AMY,0.94,SCRN_BLK)
    for wi,wc in enumerate([SCRN_GRN,'#FF6060','#60C0FF']):
        B(0.007,0.11,0.006, AMX+0.253,AMY,1.00-wi*0.065,wc)
    for ky in [-0.07,-0.02,0.02,0.07]: B(0.013,0.013,0.013, AMX+0.248,AMY+ky,0.75,'#707880')
    for li,lc in enumerate(['#00FF00','#FFAA00','#FF3030','#00AAFF']):
        B(0.007,0.007,0.005, AMX+0.253,AMY-0.09+li*0.060,0.64,lc)
    for vy,vc in [(-0.07,'#C0A040'),(0.07,'#40A0C0')]:
        B(0.024,0.024,0.09, AMX+0.22,AMY+vy,1.26,vc)
        B(0.017,0.017,0.013, AMX+0.22,AMY+vy,1.363,'#808080')
    for cdy,cc in [(-0.11,'#20A840'),(0.11,'#2060C0')]:
        B(0.044,0.044,0.38, AMX-0.07,AMY+cdy,0.40,cc)
        B(0.030,0.030,0.065, AMX-0.07,AMY+cdy,0.845,'#C0C0C0')

    # Vital signs monitor — wall bracket mount on west wall
    VMX=OR_X1+0.15; VMY=BY-0.80; VMZ=1.50
    B(0.013,0.22,0.155, VMX,VMY,VMZ,'#1A2028')
    B(0.009,0.20,0.135, VMX,VMY,VMZ,SCRN_BLK)
    for wi2,wc2 in enumerate([SCRN_GRN,'#FF8060','#60C0FF','#FFDD00']):
        B(0.007,0.18,0.005, VMX,VMY,VMZ+0.085-wi2*0.050,wc2)
        B(0.007,0.034,0.018, VMX,VMY+0.162,VMZ+0.085-wi2*0.050,wc2)
    B(0.009,0.016,0.016, VMX,VMY-0.195,VMZ+0.120,'#FF4040')
    B(0.040,0.010,0.010, VMX+0.025,VMY,VMZ+0.110,STEEL)
    B(0.006,0.010,0.22, VMX+0.064,VMY,VMZ,STEEL)

    # Scrub nurse instrument trolley
    STX=OR_X1+1.80; STY=OR_Y1+0.70
    for stlx,stly in [(-0.34,-0.24),(0.34,-0.24),(-0.34,0.24),(0.34,0.24)]:
        B(0.015,0.015,0.50, STX+stlx,STY+stly,0.50,'#A0A8B0')
    B(0.36,0.26,0.011, STX,STY,0.970,'#C0CCD4')
    for sex,sey,sew,sed in [(0.36,0,0.011,0.26),(-0.36,0,0.011,0.26),
                             (0,0.26,0.36,0.011),(0,-0.26,0.36,0.011)]:
        B(sew,sed,0.024, STX+sex,STY+sey,0.985,'#909AA0')
    B(0.34,0.24,0.004, STX,STY,0.988,'#3070A0')

    # Crash cart (SE corner)
    CCX=OR_X2-0.80; CCY=OR_Y1+0.80
    B(0.20,0.14,0.48, CCX,CCY,0.48,'#E04020')
    for di in range(4):
        dz2=0.09+di*0.20
        B(0.011,0.11,0.075, CCX+0.20,CCY,dz2+0.038,'#C03010')
        B(0.015,0.045,0.011, CCX+0.208,CCY,dz2+0.038,'#D0D8DC')
    B(0.13,0.10,0.065, CCX,CCY,1.005,'#202830')
    B(0.10,0.078,0.009, CCX,CCY,1.068,SCRN_BLK)
    B(0.085,0.058,0.005, CCX,CCY,1.075,'#10C840')

    # IV pole
    IVX=BX-1.10; IVY=BY+0.50
    B(0.028,0.017,0.015, IVX,IVY,0.015,'#404848')
    B(0.009,0.009,0.95, IVX,IVY,0.95,'#B0B8C0')
    B(0.065,0.007,0.007, IVX,IVY,1.88,'#9098A0')
    for hxo in [-0.058,0.058]: B(0.007,0.007,0.027, IVX+hxo,IVY,1.915,'#C0C8D0')
    B(0.044,0.011,0.070, IVX,IVY,1.820,'#C8E8F0')

    # Wall clock (west wall — physically attached)
    CKX=OR_X1+0.13; CKY=OCY; CKZ=1.90
    B(0.013,0.10,0.10, CKX,CKY,CKZ,'#E8ECEE')
    B(0.009,0.086,0.086, CKX+0.011,CKY,CKZ,'#F8FAFA')
    for hm in range(12):
        ang=hm*math.pi/6
        B(0.005,0.005,0.007, CKX+0.017,CKY+0.072*math.sin(ang),CKZ+0.072*math.cos(ang),'#303838')
    B(0.004,0.004,0.038, CKX+0.017,CKY+0.018,CKZ+0.013,'#202828')
    B(0.004,0.004,0.054, CKX+0.017,CKY-0.025,CKZ+0.022,'#303838')
    B(0.003,0.003,0.052, CKX+0.017,CKY+0.030,CKZ-0.020,'#E04020')

    # Robot pedestal
    PDX=BX+0.10; PDY=BY-0.70
    B(0.26,0.26,0.032, PDX,PDY,0.032,'#2A3540')
    for bxo,byo in [(-0.20,-0.20),(0.20,-0.20),(-0.20,0.20),(0.20,0.20)]:
        B(0.018,0.018,0.015, PDX+bxo,PDY+byo,0.078,TEAL_DRK)
    for zh in range(6):
        z2=0.066+zh*0.034; r=0.108
        B(r,r*0.44,0.017, PDX,PDY,z2,TEAL_MID if zh%2==0 else '#505E6A')
        B(r*0.44,r,0.017, PDX,PDY,z2,TEAL_MID if zh%2==0 else '#505E6A')
    lt=0.066+6*0.034; uz=lt+0.052
    B(0.145,0.145,0.026, PDX,PDY,lt+0.026,'#2A3540')
    for zh2 in range(3):
        z3=uz+zh2*0.028; r2=0.090
        B(r2,r2*0.44,0.014, PDX,PDY,z3,'#505E6A')
        B(r2*0.44,r2,0.014, PDX,PDY,z3,'#4A5860')
    B(0.135,0.135,0.022, PDX,PDY,uz+3*0.028+0.022,'#2A3540')

    # Instrument tray
    TRX=PDX+0.65; TRY=PDY; TRH=0.80
    for tlx,tly in [(-0.30,-0.20),(0.30,-0.20),(-0.30,0.20),(0.30,0.20)]:
        B(0.014,0.014,TRH/2, TRX+tlx,TRY+tly,TRH/2,'#A0A8B0')
        B(0.024,0.024,0.013, TRX+tlx,TRY+tly,TRH*0.55,TEAL_MID)
    for tly2 in [-0.20,0.20]: B(0.30,0.009,0.009, TRX,TRY+tly2,TRH*0.42,'#909AA0')
    for tlx2 in [-0.30,0.30]: B(0.009,0.20,0.009, TRX+tlx2,TRY,TRH*0.42,'#909AA0')
    B(0.29,0.19,0.008, TRX,TRY,TRH*0.42-0.040,'#B8C4CC')
    B(0.34,0.23,0.010, TRX,TRY,TRH+0.010,'#C0CCD4')
    for tex,tey,tew,ted in [(0.34,0,0.012,0.23),(-0.34,0,0.012,0.23),
                              (0,0.23,0.34,0.012),(0,-0.23,0.34,0.012)]:
        B(tew,ted,0.030, TRX+tex,TRY+tey,TRH+0.030,'#A8B4BC')
    B(0.32,0.21,0.005, TRX,TRY,TRH+0.024,'#2A6888')
    OZ=TRH+0.028
    SCX2=TRX-0.16
    for i in range(7): B(0.009,0.042-i*0.001,0.009, SCX2,TRY+0.022-i*0.016,OZ+0.010,'#C8D0D8')
    B(0.005,0.032,0.005, SCX2,TRY+0.110,OZ+0.006,'#E0E8F0')
    FCX2=TRX
    for i in range(10):
        B(0.006,0.006,0.007, FCX2-0.007,TRY-0.055+i*0.020,OZ+0.007,'#C0C8D0')
        B(0.006,0.006,0.007, FCX2+0.007,TRY-0.055+i*0.020,OZ+0.007,'#C8D0D8')
    SKX2=TRX+0.16
    B(0.048,0.058,0.011, SKX2,TRY,OZ+0.011,'#E8E0C8')
    B(0.032,0.032,0.003, SKX2,TRY,OZ+0.024,'#3060A0')

    return PDX,PDY,TRX,TRY,TRH,OZ,SCX2,FCX2,SKX2

# ════════════════════════════════════════════════════════════
# SCRUB / PREP ROOM
# ════════════════════════════════════════════════════════════
def build_scrub():
    RCX=rcx(SCRUB_X1,SCRUB_X2); RCY=rcy(SCRUB_Y1,SCRUB_Y2)
    for sxo in [-0.80,0.0,0.80]:
        SX2=RCX+sxo; SY2=SCRUB_Y1+0.40
        B(0.32,0.24,0.50, SX2,SY2,0.50,'#C8D0D4')
        B(0.22,0.15,0.042, SX2,SY2+0.04,0.963,'#E0EAF0')
        B(0.20,0.13,0.013, SX2,SY2+0.04,0.915,'#C0D0D8')
        B(0.007,0.007,0.085, SX2,SY2-0.05,1.008,'#C0C8D0')
        B(0.042,0.007,0.007, SX2,SY2-0.05,1.095,'#C0C8D0')
        B(0.024,0.018,0.054, SX2+0.20,SY2,1.020,'#E04080')
        B(0.011,0.011,0.013, SX2+0.20,SY2,1.088,'#C03060')
        B(0.019,0.042,0.064, SX2-0.24,SY2+0.08,1.28,'#E8EEF0')
    for hyo in [-1.80,-0.90,0.0,0.90,1.80]:
        HX2=SCRUB_X1+0.05; HY2=RCY+hyo; HZ2=1.60
        B(0.020,0.013,0.022, HX2,HY2,HZ2,'#A0A8B0')
        B(0.005,0.005,0.055, HX2+0.025,HY2,HZ2,'#808890')
        if abs(hyo)<2:
            B(0.016,0.007,0.20, HX2+0.025,HY2,HZ2-0.12,'#90C8B8')
            B(0.044,0.007,0.14, HX2+0.025,HY2,HZ2-0.31,'#90C8B8')
    B(rhw(SCRUB_X1,SCRUB_X2)-0.12,0.24,0.50, RCX,SCRUB_Y2-0.27,0.50,'#C4D0D4')
    B(rhw(SCRUB_X1,SCRUB_X2)-0.10,0.24,0.013, RCX,SCRUB_Y2-0.27,1.013,'#D4E0E4')
    for si,(scol,shw2,shd2,shh2) in enumerate([('#3080C0',0.045,0.045,0.090),
                                                ('#E8EEF0',0.11,0.065,0.13),
                                                ('#2060A0',0.034,0.034,0.11)]):
        B(shw2,shd2,shh2, RCX-0.65+si*0.65,SCRUB_Y2-0.27,1.026+shh2,scol)
    B(rhw(SCRUB_X1,SCRUB_X2)-0.16,0.16,0.013, RCX,SCRUB_Y2-0.17,1.55,'#C0CCD4')
    for boi in range(5): B(0.045,0.085,0.11, RCX-0.90+boi*0.45,SCRUB_Y2-0.17,1.618,'#E8F0F4')
    # Floor drain
    B(0.085,0.085,0.006, RCX,RCY+0.30,0.006,'#606868')
    for di in range(5): B(0.075,0.007,0.004, RCX,RCY+0.26+di*0.033,0.009,'#484E50')

# ════════════════════════════════════════════════════════════
# STERILE STORE
# ════════════════════════════════════════════════════════════
def build_sterile_store():
    RCX=rcx(STER_X1,STER_X2); RCY=rcy(STER_Y1,STER_Y2)
    shelf_cols=['#3060C0','#E8EEF0','#20A040','#D0D8DC','#C04020','#2080C0','#A040C0','#D0C840']
    for wx2 in [STER_X1+0.18,STER_X2-0.18]:
        B(0.14,rhh(STER_Y1,STER_Y2)-0.24,0.95, wx2,RCY,0.95,'#A8B4BC')
        for shz in [0.28,0.58,0.88,1.18]:
            B(0.135,rhh(STER_Y1,STER_Y2)-0.26,0.005, wx2,RCY,shz,'#C0CCD4')
        for bi2,bz2 in enumerate([0.30,0.60,0.90,1.20]):
            for bj2 in range(4):
                cy4=STER_Y1+0.50+bj2*1.30
                B(0.030,0.12,0.085, wx2,cy4,bz2+0.043,shelf_cols[(bi2*4+bj2)%8])
    for tl in [(-0.15,-0.42),(0.15,-0.42),(-0.15,0.42),(0.15,0.42)]:
        B(0.013,0.013,0.38, RCX+tl[0],RCY+tl[1],0.38,'#A0A8B0')
    B(0.17,0.44,0.011, RCX,RCY,0.81,'#C0CCD4')
    B(0.17,0.44,0.011, RCX,RCY,0.50,'#C0CCD4')
    for ci3,cc3 in enumerate(['#FF4040','#4080C0','#40C080','#C0C040']):
        B(0.030,0.040,0.045, RCX-0.10+ci3*0.067,RCY,0.862,cc3)
    # Wall-mounted temp monitor
    B(0.009,0.10,0.07, STER_X2-0.12,RCY,1.80,'#1A2028')
    B(0.007,0.088,0.058, STER_X2-0.114,RCY,1.80,SCRN_BLK)
    B(0.006,0.075,0.012, STER_X2-0.112,RCY,1.80,'#00C860')

# ════════════════════════════════════════════════════════════
# RECOVERY ROOM
# ════════════════════════════════════════════════════════════
def build_recovery():
    RCX=rcx(REC_X1,REC_X2); RCY=rcy(REC_Y1,REC_Y2)
    for ctx in [4.50,6.00,7.50]:
        B(0.006,rhh(REC_Y1,REC_Y2),0.011, ctx,RCY,WH-0.085,'#C8D0D4')
        B(0.005,rhh(REC_Y1,REC_Y2)-0.20,0.85, ctx,RCY,WH-0.085-0.45,'#A0C4D8')
        for hky in range(-5,6):
            B(0.004,0.004,0.032, ctx,RCY+hky*0.48,WH-0.100,'#707880')
    for bay_x in [3.20,5.10,6.90,8.80]:
        BY2=RCY
        for lx3,ly3 in [(-0.60,-0.26),(0.60,-0.26),(-0.60,0.26),(0.60,0.26)]:
            B(0.022,0.022,0.22, bay_x+lx3,BY2+ly3,0.22,'#7A8088')
        B(0.63,0.28,0.042, bay_x,BY2,0.47,'#8A9294')
        for rs in [-0.28,0.28]: B(0.63,0.013,0.052, bay_x,BY2+rs,0.522,STEEL)
        B(0.61,0.26,0.054, bay_x,BY2,0.578,'#E0ECF4')
        B(0.11,0.20,0.044, bay_x-0.48,BY2,0.650,'#F8FAFA')
        B(0.42,0.16,0.058, bay_x+0.12,BY2,0.650,'#E8C4A0')
        B(0.085,0.080,0.080, bay_x-0.44,BY2,0.682,SKIN_TON)
        B(0.42,0.26,0.022, bay_x+0.12,BY2,0.668,LIGHT_BLUE)
        for rs2 in [-0.26,0.26]: B(0.35,0.008,0.12, bay_x+0.18,BY2+rs2,0.65,'#9098A0')
        B(0.007,0.007,0.34, bay_x+0.68,BY2-0.20,0.84,STEEL)
        B(0.014,0.12,0.10, bay_x+0.68,BY2-0.20,1.18,'#1A2028')
        B(0.010,0.105,0.085, bay_x+0.685,BY2-0.20,1.18,SCRN_BLK)
        for wl2,wc4 in enumerate([SCRN_GRN,'#FF8060','#60C0FF']):
            B(0.006,0.085,0.005, bay_x+0.688,BY2-0.20,1.20-wl2*0.040,wc4)
        B(0.007,0.007,0.90, bay_x-0.72,BY2+0.22,0.90,'#B0B8C0')
        B(0.048,0.005,0.005, bay_x-0.72,BY2+0.22,1.78,'#9098A0')
        B(0.034,0.009,0.058, bay_x-0.72,BY2+0.22,1.73,'#D0E8F0')
        B(0.009,0.044,0.054, bay_x-0.64,BY2+0.25,1.14,'#E8EEF0')
        B(0.007,0.015,0.015, bay_x-0.64,BY2+0.252,1.155,'#FF4040')
        # Window (south wall — physically part of wall)
        B(0.009,WT+0.02,0.42, bay_x,REC_Y1+0.01,1.35,'#C8DCE8')
        B(0.006,WT+0.015,0.40, bay_x,REC_Y1+0.008,1.35,'#D8EAF4')
        for bl in range(9): B(0.004,WT+0.013,0.005, bay_x,REC_Y1+0.006,1.16+bl*0.048,'#D0E4F0')
        B(0.15,WT+0.03,0.012, bay_x,REC_Y1+0.010,0.92,'#C8D4D8')
    # Nurses chart desk
    CSX=REC_X2-0.35; CSY=RCY
    B(0.22,0.16,0.50, CSX,CSY,0.50,'#C0CCD4')
    B(0.24,0.18,0.013, CSX,CSY,1.013,'#D0DCE4')
    B(0.017,0.11,0.13, CSX,CSY+0.17,1.10,'#1A2028')
    B(0.011,0.095,0.11, CSX,CSY+0.172,1.10,SCRN_BLK)
    B(0.009,0.082,0.025, CSX,CSY+0.173,1.106,'#20A840')
    # Defibrillator cart
    DEFX=REC_X1+0.35; DEFY=RCY+2.40
    B(0.18,0.13,0.50, DEFX,DEFY,0.50,'#E04020')
    B(0.13,0.10,0.065, DEFX,DEFY,1.010,'#202830')
    B(0.10,0.078,0.009, DEFX,DEFY,1.074,SCRN_BLK)
    B(0.085,0.058,0.005, DEFX,DEFY,1.080,'#10C840')

# ════════════════════════════════════════════════════════════
# NURSES' STATION
# ════════════════════════════════════════════════════════════
def build_nurses_station():
    RCX=rcx(NURS_X1,NURS_X2); RCY=rcy(NURS_Y1,NURS_Y2)
    for dxo,dyo,dhw,dhd in [(0,1.40,2.20,0.22),(0,-1.40,2.20,0.22),(2.00,0,0.22,1.62)]:
        B(dhw,dhd,0.50, RCX+dxo,RCY+dyo,0.50,'#C0CCD4')
        B(dhw+0.02,dhd+0.02,0.013, RCX+dxo,RCY+dyo,1.013,'#D0DCE4')
    for cx6,cy6 in [(-1.0,1.40),(1.0,1.40),(-1.0,-1.40),(1.0,-1.40)]:
        B(0.015,0.13,0.11, RCX+cx6,RCY+cy6+0.06,1.10,'#1A2028')
        B(0.010,0.115,0.095, RCX+cx6,RCY+cy6+0.062,1.10,SCRN_BLK)
        B(0.009,0.095,0.025, RCX+cx6,RCY+cy6+0.063,1.108,'#20A840')
        B(0.007,0.112,0.046, RCX+cx6,RCY+cy6-0.010,0.982,'#2A3038')
    MCX=RCX-2.10; MCY=RCY
    B(0.25,0.16,0.90, MCX,MCY,0.90,'#D0D8DC')
    B(0.006,0.14,0.88, MCX+0.244,MCY,0.90,'#C0C8CE')
    B(0.008,0.020,0.020, MCX+0.248,MCY,0.90,'#D0A020')
    SHX3=NURS_X2-0.13; SHY3=RCY
    B(0.055,0.65,0.90, SHX3+0.055,SHY3,0.90,'#C0CCD4')
    for row3 in range(4):
        for col4 in range(-2,3):
            B(0.045,0.11,0.004, SHX3+0.047,SHY3+col4*0.26,0.26+row3*0.40,'#A8B4BC')
    for col5 in range(-3,4):
        B(0.045,0.004,0.40*4, SHX3+0.047,SHY3+col5*0.26,0.90,'#A0ACB4')
    for row4 in range(4):
        for col6 in range(-2,3):
            if (row4+col6)%3!=0:
                fc2=['#3060C0','#C04020','#20A040','#A04080','#D08020'][(row4*5+col6)%5]
                B(0.038,0.09,0.16, SHX3+0.042,SHY3+col6*0.26,0.35+row4*0.40,fc2)
    # Bulletin board (wall-mounted)
    B(0.009,0.55,0.36, NURS_X1+0.12,RCY,1.75,'#D4A860')
    B(0.007,0.56,0.34, NURS_X1+0.122,RCY,1.75,'#F4E8C8')
    for bpi3 in range(3):
        for bpj3 in range(2):
            B(0.006,0.13,0.09, NURS_X1+0.124,RCY-0.30+bpi3*0.30,1.67+bpj3*0.18,
              ['#FFFFFF','#FFFFD0','#D0F0FF'][bpi3])
    # Emergency strip (on south wall face)
    for ez2 in [0.50,0.80,1.10,1.40,1.70]:
        B(0.005,0.38,0.016, RCX,NURS_Y1+0.12,ez2,'#FF3018')
    # Patient tracking whiteboard (on east partition wall)
    B(0.009,0.80,0.50, NURS_X2-0.12,RCY,1.50,'#E8EEF0')
    B(0.007,0.76,0.46, NURS_X2-0.114,RCY,1.50,'#FFFFFF')
    for row5 in range(4):
        B(0.005,0.74,0.004, NURS_X2-0.112,RCY,1.28+row5*0.12,'#C8D0D4')

# ════════════════════════════════════════════════════════════
# STAFF ROOM
# ════════════════════════════════════════════════════════════
def build_staff_room():
    RCX=rcx(STAFF_X1,STAFF_X2); RCY=rcy(STAFF_Y1,STAFF_Y2)
    for li3 in range(6):
        LX2=STAFF_X1+0.12; LY2=STAFF_Y1+0.40+li3*0.90
        B(0.10,0.38,0.90, LX2,LY2,0.90,'#C8D4D8')
        B(0.006,0.36,0.88, LX2+0.10,LY2,0.90,'#B8C4C8')
        B(0.008,0.020,0.020, LX2+0.106,LY2,0.90,'#D0A020')
    TBX2=RCX+0.50; TBY2=RCY
    B(0.60,0.45,0.018, TBX2,TBY2,0.760,'#D8C8A8')
    for leg in [(-0.55,-0.40),(0.55,-0.40),(-0.55,0.40),(0.55,0.40)]:
        B(0.016,0.016,0.38, TBX2+leg[0],TBY2+leg[1],0.38,'#908070')
    for chx,chy in [(-0.80,0),(0.80,0),(0,-0.70),(0,0.70)]:
        B(0.22,0.20,0.030, TBX2+chx,TBY2+chy,0.460,'#D08060')
        for cl in [(-0.18,-0.16),(0.18,-0.16),(-0.18,0.16),(0.18,0.16)]:
            B(0.013,0.013,0.23, TBX2+chx+cl[0],TBY2+chy+cl[1],0.23,'#A06040')
    KX2=STAFF_X2-0.22; KY2=RCY
    B(0.20,rhh(STAFF_Y1,STAFF_Y2)-0.20,0.50, KX2,KY2,0.50,'#C8D0D4')
    B(0.21,rhh(STAFF_Y1,STAFF_Y2)-0.18,0.013, KX2,KY2,1.013,'#D4DCE0')
    B(0.10,0.09,0.24, KX2,KY2+1.50,1.144,'#2A2A2A')
    B(0.14,0.09,0.075, KX2,KY2-1.50,1.062,'#1A1A1A')
    B(0.18,0.16,0.85, KX2,KY2,0.85,'#D4D8D4')
    B(0.005,0.14,0.83, KX2+0.178,KY2,0.85,'#C4C8C4')
    CX3=STAFF_X1+0.45; CY3=STAFF_Y2-0.60
    B(0.28,0.60,0.055, CX3,CY3,0.455,'#8090A8')
    B(0.28,0.030,0.28, CX3,CY3+0.60,0.595,'#6070A0')
    for arml in [-0.28,0.28]: B(0.030,0.60,0.12, CX3+arml,CY3,0.560,'#7080A8')
    # Notice board (on north wall)
    B(0.009,0.60,0.38, RCX,STAFF_Y2-0.12,1.72,'#D4A860')
    B(0.007,0.56,0.34, RCX,STAFF_Y2-0.122,1.72,'#F4E8C8')

# ════════════════════════════════════════════════════════════
# EQUIPMENT STORE
# ════════════════════════════════════════════════════════════
def build_equipment_store():
    RCX=rcx(EQUIP_X1,EQUIP_X2); RCY=rcy(EQUIP_Y1,EQUIP_Y2)
    ecols=['#3060C0','#808080','#D0A020','#A03020']
    for wx3 in [EQUIP_X1+0.18,EQUIP_X2-0.18]:
        B(0.14,rhh(EQUIP_Y1,EQUIP_Y2)-0.20,0.95, wx3,RCY,0.95,'#9098A0')
        for shz2 in [0.26,0.56,0.86,1.16]:
            B(0.135,rhh(EQUIP_Y1,EQUIP_Y2)-0.22,0.005, wx3,RCY,shz2,'#B0BCC4')
        for bi3,bz3 in enumerate([0.29,0.59,0.89,1.19]):
            for bj3 in range(3):
                cy5=EQUIP_Y1+0.60+bj3*1.60
                B(0.030,0.22,0.12, wx3,cy5,bz3+0.060,ecols[(bi3+bj3)%4])
    EX2=RCX; EY2=RCY+1.50
    B(0.011,0.011,1.10, EX2,EY2,1.10,'#B0B8C0')
    B(0.12,0.08,0.06, EX2,EY2,2.14,'#303840')
    B(0.10,0.065,0.005, EX2,EY2,2.174,SCRN_BLK)
    B(0.060,0.008,0.060, EX2-0.08,EY2,2.08,'#606870')
    UX2=RCX; UY2=RCY-1.50
    for ull in [(-0.15,-0.20),(0.15,-0.20),(-0.15,0.20),(0.15,0.20)]:
        B(0.013,0.013,0.38, UX2+ull[0],UY2+ull[1],0.38,'#A0A8B0')
    B(0.17,0.22,0.011, UX2,UY2,0.801,'#C0CCD4')
    B(0.15,0.14,0.24, UX2,UY2+0.06,0.984,'#1A2028')
    B(0.012,0.12,0.18, UX2,UY2+0.062,0.975,SCRN_BLK)
    for ocx in [-0.40,0.0,0.40]:
        B(0.048,0.048,0.60, RCX+ocx,EQUIP_Y2-0.55,0.60,'#20A840')
        B(0.034,0.034,0.065, RCX+ocx,EQUIP_Y2-0.55,1.268,'#C0C0C0')
        B(0.018,0.018,0.022, RCX+ocx,EQUIP_Y2-0.55,1.355,'#808080')
    B(0.55,0.08,0.040, RCX,EQUIP_Y2-0.55,0.80,'#A0A8B0')
    B(0.55,0.08,0.040, RCX,EQUIP_Y2-0.55,0.40,'#A0A8B0')

# ════════════════════════════════════════════════════════════
# CORRIDOR
# ════════════════════════════════════════════════════════════
def build_corridor():
    CCX=rcx(BLDG_X1,BLDG_X2); CCY=rcy(CORR_Y1,CORR_Y2)
    # Handrails (posts + rail — all grounded, no floating)
    for hx4 in range(-8,14):
        B(0.011,0.011,0.93, hx4,CORR_Y2-0.15,0.93,'#8090A0')
        B(0.011,0.011,0.93, hx4,CORR_Y1+0.15,0.93,'#8090A0')
    B(rhw(BLDG_X1,BLDG_X2)-0.10,0.020,0.020, CCX,CORR_Y2-0.15,0.960,'#7888A0')
    B(rhw(BLDG_X1,BLDG_X2)-0.10,0.020,0.020, CCX,CORR_Y1+0.15,0.960,'#7888A0')
    # Gurney
    GX3=5.0; GY3=CCY
    for glx3,gly3 in [(-0.80,-0.28),(0.80,-0.28),(-0.80,0.28),(0.80,0.28)]:
        B(0.022,0.022,0.36, GX3+glx3,GY3+gly3,0.36,'#A0A8B0')
    B(0.84,0.30,0.042, GX3,GY3,0.760,'#C0CCD4')
    B(0.82,0.28,0.032, GX3,GY3,0.814,'#F0F8FA')
    B(0.13,0.26,0.064, GX3-0.66,GY3,0.892,'#F8FAFA')
    B(0.007,0.007,0.90, GX3+0.96,GY3,0.90,'#B0B8C0')
    B(0.036,0.010,0.060, GX3+0.96,GY3,1.730,'#D0E8F0')
    # Wheelchair
    WCX3=-5.0; WCY3=CORR_Y2-0.40
    B(0.24,0.20,0.028, WCX3,WCY3,0.50,'#4880C0')
    B(0.022,0.20,0.20, WCX3,WCY3,0.70,'#3060A0')
    for wxo3 in [-0.20,0.20]:
        B(0.075,0.075,0.013, WCX3+wxo3,WCY3-0.18,0.33,'#707888')
        B(0.038,0.038,0.009, WCX3+wxo3,WCY3+0.18,0.16,'#606878')
    # Fire extinguisher (floor-standing, against east end wall)
    B(0.044,0.044,0.20, 11.80,CORR_Y1+0.12,0.70,'#CC2010')
    B(0.022,0.022,0.032, 11.80,CORR_Y1+0.12,0.932,'#B01808')
    B(0.007,0.007,0.065, 11.80,CORR_Y1+0.12,0.978,'#808080')

# ════════════════════════════════════════════════════════════
# SETUP
# ════════════════════════════════════════════════════════════
def setup_world():
    bb.addGroundPlane()
    build_perimeter()
    build_partitions()
    build_floors()
    build_room_name_plaques()
    PDX,PDY,TRX,TRY,TRH,OZ,SCX3,FCX3,SKX3 = build_or_contents()
    build_scrub()
    build_sterile_store()
    build_recovery()
    build_nurses_station()
    build_staff_room()
    build_equipment_store()
    build_corridor()
    return PDX,PDY,TRX,TRY,TRH,OZ,SCX3,FCX3,SKX3

# ════════════════════════════════════════════════════════════
# INIT
# ════════════════════════════════════════════════════════════
bb.setGravity(0,0,0)
DOWN_QUAT=bb.getQuaternionFromEuler([math.pi,0,0])
PDX,PDY,TRX,TRY,TRH,OZ,SC_X,FC_X,SK_X = setup_world()

robot=bb.loadURDF('panda.urdf',[PDX,PDY,PEDESTAL_TOP_Z],
       bb.getQuaternionFromEuler([0,0,math.pi/2]),fixedBase=True)
ee_link=10
for i,jp in enumerate(home_arm_jpos):
    bb.setJointMotorControl(robot,i,targetPosition=jp); cur_j[i]=jp

scalpel_id=bb.createBody('box',halfExtent=[0.010,0.075,0.010],
    position=[SC_X,TRY,OZ+0.010],color='#C8D0D8',mass=0)
forceps_id=bb.createBody('box',halfExtent=[0.010,0.095,0.010],
    position=[FC_X,TRY,OZ+0.010],color='#8090A0',mass=0)
suture_id =bb.createBody('box',halfExtent=[0.048,0.058,0.012],
    position=[SK_X,TRY,OZ+0.012],color='#3060A0',mass=0)

OBJ_IDS ={'scalpel':scalpel_id,'forceps':forceps_id,'suture':suture_id}
OBJ_PICK={'scalpel':[SC_X,TRY,OZ+0.015],'forceps':[FC_X,TRY,OZ+0.015],'suture':[SK_X,TRY,OZ+0.016]}
ORIGIN  ={'scalpel':[SC_X,TRY,OZ+0.010],'forceps':[FC_X,TRY,OZ+0.010],'suture':[SK_X,TRY,OZ+0.012]}
OCX2=rcx(OR_X1,OR_X2); OCY2=rcy(OR_Y1,OR_Y2)
BED_DROPS=[[OCX2+0.20,OCY2-0.15,0.82],[OCX2+0.20,OCY2+0.00,0.82],[OCX2+0.20,OCY2+0.15,0.82]]

for jname,jdef,jlo,jhi in zip(JOINT_NAMES,home_arm_jpos,JOINT_LO,JOINT_HI):
    bb.addDebugSlider(jname,jdef,jlo,jhi)
bb.addDebugButton('Pick Scalpel')
bb.addDebugButton('Pick Forceps')
bb.addDebugButton('Pick Suture')
bb.addDebugButton('Drop on Table')
bb.addDebugButton('Return Home')
bb.addDebugButton('Reset All')
old_vals={'Pick Scalpel':0,'Pick Forceps':0,'Pick Suture':0,'Drop on Table':0,'Return Home':0,'Reset All':0}

# ════════════════════════════════════════════════════════════
# MOTION
# ════════════════════════════════════════════════════════════
def get_ee_pos():
    try: return list(bb.getLinkState(robot,ee_link)[0])
    except: return [0.0,0.0,1.0]
def ik(pos): return list(bb.calculateInverseKinematics(robot,ee_link,pos,DOWN_QUAT))
def ease(t):
    t=max(0.0,min(1.0,t)); return t*t*(3.0-2.0*t)
def snap_to_ee(obj_name):
    if obj_name and obj_name in OBJ_IDS:
        try: bb.resetBasePositionAndOrientation(OBJ_IDS[obj_name],get_ee_pos(),ZERO_QUAT)
        except: pass
def place_obj(obj_name,pos):
    if obj_name and obj_name in OBJ_IDS:
        try: bb.resetBasePositionAndOrientation(OBJ_IDS[obj_name],[float(pos[0]),float(pos[1]),float(pos[2])],ZERO_QUAT)
        except: pass
def smooth_move(target_j,steps=60,carrying=False,carry_obj=''):
    global cur_j
    start_j=list(cur_j)
    for s in range(steps+1):
        t=s/max(steps,1); t2=ease(t)
        for i in range(7):
            v=start_j[i]+(target_j[i]-start_j[i])*t2
            bb.setJointMotorControl(robot,i,targetPosition=float(v)); cur_j[i]=float(v)
        if carrying and carry_obj: snap_to_ee(carry_obj)
        time.sleep(0.018)
def do_pick(obj):
    global held_obj
    pick=OBJ_PICK[obj]; above=[pick[0],pick[1],pick[2]+0.22]
    smooth_move(ik(above),steps=70)
    smooth_move(ik(pick),steps=40,carrying=True,carry_obj=obj)
    held_obj=obj; snap_to_ee(obj)
    smooth_move(ik(above),steps=45,carrying=True,carry_obj=obj)
    smooth_move(home_arm_jpos,steps=65,carrying=True,carry_obj=obj)
def do_drop(slot):
    global held_obj
    obj=held_obj; da=[slot[0],slot[1],slot[2]+0.22]
    smooth_move(ik(da),steps=65,carrying=True,carry_obj=obj)
    smooth_move(ik(slot),steps=40,carrying=True,carry_obj=obj)
    place_obj(obj,slot); held_obj=''
    smooth_move(ik(da),steps=40)
    smooth_move(home_arm_jpos,steps=60)
def do_home():
    smooth_move(home_arm_jpos,steps=60,carrying=(held_obj!=''),carry_obj=held_obj)
def do_reset():
    global held_obj,drop_idx
    held_obj=''; drop_idx=0
    for name,pos in ORIGIN.items(): place_obj(name,pos)
    smooth_move(home_arm_jpos,steps=40)

# ════════════════════════════════════════════════════════════
# MAIN LOOP
# ════════════════════════════════════════════════════════════
print('=== HOSPITAL READY ===')
print('Rooms: SCRUB/PREP | OPERATING ROOM | STERILE STORE')
print('       CENTRAL CORRIDOR')
print('       STAFF ROOM | NURSES STATION | RECOVERY ROOM | EQUIPMENT STORE')

while True:
    vps=bb.readDebugParameter('Pick Scalpel')
    vpf=bb.readDebugParameter('Pick Forceps')
    vpu=bb.readDebugParameter('Pick Suture')
    vd =bb.readDebugParameter('Drop on Table')
    vh =bb.readDebugParameter('Return Home')
    vr =bb.readDebugParameter('Reset All')
    if vps>old_vals['Pick Scalpel']:
        old_vals['Pick Scalpel']=vps
        if held_obj: place_obj(held_obj,ORIGIN[held_obj]); held_obj=''
        do_pick('scalpel')
    elif vpf>old_vals['Pick Forceps']:
        old_vals['Pick Forceps']=vpf
        if held_obj: place_obj(held_obj,ORIGIN[held_obj]); held_obj=''
        do_pick('forceps')
    elif vpu>old_vals['Pick Suture']:
        old_vals['Pick Suture']=vpu
        if held_obj: place_obj(held_obj,ORIGIN[held_obj]); held_obj=''
        do_pick('suture')
    elif vd>old_vals['Drop on Table']:
        old_vals['Drop on Table']=vd
        if held_obj:
            slot=BED_DROPS[drop_idx%len(BED_DROPS)]; drop_idx+=1; do_drop(slot)
    elif vh>old_vals['Return Home']:
        old_vals['Return Home']=vh; do_home()
    elif vr>old_vals['Reset All']:
        old_vals['Reset All']=vr; do_reset()
    else:
        if not held_obj:
            for i,jname in enumerate(JOINT_NAMES):
                jp=bb.readDebugParameter(jname)
                bb.setJointMotorControl(robot,i,targetPosition=jp); cur_j[i]=jp
        else: snap_to_ee(held_obj)
    time.sleep(0.05)
