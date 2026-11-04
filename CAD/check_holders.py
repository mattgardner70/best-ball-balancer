"""Fit check for the board holders: no collision with the base, screws on the pads, every board hole on
an insert, boards clear, flat backs. Run before printing:  uv run python -m CAD.check_holders"""
import numpy as np, cadquery as cq
from CAD import chassis as c
V=cq.Vector
base=c.base_plate().val()
printed={"mega":[(99.5,-40.3),(99.5,80.3),(85.0,-20.0),(85.0,2.0),(85.0,38.0),(85.0,60.0)],
         "driver":[(69.5,-51.0),(69.5,-90.0),(83.5,-51.0),(83.5,-90.0)],
         "switch":[(-97.5,39.5),(-97.5,80.5)]}
parts={"mega":c.mega_mount().val(),"driver":c.driver_mount().val(),"switch":c.switch_mount().val()}
fails=[]
def chk(cond,msg):
    print(("  PASS  " if cond else "  FAIL  ")+msg)
    if not cond: fails.append(msg)
for k,h in parts.items():
    print(f"[{k}]")
    chk(base.intersect(h).Volume()<0.01, "no collision with the base plate")
    chk(len(cq.Workplane().add(h).solids().vals())==1, "one solid piece")
    for x,y in printed[k]:
        top=c.PLATE_T+c.PAD_H
        chk(not h.isInside(V(x,y,top+c.TAB_T/2)) and not h.isInside(V(x,y,top+0.3)), f"screw hole open over pad ({x},{y})")
        chk(all(h.isInside(V(x+2.6*np.cos(a),y+2.6*np.sin(a),top+0.6)) for a in np.linspace(0,2*np.pi,12,endpoint=False)), f"tab sits solid on pad ({x},{y})")
        chk(not any(h.isInside(V(x+dx,y+dy,top-1.0)) for dx in (-4,0,4) for dy in (-4,0,4)), f"nothing reaches into pad ({x},{y})")
m=parts["mega"]; L,H=c.MEGA; face=c.MEGA_FACE_X+1.6; zb=c.PLATE_T+c.MEGA_LIFT
for y,up in c._mega_holes():
    z=zb+up
    if up>H-6.0:
        chk(not m.isInside(V(face+0.5,y,z)), f"Mega hole at y {y:.1f} z {z:.1f} (top edge, unused) clear")
        continue
    chk((not m.isInside(V(face+0.5,y,z))) and all(m.isInside(V(face+0.5,y+3*np.cos(a),z+3*np.sin(a))) for a in np.linspace(0,6.28,8,endpoint=False)), f"Mega hole at y {y:.1f} z {z:.1f} on an insert standoff")
    chk(all(not m.isInside(V(face+dx,y,z)) for dx in np.arange(0.3,c.INSERT_M3[1],0.5)) and m.isInside(V(face+c.INSERT_M3[1]+0.4,y,z)), f"  insert hole {c.INSERT_M3[1]:.0f} mm deep with a floor behind it")
board=cq.Workplane("XY").box(1.6,L,H,centered=(False,True,False)).translate((face-1.6,c.MEGA_CY,zb)).val()
chk(board.intersect(m).Volume()<0.01, "Mega board itself clear of the holder")
d=parts["driver"]
for side in (1,-1):
    fx=c.DRIVER_WALL_X+side*(1.5+c.DRIVER_STANDOFF)
    for dy in (-18.29,18.29):
        y,z=c.DRIVER_Y+dy, c.PLATE_T+c.MEGA_LIFT+24.13+21.59
        chk((not d.isInside(V(fx-side*0.5,y,z))) and all(d.isInside(V(fx-side*0.5,y+3*np.cos(a),z+3*np.sin(a))) for a in np.linspace(0,6.28,8,endpoint=False)), f"MDD3A face {side:+d} hole y {y:.1f} on an insert post")
    bx=fx if side>0 else fx-1.6
    mdd=cq.Workplane("XY").box(1.6,41.66,48.26,centered=(False,True,False)).translate((bx,c.DRIVER_Y,c.PLATE_T+c.MEGA_LIFT)).val()
    chk(mdd.intersect(d).Volume()<0.01, f"MDD3A face {side:+d} board clear of the holder")
for k,side in (("mega","max"),("switch","min")):
    h=parts[k]; bb=h.BoundingBox(); xv=bb.xmax if side=="max" else bb.xmin
    area=sum(f.Area() for f in h.Faces() if abs(f.normalAt().x)>0.999 and abs(f.Center().x-xv)<0.01)
    chk(area>500, f"{k} prints flat on its back (face {area:.0f} mm2 at x {xv:.1f})")
print("\nALL CHECKS PASS" if not fails else f"\n{len(fails)} FAILED")
raise SystemExit(1 if fails else 0)
