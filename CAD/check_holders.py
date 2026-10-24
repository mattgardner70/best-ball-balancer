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
m=parts["mega"]; L=101.6; x0,y0=c.MEGA_AT; face=x0+10-6
for hx,hy in [(13.97,2.54),(66.04,7.62),(66.04,35.56),(96.52,2.54)]:
    y,z=y0+L/2-hx, c.PLATE_T+c.MEGA_LIFT+hy
    chk((not m.isInside(V(face+0.5,y,z))) and all(m.isInside(V(face+0.5,y+3*np.cos(a),z+3*np.sin(a))) for a in np.linspace(0,6.28,8,endpoint=False)), f"Mega hole {(hx,hy)} on an insert standoff")
board=cq.Workplane("XY").box(1.6,L,53.34,centered=(False,True,False)).translate((face-1.6,y0,c.PLATE_T+c.MEGA_LIFT)).val()
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
