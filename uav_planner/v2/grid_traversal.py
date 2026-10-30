"""Exact plane crossings of analytic motion in a regular local metric grid.

Cells are half-open on their upper faces. A face/corner crossing has no dwell
in adjacent cells; simultaneous crossings therefore produce one transition.
Explicit cell bounds split traversal too and their codes override regular codes.
"""
from __future__ import annotations

import math
import numpy as np


def grid_frame(model):
    if model.grid:
        g = model.grid
        return np.array(g.origin), np.array(g.resolution), list(g.shape)
    b = model.bounds
    origin = np.array([b.x_min, b.y_min, b.z_min])
    resolution = np.array(model.resolution)
    shape = [math.ceil((hi-lo)/step) for lo,hi,step in zip(origin, [b.x_max,b.y_max,b.z_max], resolution)]
    return origin, resolution, shape


def grid_location(model, point):
    origin, resolution, _ = grid_frame(model)
    q = (point-origin)/resolution
    nearest = np.rint(q)
    index = np.floor(np.where(np.abs(q-nearest)<1e-8,nearest,q)).astype(int).tolist()
    code = model.grid.codes.get(','.join(map(str,index))) if model.grid else None
    if model.grid:
        for cell in model.grid.cells:
            b = cell.bounds
            if cell.code is not None and all(lo<=v<hi for v,lo,hi in zip(point,[b.x_min,b.y_min,b.z_min],[b.x_max,b.y_max,b.z_max])):
                code = cell.code
                break
    return index, code


def traversed_grids(model, motion):
    origin, spacing, shape = grid_frame(model)
    entries = []
    for p, start in zip(motion.primitives, motion.starts):
        fractions = [0.,1.]
        # Arc extrema can exceed endpoint bounds: use the complete circle.
        low, high = np.minimum(p.start,p.end), np.maximum(p.start,p.end)
        if p.kind == 'arc':
            low[:2], high[:2] = p.center[:2]-p.radius, p.center[:2]+p.radius
        for axis in range(3):
            begin = math.ceil((low[axis]-origin[axis])/spacing[axis])
            end = math.floor((high[axis]-origin[axis])/spacing[axis])
            planes = [origin[axis]+k*spacing[axis] for k in range(begin,end+1)]
            if model.grid:
                for cell in model.grid.cells:
                    planes.extend([getattr(cell.bounds,'xyz'[axis]+'_min'),getattr(cell.bounds,'xyz'[axis]+'_max')])
            for plane in planes:
                if p.kind == 'arc' and axis<2:
                    q = (plane-p.center[axis])/p.radius
                    if abs(q)>1+1e-10:
                        continue
                    q = max(-1.,min(1.,q))
                    angles = [math.acos(q),-math.acos(q)] if axis==0 else [math.asin(q),math.pi-math.asin(q)]
                    a0 = math.atan2(p.start[1]-p.center[1],p.start[0]-p.center[0])
                    for a in angles:
                        lower,upper = sorted([a0,a0+p.sweep])
                        for winding in range(math.ceil((lower-a)/(2*math.pi)),math.floor((upper-a)/(2*math.pi))+1):
                            if abs(p.sweep)>1e-12:
                                fractions.append((a+winding*2*math.pi-a0)/p.sweep)
                elif abs(p.end[axis]-p.start[axis])>1e-12:
                    fractions.append((plane-p.start[axis])/(p.end[axis]-p.start[axis]))
        fractions = sorted(set(round(max(0.,min(1.,f)),13) for f in fractions if -1e-10<=f<=1+1e-10))
        for a,b in zip(fractions,fractions[1:]):
            if b-a<1e-12:
                continue
            index,code = grid_location(model,p.position(p.length*(a+b)/2))
            ta = start+p.time_at(p.length*a) if p.length else start
            tb = start+p.time_at(p.length*b) if p.length else start+p.duration
            if entries and entries[-1]['grid_indices']==index and entries[-1]['grid_code']==code and abs(entries[-1]['exit_time_s']-ta)<1e-7:
                entries[-1]['exit_time_s'] = tb
            else:
                entries.append(dict(grid_indices=index,grid_code=code,entry_time_s=ta,exit_time_s=tb))
    if not entries and len(motion.points):
        index,code=grid_location(model,motion.points[0])
        entries.append(dict(grid_indices=index,grid_code=code,entry_time_s=float(motion.times[0]),exit_time_s=float(motion.times[-1])))
    context = dict(crs=model.grid.crs if model.grid else 'LOCAL_AEQD',origin=origin.tolist(),resolution=spacing.tolist(),shape=shape,
                   level=model.grid.level if model.grid else None,source='scene.grid' if model.grid else 'scene.bounds',keypoint_indices_source='scene.bounds/resolution',boundary_convention='half-open; simultaneous face/corner crossings have no dwell')
    if model.grid:
        context['cells'] = [c.model_dump(mode='json') for c in model.grid.cells]
    return entries,context
