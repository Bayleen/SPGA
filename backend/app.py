"""HTTP bridge for the UNMODIFIED research SPGA-M3 warping solver.

The adapter converts browser scene geometry/dual-arc poses into metric units.
The solver and force field are imported from the supplied 2026-11-02 M3 suite.
This service does not simulate warping in JS or replace the force calculation.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, model_validator

from spga.dual_arc import DualArcReference
from spga.conic import build_conic_from_tilted
from spga.warping import warp_with_conic
from spga.surface import BunnyPathSurface

ROOT = Path(__file__).resolve().parent
METRES_PER_DISPLAY_UNIT = 0.05  # Explicit scene scale: 1 unit = 5 cm
MAX_POSES = 280
MAX_FACES = 5000
MAX_JOBS = 40
ALLOWED_ORIGINS = os.getenv('SPGA_CORS_ORIGINS', 'https://bayleen.github.io,http://localhost:8000,http://127.0.0.1:8000').split(',')

api = FastAPI(title='SPGA M3 Research Solver API', version='1.0.0')
api.add_middleware(CORSMiddleware, allow_origins=ALLOWED_ORIGINS, allow_methods=['GET','POST'], allow_headers=['Content-Type'])

class Point(BaseModel):
    x: float
    y: float
    z: float

class Pose(BaseModel):
    a0: Point
    a1: Point
    contact: bool = False
    passId: int = 0

class Balance(BaseModel):
    geometry: float = Field(1.0, ge=0.0, le=8.0)
    conic: float = Field(1.0, ge=0.0, le=8.0)
    tension: float = Field(1.0, ge=0.0, le=8.0)
    bending: float = Field(1.0, ge=0.0, le=8.0)

class WarpRequest(BaseModel):
    verts: list[Point] = Field(min_length=3, max_length=3000)
    faces: list[tuple[int,int,int]] = Field(min_length=1, max_length=MAX_FACES)
    poses: list[Pose] = Field(min_length=1, max_length=MAX_POSES)
    rodLength: float = Field(gt=0.01, lt=10.0)
    coneAngle: float = Field(15, ge=0.0, le=180.0)
    gains: Balance = Field(default_factory=Balance)
    maxIterations: int = Field(3000, ge=20, le=3000)

    @model_validator(mode='after')
    def validate_input(self):
        verts = np.array([_vec(p) for p in self.verts])
        P = np.array([[_vec(p.a0), _vec(p.a1)] for p in self.poses])
        if not np.isfinite(verts).all() or not np.isfinite(P).all():
            raise ValueError('All coordinates must be finite')
        if any(min(f)<0 or max(f)>=len(self.verts) or len(set(f))!=3 for f in self.faces):
            raise ValueError('Invalid triangle vertex indices')
        L = np.linalg.norm(P[:,1]-P[:,0],axis=1)
        if not np.allclose(L,self.rodLength,atol=2e-2,rtol=0.02):
            raise ValueError('Every A0-A1 segment must match rodLength')
        return self

def _vec(p):
    return [p.x, p.y, p.z]

class MeshSurface:
    """Mesh query adapter matching M3's signed-distance query semantics.

    Uses the original Bunny triangle-projection routine and face KD-tree, with
    smoothly averaged mesh vertex normals.  M3's spatial BVH/visibility and
    multi-normal force calculation are the ORIGINAL research code, unchanged.
    """
    def __init__(self, verts: np.ndarray, faces: np.ndarray):
        from scipy.spatial import cKDTree
        self.vertices = verts
        self.faces = faces
        self.tri = verts[faces]
        self.tree = cKDTree(self.tri.mean(axis=1))
        cross = np.cross(self.tri[:,1]-self.tri[:,0],self.tri[:,2]-self.tri[:,0])
        lengths = np.linalg.norm(cross,axis=1)
        if np.any(lengths < 1e-13):
            raise ValueError('Degenerate mesh triangles')
        self.face_normals = cross / lengths[:,None]
        vn = np.zeros_like(self.vertices)
        np.add.at(vn,faces[:,0],cross)
        np.add.at(vn,faces[:,1],cross)
        np.add.at(vn,faces[:,2],cross)
        self.vertex_normals = vn / np.maximum(np.linalg.norm(vn,axis=1,keepdims=True),1e-12)
        self._full_vertex_normals = self.vertex_normals
        self._full_face_normals = self.face_normals

    def _closest(self, points):
        P = np.atleast_2d(np.asarray(points,float))
        k=min(24,len(self.tri))
        _, candidates=self.tree.query(P,k=k)
        if k==1:
            candidates=np.asarray(candidates)[:,None]
        n=len(P)
        triangles=self.tri[candidates.reshape(-1)]
        qflat=BunnyPathSurface._closest_point_tri_batch(
            np.repeat(P,k,axis=0),triangles[:,0],triangles[:,1],triangles[:,2]
        )
        Q=qflat.reshape(n,k,3)
        j=np.argmin(np.sum((Q-P[:,None,:])**2,axis=2),axis=1)
        chosen=candidates[np.arange(n),j]
        q=Q[np.arange(n),j]
        # Smooth interpolated normal on nearest surface triangle.
        T=self.tri[chosen]
        a=T[:,0]; ab=T[:,1]-a;ac=T[:,2]-a;aq=q-a
        d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac)
        d11=np.einsum('ij,ij->i',ac,ac)
        d20=np.einsum('ij,ij->i',aq,ab);d21=np.einsum('ij,ij->i',aq,ac)
        den=np.maximum(d00*d11-d01*d01,1e-18)
        v=np.clip((d11*d20-d01*d21)/den,0,1)
        w=np.clip((d00*d21-d01*d20)/den,0,1)
        bary=np.column_stack((np.maximum(0,1-v-w),v,w))
        bary/=np.maximum(bary.sum(axis=1,keepdims=True),1e-12)
        nbar=np.einsum('ni,nij->nj',bary,self.vertex_normals[self.faces[chosen]])
        nbar/=np.maximum(np.linalg.norm(nbar,axis=1,keepdims=True),1e-12)
        face_n=self.face_normals[chosen]
        flip=np.sum(nbar*face_n,axis=1)<0
        nbar[flip]*=-1
        return q,nbar

    def normal_from_world_batch(self,points):
        return self._closest(points)[1]

    def query_batch(self,points,sigma_n,gate_width):
        P=np.atleast_2d(np.asarray(points,float))
        q,n=self._closest(P)
        d=P-q
        # M3 research solver uses signed phi as a diagnostic (the force is
        # computed separately from visible quadrature sources).
        phi=np.linalg.norm(d,axis=1)*np.where(np.sum(d*n,axis=1)>=0,1,-1)
        return phi,q,n,n


def _config(req: WarpRequest):
    cfg=json.loads((ROOT/'configs/milestone_m3.json').read_text())
    cfg['nominal']['tool_axis_length_m']=req.rodLength*METRES_PER_DISPLAY_UNIT
    cfg['conic']['half_angle_deg']=req.coneAngle
    # Force field and structural regularizers keep their original equations;
    # UI gains multiply the corresponding research coefficients.
    b=req.gains
    mc=cfg['geometry_field']['multi_normal']
    mc['safety_gain']*=b.geometry
    mc['penetration_gain']*=b.geometry
    cfg['conic']['k_in']*=b.conic
    cfg['conic']['k_out']*=b.conic
    cfg['ribbon']['tension_gain']*=b.tension
    cfg['ribbon']['bending_gain']*=b.bending
    cfg['solver']['max_iterations']=req.maxIterations
    return cfg


def solve(req: WarpRequest) -> dict:
    """Actual M3: build_conic_from_tilted -> warp_with_conic (unmodified)."""
    t0=time.monotonic()
    s=METRES_PER_DISPLAY_UNIT
    V=np.array([_vec(x) for x in req.verts],dtype=float)*s
    F=np.asarray(req.faces,dtype=int)
    a0=np.array([_vec(x.a0) for x in req.poses],dtype=float)*s
    a1=np.array([_vec(x.a1) for x in req.poses],dtype=float)*s
    fixed=np.array([x.contact for x in req.poses],bool)
    pid=np.array([x.passId for x in req.poses],int)
    tool_length=req.rodLength*s
    ref=DualArcReference(a0.copy(),a1.copy(),fixed.copy(),(a1-a0)/tool_length,tool_length)
    # Rotations used only to rebuild output frames (R_warp); select a valid
    # orthonormal reference frame with tool axis equal to its second column.
    R=np.empty((len(a0),3,3))
    for i,u in enumerate(ref.tool_axis):
        seed=np.array([0.,0.,1.]) if abs(u[2])<0.85 else np.array([1.,0.,0.])
        x=np.cross(u,seed);x/=max(np.linalg.norm(x),1e-12)
        z=np.cross(x,u)
        R[i]=np.column_stack((x,u,z))
    ref_traj=SimpleNamespace(trajectory=SimpleNamespace(
        positions=a0,rotations=R,contact_mask=fixed,pass_ids=pid),
        contact_anchors=a0.copy())
    surf=MeshSurface(V,F)
    cfg=_config(req)
    conic=build_conic_from_tilted(ref,surf,cfg)
    result=warp_with_conic(ref,ref_traj,conic,surf,cfg)
    out0=result.arc0/s;out1=result.arc1/s
    d=result.diagnostics
    updated=[]
    for i in range(len(a0)):
        d1=(out1[i]-out0[i])
        d1/=max(np.linalg.norm(d1),1e-12)
        angle=math.degrees(math.acos(float(np.clip(d1@ref.tool_axis[i],-1,1))))
        updated.append(dict(a0=dict(zip('xyz',out0[i].tolist())),
                            a1=dict(zip('xyz',out1[i].tolist())),
                            axis=dict(zip('xyz',d1.tolist())),
                            contact=bool(fixed[i]),passId=int(pid[i]),deviationDeg=angle))
    return {
        'solver':'SPGA-M3-20261102-original-python',
        'units':{'scene':'display_unit','metres_per_display_unit':s},
        'warped':updated,
        'diagnostics':{
            'converged':bool(d['converged']),
            'mode':str(d.get('convergence_mode')),
            'iterations':int(d['iterations']),
            'force_residual':float(d.get('final_allowed_force_residual',float('nan'))),
            'max_anchor_error_m':float(d.get('fixed_A0_max_error_m',0.0)),
            'max_pre_retraction_length_error_m':float(d.get('max_pre_retraction_length_error_m',0.0)),
            'final_min_phi_full_tool_m':float(d.get('final_min_phi_full_tool_m',float('nan'))),
            'elapsed_seconds':round(time.monotonic()-t0,3),
            'geometry_field':'M3 multi_normal: visibility + BVH + exponential source weighting',
        }
    }

_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='spga-m3')
_jobs=OrderedDict()
_lock=threading.Lock()

def _work(job_id,req):
    try:
        answer=solve(req)
        with _lock:_jobs[job_id]={'status':'complete','result':answer,'time':time.time()}
    except Exception as exc:
        with _lock:_jobs[job_id]={'status':'error','error':f'{type(exc).__name__}: {exc}','time':time.time()}

@api.get('/api/health')
def health():
    return {'ok':True,'solver':'SPGA-M3 original Python','scale_m_per_unit':METRES_PER_DISPLAY_UNIT}

@api.post('/api/warp')
def start_warp(req:WarpRequest):
    with _lock:
        running=sum(1 for j in _jobs.values() if j['status'] in ('queued','running'))
        if running>=3:
            raise HTTPException(status_code=429,detail='Solver busy; retry after current tasks complete')
        while len(_jobs)>=MAX_JOBS:_jobs.popitem(last=False)
        job_id=uuid.uuid4().hex
        _jobs[job_id]={'status':'running','time':time.time()}
    _executor.submit(_work,job_id,req)
    return {'jobId':job_id,'status':'running'}

@api.get('/api/warp/{job_id}')
def get_warp(job_id:str):
    with _lock:
        job=_jobs.get(job_id)
        if job is None:raise HTTPException(status_code=404,detail='Unknown or expired job')
        return {'jobId':job_id,**job}