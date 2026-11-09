from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math
import numpy as np
from .geometry import normalize


@dataclass
class QueryResult:
    phi: float
    nearest_point: np.ndarray
    nearest_normal: np.ndarray
    extended_normal: np.ndarray


class SurfaceBase:
    """Minimal surface interface used by SPGA.

    The SPGA pipeline only assumes a 2-D task parameterization (x,z):
      point(x,z)              -> R^3 surface point
      blade_tangent_at(x,z)   -> across-pass tangent
      tangent_up_at(x,z)      -> along-pass tangent
      normal_at(x,z)          -> consistently oriented surface normal
      query_batch(...)        -> signed-distance/nearest-normal query

    This keeps Guard, LDC, cone, dual-arc and the solver surface-agnostic.
    """
    width: float
    height: float

    def point(self, x: float, z: float) -> np.ndarray:
        raise NotImplementedError

    def blade_tangent_at(self, x: float, z: float) -> np.ndarray:
        raise NotImplementedError

    def tangent_up_at(self, x: float, z: float) -> np.ndarray:
        raise NotImplementedError

    def normal_at(self, x: float, z: float) -> np.ndarray:
        raise NotImplementedError

    def normal_from_world_batch(self, points: np.ndarray) -> np.ndarray:
        """Consistent local surface normal associated with world points."""
        _,_,n,_=self.query_batch(np.atleast_2d(np.asarray(points,float)),0.03,0.01)
        return n

    def project(self, p: np.ndarray):
        P=np.asarray(p,float)
        _,q,n,_=self.query_batch(P[None,:],0.03,0.01)
        return q[0],n[0]

    def signed_distance(self,p:np.ndarray)->float:
        phi,_,_,_=self.query_batch(np.asarray(p,float)[None,:],0.03,0.01)
        return float(phi[0])

    def query(self,p:np.ndarray,sigma_n:float,gate_width:float)->QueryResult:
        phi,q,n,nb=self.query_batch(np.asarray(p,float)[None,:],sigma_n,gate_width)
        return QueryResult(float(phi[0]),q[0],n[0],nb[0])

    def build_obj(self,path:str|Path,nx=32,nz=140)->Path:
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        xs=np.linspace(-0.5*self.width,0.5*self.width,int(nx))
        zs=np.linspace(0.0,self.height,int(nz))
        verts=[self.point(float(x),float(z)) for z in zs for x in xs]
        def idx(i,j): return i*len(xs)+j+1
        faces=[]
        for i in range(len(zs)-1):
            for j in range(len(xs)-1):
                a,b,c,d=idx(i,j),idx(i,j+1),idx(i+1,j),idx(i+1,j+1)
                faces += [(a,c,b),(b,c,d)]
        with path.open('w',encoding='utf-8') as f:
            for v in verts:f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            for a,b,c in faces:f.write(f"f {a} {b} {c}\n")
        return path


def _load_obj_triangles(path:Path):
    """Tiny OBJ reader: vertices + triangular faces only (sufficient for Bunny.obj)."""
    verts=[]; faces=[]
    with Path(path).open('r',encoding='utf-8',errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                q=line.split(); verts.append([float(q[1]),float(q[2]),float(q[3])])
            elif line.startswith('f '):
                ids=[]
                for tok in line.split()[1:]:
                    ids.append(int(tok.split('/')[0])-1)
                if len(ids)==3:
                    faces.append(ids)
                elif len(ids)>3:
                    for j in range(1,len(ids)-1): faces.append([ids[0],ids[j],ids[j+1]])
    V=np.asarray(verts,float); F=np.asarray(faces,int)
    if len(V)==0 or len(F)==0: raise ValueError(f"OBJ has no triangle mesh: {path}")
    return V,F


class BunnyPatchSurface(SurfaceBase):
    """A smooth front-body chart on a Stanford-Bunny-style triangle mesh.

    SPGA itself is unchanged.  This class only provides the common SurfaceBase
    interface.  The working chart is a rectangular (x,z) parameter domain on
    the bunny torso.  Each chart sample is obtained from the front-most mesh
    intersection along +Y, after transforming the mesh to world coordinates:
        world X <- mesh X
        world Y <- mesh Z   (outward/front direction)
        world Z <- mesh Y   (up)

    The full bunny mesh is shown in the viewer, while Guard/conic queries use
    the selected painting patch.  This deliberately tests the same SPGA
    pipeline on a mesh-derived surface rather than inventing a Bunny-specific
    warping rule.
    """
    def __init__(self,cfg:dict):
        from scipy.spatial import cKDTree
        self.cfg=cfg
        bc=cfg.get('bunny',{})
        root=Path(__file__).resolve().parents[1]
        asset=Path(str(bc.get('asset_obj','assets/Bunny.obj')))
        if not asset.is_absolute(): asset=root/asset
        self.asset_path=asset
        V0,F=_load_obj_triangles(asset)
        self.faces=F
        self.scale=float(bc.get('scale',1.5))

        # Reflection/permutation is intentional: it makes mesh-Y vertical and
        # mesh-Z the front/back direction. Normals are transformed as vectors.
        xmid=0.5*(float(np.min(V0[:,0]))+float(np.max(V0[:,0])))
        self.vertices=np.column_stack([
            (V0[:,0]-xmid)*self.scale,
            V0[:,2]*self.scale,
            (V0[:,1]-float(np.min(V0[:,1])))*self.scale,
        ])
        tri0=V0[F]
        n0=np.cross(tri0[:,1]-tri0[:,0],tri0[:,2]-tri0[:,0])
        n0=n0/np.maximum(np.linalg.norm(n0,axis=1,keepdims=True),1e-12)
        self.face_normals=np.column_stack([n0[:,0],n0[:,2],n0[:,1]])
        self.face_normals=self.face_normals/np.maximum(np.linalg.norm(self.face_normals,axis=1,keepdims=True),1e-12)

        self.patch_center_x=float(bc.get('patch_center_x_m',0.0))
        self.patch_z_min=float(bc.get('patch_z_min_m',0.05))
        self.width=float(bc.get('patch_width_x_m',0.09))
        self.height=float(bc.get('patch_height_z_m',0.09))
        self.front_sign=float(bc.get('front_sign',1.0))
        self.grid_nx=int(bc.get('chart_grid_nx',41))
        self.grid_nz=int(bc.get('chart_grid_nz',61))
        self.curve_type='bunny_mesh'

        tri=self.vertices[F]
        self._tri=tri
        self._A=tri[:,0][:,[0,2]]; self._B=tri[:,1][:,[0,2]]; self._C=tri[:,2][:,[0,2]]
        self._v0=self._B-self._A; self._v1=self._C-self._A
        self._den=self._v0[:,0]*self._v1[:,1]-self._v1[:,0]*self._v0[:,1]

        # Build a regular chart once. Point/normal calls then use smooth bilinear
        # interpolation instead of jumping between raw triangle normals.
        self._xs=np.linspace(-0.5*self.width,0.5*self.width,self.grid_nx)
        self._zs=np.linspace(0.0,self.height,self.grid_nz)
        Y=np.zeros((self.grid_nz,self.grid_nx),float)
        N=np.zeros((self.grid_nz,self.grid_nx,3),float)
        fallback=0
        for iz,z in enumerate(self._zs):
            wz=self.patch_z_min+float(z)
            for ix,x in enumerate(self._xs):
                wx=self.patch_center_x+float(x)
                h=self._front_hit(wx,wz)
                if h is None:
                    fallback+=1
                    # Rare silhouette/hole fallback: nearest mesh vertex in XZ.
                    d=(self.vertices[:,0]-wx)**2+(self.vertices[:,2]-wz)**2
                    j=int(np.argmin(d)); y=float(self.vertices[j,1])
                    # Average adjacent face normals through a direct local search.
                    fi=np.where(np.any(F==j,axis=1))[0]
                    n=np.mean(self.face_normals[fi],axis=0) if len(fi) else np.array([0,1,0],float)
                    if n[1]*self.front_sign<0:n=-n
                    n=normalize(n)
                else:
                    y,n=h
                Y[iz,ix]=y; N[iz,ix]=n
        self._Y=Y; self._N=N; self._fallback_count=int(fallback)

        pts=[]; nrms=[]
        for iz,z in enumerate(self._zs):
            for ix,x in enumerate(self._xs):
                pts.append([self.patch_center_x+x,Y[iz,ix],self.patch_z_min+z])
                nrms.append(N[iz,ix])
        self._chart_points=np.asarray(pts,float)
        self._chart_normals=np.asarray(nrms,float)
        self._tree=cKDTree(self._chart_points)

    def _front_hit(self,wx:float,wz:float):
        P=np.array([float(wx),float(wz)])
        v2=P-self._A
        ok=np.abs(self._den)>1e-12
        u=np.zeros(len(self.faces)); v=np.zeros(len(self.faces))
        u[ok]=(v2[ok,0]*self._v1[ok,1]-self._v1[ok,0]*v2[ok,1])/self._den[ok]
        v[ok]=(self._v0[ok,0]*v2[ok,1]-v2[ok,0]*self._v0[ok,1])/self._den[ok]
        inside=ok&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1.0+1e-9)
        inds=np.where(inside)[0]
        if len(inds)==0:return None
        w=1.0-u[inds]-v[inds]
        ys=w*self._tri[inds,0,1]+u[inds]*self._tri[inds,1,1]+v[inds]*self._tri[inds,2,1]
        # Select the visible/front-most intersection. front_sign=-1 selects back.
        k=int(np.argmax(self.front_sign*ys)); fi=int(inds[k]); y=float(ys[k])
        n=self.face_normals[fi].copy()
        if n[1]*self.front_sign<0:n=-n
        return y,normalize(n)

    def _grid_coords(self,x,z):
        x=float(np.clip(x,-0.5*self.width,0.5*self.width)); z=float(np.clip(z,0.0,self.height))
        fx=(x+0.5*self.width)/self.width*(self.grid_nx-1)
        fz=z/self.height*(self.grid_nz-1)
        ix=min(int(math.floor(fx)),self.grid_nx-2); iz=min(int(math.floor(fz)),self.grid_nz-2)
        tx=fx-ix; tz=fz-iz
        return ix,iz,tx,tz

    def _interp_y_n(self,x,z):
        ix,iz,tx,tz=self._grid_coords(x,z)
        w00=(1-tx)*(1-tz); w10=tx*(1-tz); w01=(1-tx)*tz; w11=tx*tz
        y=w00*self._Y[iz,ix]+w10*self._Y[iz,ix+1]+w01*self._Y[iz+1,ix]+w11*self._Y[iz+1,ix+1]
        n=w00*self._N[iz,ix]+w10*self._N[iz,ix+1]+w01*self._N[iz+1,ix]+w11*self._N[iz+1,ix+1]
        n=normalize(n)
        if n[1]*self.front_sign<0:n=-n
        return float(y),n

    def point(self,x:float,z:float)->np.ndarray:
        y,_=self._interp_y_n(x,z)
        return np.array([self.patch_center_x+float(x),y,self.patch_z_min+float(z)])

    def normal_at(self,x:float,z:float)->np.ndarray:
        return self._interp_y_n(x,z)[1]

    def tangent_up_at(self,x:float,z:float)->np.ndarray:
        n=self.normal_at(x,z); ez=np.array([0.0,0.0,1.0])
        t=ez-float(np.dot(ez,n))*n
        if np.linalg.norm(t)<1e-8:
            ex=np.array([1.0,0.0,0.0]); t=np.cross(n,ex)
        t=normalize(t)
        if t[2]<0:t=-t
        return t

    def blade_tangent_at(self,x:float,z:float)->np.ndarray:
        n=self.normal_at(x,z); t=self.tangent_up_at(x,z)
        b=normalize(np.cross(n,t))
        if b[0]<0:b=-b
        return b

    def normal_from_world_batch(self,points:np.ndarray)->np.ndarray:
        P=np.atleast_2d(np.asarray(points,float))
        xs=np.clip(P[:,0]-self.patch_center_x,-0.5*self.width,0.5*self.width)
        zs=np.clip(P[:,2]-self.patch_z_min,0.0,self.height)
        return np.stack([self.normal_at(float(x),float(z)) for x,z in zip(xs,zs)],axis=0)

    def query_batch(self,points:np.ndarray,sigma_n:float,gate_width:float):
        P=np.atleast_2d(np.asarray(points,float))
        k=min(20,len(self._chart_points))
        dist,idx=self._tree.query(P,k=k)
        if k==1:
            dist=dist[:,None];idx=idx[:,None]
        q=self._chart_points[idx[:,0]]; nnear=self._chart_normals[idx[:,0]]
        phi=np.sum((P-q)*nnear,axis=1)
        sigma=max(float(sigma_n),1e-6)
        w=np.exp(-(dist*dist)/(2.0*sigma*sigma))
        ns=np.sum(w[:,:,None]*self._chart_normals[idx],axis=1)
        nbar=ns/np.maximum(np.linalg.norm(ns,axis=1,keepdims=True),1e-12)
        bad=np.linalg.norm(ns,axis=1)<1e-10; nbar[bad]=nnear[bad]
        return phi,q,nnear,nbar

    def build_obj(self,path:str|Path,nx=32,nz=140)->Path:
        # Export the FULL transformed bunny, not just the painting chart.
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('w',encoding='utf-8') as f:
            f.write('# Stanford Bunny-style mesh transformed for SPGA viewer\n')
            for v in self.vertices:f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            # Coordinate swap is a reflection; reverse winding for consistent visual normals.
            for a,b,c in self.faces:f.write(f"f {a+1} {c+1} {b+1}\n")
        return path

    def stats(self):
        n=self._chart_normals
        return {
            'surface_type':'mesh_patch','curve_type':'bunny_mesh',
            'mesh_vertices':int(len(self.vertices)),'mesh_faces':int(len(self.faces)),
            'patch_width_x_m':self.width,'patch_height_z_m':self.height,
            'patch_z_min_m':self.patch_z_min,'mesh_scale':self.scale,
            'chart_fallback_samples':self._fallback_count,
            'min_patch_normal_y':float(np.min(n[:,1])),'max_patch_normal_y':float(np.max(n[:,1])),
        }



class BunnyPathSurface(BunnyPatchSurface):
    """Narrow task chart routed over hard Bunny regions (ears/head/back).

    This changes only the *surface parameterization*.  The nominal skill,
    Tilt, dual arc, Guard, LDC, cone and warping solver are untouched.

    Two work regions are supported:
      - ``ears_head``: left-ear tip -> head crown -> right-ear tip
      - ``ear_head_back``: left-ear tip -> head crown -> along the back

    The centerline itself is a shortest path on the Bunny triangle-mesh graph.
    A narrow ribbon is then built around that centerline and every ribbon sample
    is projected back to the full mesh.  Guard queries also use the full Bunny
    mesh, so nearby head/ear geometry can influence A0/A1 rather than only the
    rectangular torso chart.
    """
    def __init__(self,cfg:dict):
        from scipy.spatial import cKDTree
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import dijkstra

        # Reuse Bunny mesh loading/transformation from the original class.
        super().__init__(cfg)
        bc=cfg.get('bunny',{})
        self.work_region=str(bc.get('work_region','ears_head')).lower()
        self.full_query_k=int(bc.get('full_query_k',12))
        # M2 experiment: M1 remains the default without an explicit opt-in.
        self.continuous_normal_enabled=bool(cfg.get('continuous_normal_field',{}).get('enabled',False))
        self.path_physical_width=float(bc.get('path_physical_width_m',self.width))

        # Use the physically outward normals inherited from BunnyPatchSurface.
        # NOTE: v15 flipped these a second time, which made full-mesh Guard signs
        # wrong on the ear/back stress test.  The transformed face_normals are
        # already outward for the reflected world geometry.
        self._full_face_normals=np.asarray(self.face_normals,float).copy()
        tri=self.vertices[self.faces]
        self._full_tri=tri
        self._face_centers=np.mean(tri,axis=1)
        self._face_tree=cKDTree(self._face_centers)

        # Smooth, consistently oriented vertex normals for path construction and
        # nearest-surface queries.  Area weighting reduces triangle-edge jitter.
        area2=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
        self._full_vertex_normals=np.zeros_like(self.vertices)
        for fi,face in enumerate(self.faces):
            self._full_vertex_normals[face] += area2[fi]*self._full_face_normals[fi]
        self._full_vertex_normals /= np.maximum(np.linalg.norm(self._full_vertex_normals,axis=1,keepdims=True),1e-12)

        # Build sparse mesh-edge graph for an actual surface-following path.
        edges=np.vstack([self.faces[:,[0,1]],self.faces[:,[1,2]],self.faces[:,[2,0]]])
        e0=edges[:,0]; e1=edges[:,1]
        w=np.linalg.norm(self.vertices[e0]-self.vertices[e1],axis=1)
        rows=np.r_[e0,e1]; cols=np.r_[e1,e0]; data=np.r_[w,w]
        G=coo_matrix((data,(rows,cols)),shape=(len(self.vertices),len(self.vertices))).tocsr()

        V=self.vertices

        # Optional experiment-only route override: a preselected geodesic path
        # given as Bunny mesh vertex IDs.  This changes ONLY the target surface
        # chart/guide curve.  SPGA forces, ribbon, cone, contact semantics, and
        # fixed dual-arc length are unchanged.
        custom_ids = bc.get('custom_path_vertex_ids', None)
        if custom_ids is not None:
            custom_ids = np.asarray(custom_ids, int).reshape(-1)
            if len(custom_ids) < 2:
                raise ValueError('bunny.custom_path_vertex_ids must contain at least two mesh vertices')
            if np.any(custom_ids < 0) or np.any(custom_ids >= len(V)):
                raise ValueError('bunny.custom_path_vertex_ids contains an invalid vertex index')
            self.work_region = 'custom_whole_bunny_geodesic'
            self._anchor_vertex_ids = [int(custom_ids[0]), int(custom_ids[-1])]
            self._custom_path_vertex_ids = custom_ids.copy()
            self._build_custom_geodesic_route(custom_ids)
            return

        # v16 hard-region test: use a 2-D XZ route on one visible mesh side
        # instead of an unconstrained geodesic that may jump from one thin-ear
        # sheet to the other.  The route starts near the left-ear tip, descends
        # through the head, then continues over the back and down the rump.
        if self.work_region in {'left_ear_head_back_down','ear_head_back_down','ear_back_down'}:
            self._build_side_projected_route(cfg)
            return

        # Robust anchors for this fixed Stanford Bunny geometry.
        ids=np.arange(len(V))
        m=(V[:,0] < -0.055) & (V[:,2] > 0.17)
        left_ear=int(ids[m][np.argmax(V[m,2])])
        m=(V[:,0] > -0.04) & (V[:,2] > 0.17)
        right_ear=int(ids[m][np.argmax(V[m,2])])
        head=int(np.argmin((V[:,0]+0.045)**2 + (V[:,2]-0.165)**2))
        back1=int(np.argmin((V[:,0]-0.015)**2 + (V[:,2]-0.145)**2))
        back2=int(np.argmin((V[:,0]-0.050)**2 + (V[:,2]-0.110)**2))

        if self.work_region in {'ears_head','ear_to_ear','ears'}:
            anchors=[left_ear,head,right_ear]
        elif self.work_region in {'ear_head_back','ear_to_back','head_back'}:
            anchors=[left_ear,head,back1,back2]
        else:
            raise ValueError(f"unknown bunny work_region: {self.work_region}")
        self._anchor_vertex_ids=anchors

        def shortest_vertex_path(a,b):
            _,pred=dijkstra(G,directed=False,indices=int(a),return_predecessors=True)
            if pred[int(b)] < 0:
                raise RuntimeError(f'No Bunny mesh path between vertices {a} and {b}')
            out=[int(b)]; cur=int(b)
            while cur!=int(a):
                cur=int(pred[cur]); out.append(cur)
            return out[::-1]

        path_ids=[]
        for a,b in zip(anchors[:-1],anchors[1:]):
            seg=shortest_vertex_path(a,b)
            if path_ids: seg=seg[1:]
            path_ids.extend(seg)
        raw=V[np.asarray(path_ids,int)]

        # Arc-length resample the centerline densely and smoothly enough for the
        # existing trajectory samples.  Do not smooth off the surface; projection
        # after interpolation keeps the line on the Bunny.
        d=np.linalg.norm(np.diff(raw,axis=0),axis=1)
        s=np.r_[0.0,np.cumsum(d)]
        if s[-1] < 1e-9: raise RuntimeError('degenerate Bunny path')
        ns=max(200,self.grid_nz*4)
        ss=np.linspace(0.0,s[-1],ns)
        ctr=np.column_stack([np.interp(ss,s,raw[:,j]) for j in range(3)])
        ctr,nctr=self._project_full_batch(ctr)
        self._centerline=ctr
        self._centerline_normals=nctr
        self._centerline_s=np.linspace(0.0,1.0,len(ctr))

        # Replace the old rectangular front-torso chart by a ribbon chart around
        # the hard-region centerline.  Keep the SAME width/height task domain so
        # the nominal skill and all downstream code stay unchanged.
        xs=np.linspace(-0.5*self.width,0.5*self.width,self.grid_nx)
        zs=np.linspace(0.0,self.height,self.grid_nz)
        Q=np.zeros((self.grid_nz,self.grid_nx,3),float)
        N=np.zeros_like(Q)
        prev_side=None
        for iz,z in enumerate(zs):
            # Contact runs H -> 0.  Map H to path start and 0 to path end.
            scontact=1.0-float(z)/max(self.height,1e-12)
            u=scontact*(len(ctr)-1)
            j=min(int(np.floor(u)),len(ctr)-2); a=u-j
            c=(1-a)*ctr[j]+a*ctr[j+1]
            n=normalize((1-a)*nctr[j]+a*nctr[j+1])
            jm=max(0,j-1); jp=min(len(ctr)-1,j+2)
            tmove=normalize(ctr[jp]-ctr[jm])       # start -> end contact motion
            side=normalize(np.cross(n,tmove))
            if prev_side is not None and np.dot(side,prev_side)<0: side=-side
            prev_side=side.copy()
            # Task-x domain remains unchanged, but the physical working ribbon
            # can be narrower on the ears/head.  This is a surface-chart choice,
            # not a change to the nominal skill or SPGA method.
            scale_x=self.path_physical_width/max(self.width,1e-12)
            guesses=np.asarray([c+float(x)*scale_x*side for x in xs],float)
            q,nq=self._project_full_batch(guesses)
            Q[iz]=q; N[iz]=nq
        self._Q=Q; self._N=N
        self._xs=xs; self._zs=zs
        self._chart_points=Q.reshape(-1,3)
        self._chart_normals=N.reshape(-1,3)
        self._tree=cKDTree(self._chart_points)
        self._fallback_count=0

    def _build_custom_geodesic_route(self, vertex_ids):
        """Install an arbitrary whole-Bunny geodesic as the task guide.

        ``vertex_ids`` are ordered vertices along a connected mesh-edge path.
        The guide is arc-length resampled, projected back to the full Bunny,
        and expanded into the same narrow ribbon chart expected by the existing
        Tilt/SPGA pipeline.  No trajectory sample is individually projected.
        """
        from scipy.spatial import cKDTree

        ids=np.asarray(vertex_ids,int).reshape(-1)
        raw=self.vertices[ids]
        d=np.linalg.norm(np.diff(raw,axis=0),axis=1)
        s=np.r_[0.0,np.cumsum(d)]
        if s[-1] < 1e-9:
            raise RuntimeError('degenerate custom Bunny geodesic')

        # Dense arc-length resampling preserves the complete selected geodesic.
        # About 1 mm spacing is enough for guide interpolation while keeping the
        # chart small.  Projection places interpolated edge samples back on mesh.
        ns=max(240, int(np.ceil(float(s[-1])/0.001))+1, self.grid_nz*4)
        ns=min(ns, 900)
        ss=np.linspace(0.0,float(s[-1]),ns)
        ctr=np.column_stack([np.interp(ss,s,raw[:,j]) for j in range(3)])
        ctr,nctr=self._project_full_batch(ctr)

        # Keep outward normals but smooth their direction slightly along the
        # polyline to avoid raw triangle jitter in Tilt/cone construction.
        niter=max(0,int(self.cfg.get('bunny',{}).get('custom_path_normal_smooth_iterations',4)))
        Ns=nctr.copy()
        for _ in range(niter):
            N2=Ns.copy()
            for i in range(len(Ns)):
                acc=3.0*Ns[i]
                for j in (max(0,i-1), min(len(Ns)-1,i+1)):
                    v=Ns[j].copy()
                    if float(np.dot(v,Ns[i])) < 0.0:
                        v=-v
                    acc+=v
                N2[i]=normalize(acc)
            Ns=N2
        nctr=Ns

        self._centerline=ctr
        self._centerline_normals=nctr
        self._centerline_s=np.linspace(0.0,1.0,len(ctr))
        self._custom_geodesic_length_m=float(s[-1])
        self._route_waypoints_xz=np.empty((0,2),float)
        self._route_side='custom_geodesic'

        # Build the same narrow task ribbon around this centerline.  The chart
        # coordinates remain the standard SPGA task domain; only their mapping
        # to the Bunny changes.
        xs=np.linspace(-0.5*self.width,0.5*self.width,self.grid_nx)
        zs=np.linspace(0.0,self.height,self.grid_nz)
        Q=np.zeros((self.grid_nz,self.grid_nx,3),float)
        N=np.zeros_like(Q)
        prev_side=None
        scale_x=self.path_physical_width/max(self.width,1e-12)
        fallback=0
        for iz,z in enumerate(zs):
            # sample_guide_curve executes z: H -> 0, therefore guide start -> end
            scontact=1.0-float(z)/max(self.height,1e-12)
            u=scontact*(len(ctr)-1)
            j=min(int(np.floor(u)),len(ctr)-2); a=u-j
            c=(1-a)*ctr[j]+a*ctr[j+1]
            n=normalize((1-a)*nctr[j]+a*nctr[j+1])
            jm=max(0,j-2); jp=min(len(ctr)-1,j+3)
            tmove=normalize(ctr[jp]-ctr[jm])
            side=normalize(np.cross(n,tmove))
            if prev_side is not None and float(np.dot(side,prev_side)) < 0.0:
                side=-side
            prev_side=side.copy()
            guesses=np.asarray([c+float(x)*scale_x*side for x in xs],float)
            q,nq=self._project_full_batch(guesses)
            # Keep each width-row normal oriented consistently with center normal.
            flip=np.einsum('ij,j->i',nq,n) < 0.0
            nq[flip]*=-1.0
            Q[iz]=q; N[iz]=nq

        self._Q=Q; self._N=N
        self._xs=xs; self._zs=zs
        self._chart_points=Q.reshape(-1,3)
        self._chart_normals=N.reshape(-1,3)
        self._tree=cKDTree(self._chart_points)
        self._fallback_count=int(fallback)

    def _ray_hits_smooth(self,wx:float,wz:float):
        """All world-Y ray intersections at (x,z), with smooth outward normals."""
        P2=np.array([float(wx),float(wz)],float)
        v2=P2-self._A
        ok=np.abs(self._den)>1e-12
        u=np.zeros(len(self.faces)); v=np.zeros(len(self.faces))
        u[ok]=(v2[ok,0]*self._v1[ok,1]-self._v1[ok,0]*v2[ok,1])/self._den[ok]
        v[ok]=(self._v0[ok,0]*v2[ok,1]-v2[ok,0]*self._v0[ok,1])/self._den[ok]
        inside=ok&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1.0+1e-9)
        inds=np.where(inside)[0]
        out=[]
        for fi0 in inds:
            fi=int(fi0); ww=1.0-u[fi]-v[fi]
            y=float(ww*self._tri[fi,0,1]+u[fi]*self._tri[fi,1,1]+v[fi]*self._tri[fi,2,1])
            bary=np.array([ww,u[fi],v[fi]],float)
            face=self.faces[fi]
            n=np.sum(bary[:,None]*self._full_vertex_normals[face],axis=0)
            n=normalize(n)
            if float(np.dot(n,self._full_face_normals[fi]))<0.0: n=-n
            q=np.array([float(wx),y,float(wz)],float)
            out.append((q,n,fi))
        return out

    def _side_hit_smooth(self,wx:float,wz:float,side_sign:float=-1.0):
        """Select front-most (+1) or back-most (-1) smooth ray hit."""
        hits=self._ray_hits_smooth(wx,wz)
        if not hits:return None
        return (max(hits,key=lambda h:h[0][1]) if side_sign>=0 else min(hits,key=lambda h:h[0][1]))[:2]

    @staticmethod
    def _continuous_hit(hits,ref_q,ref_n,y_weight=1.0,normal_weight=0.015):
        """Choose the ray hit that stays on the same local mesh sheet."""
        if not hits:return None
        ref_q=np.asarray(ref_q,float); ref_n=normalize(ref_n)
        def cost(h):
            q,n,_=h
            return y_weight*abs(float(q[1]-ref_q[1])) + normal_weight*(1.0-float(np.clip(np.dot(n,ref_n),-1.0,1.0)))
        q,n,_=min(hits,key=cost)
        return q,n

    def _build_side_projected_route(self,cfg:dict):
        """Build a continuous narrow ribbon from ear -> head -> back -> lower back.

        The XZ guide only says *where* the task should travel.  Actual points are
        attached to one continuous Bunny mesh sheet by tracking the previous
        ray-intersection and normal.  This avoids the v15 thin-ear front/back
        sheet switches while leaving every SPGA equation unchanged.
        """
        from scipy.interpolate import PchipInterpolator
        from scipy.spatial import cKDTree
        bc=cfg.get('bunny',{})
        side_name=str(bc.get('route_side','back')).lower()
        side_sign=-1.0 if side_name in {'back','rear','negative_y','-y'} else 1.0
        route=np.asarray(bc.get('route_xz_m',[
            [-0.079,0.202],[-0.077,0.192],[-0.073,0.179],[-0.069,0.165],
            [-0.055,0.150],[-0.036,0.138],[-0.012,0.126],[0.015,0.112],
            [0.041,0.093],[0.061,0.073],[0.073,0.054]
        ]),float)
        if route.ndim!=2 or route.shape[1]!=2 or len(route)<3:
            raise ValueError('bunny.route_xz_m must be an Nx2 list of [x,z] waypoints')
        dd=np.linalg.norm(np.diff(route,axis=0),axis=1)
        ss=np.r_[0.0,np.cumsum(dd)]
        if ss[-1]<1e-8: raise RuntimeError('degenerate Bunny projected route')
        ss/=ss[-1]
        ns=max(int(bc.get('route_samples',320)),self.grid_nz*5)
        st=np.linspace(0.0,1.0,ns)
        px=PchipInterpolator(ss,route[:,0])(st)
        pz=PchipInterpolator(ss,route[:,1])(st)

        # Track ONE mesh sheet continuously.  Only the first sample uses the
        # requested front/back side selector; subsequent samples use continuity.
        ctr=[]; nctr=[]; prev_q=None; prev_n=None
        for k,(x,z) in enumerate(zip(px,pz)):
            hits=self._ray_hits_smooth(float(x),float(z))
            if not hits:
                raise RuntimeError(f'Bunny route left the visible mesh near x={x:.4f}, z={z:.4f}')
            if k==0:
                q,n,_=(max(hits,key=lambda h:h[0][1]) if side_sign>=0 else min(hits,key=lambda h:h[0][1]))
            else:
                chosen=self._continuous_hit(hits,prev_q,prev_n)
                q,n=chosen
            # Extra sign continuity is legitimate for interpolated vertex normals:
            # it chooses the equivalent local normal representation, not another surface.
            if prev_n is not None and float(np.dot(n,prev_n))<0.0:n=-n
            ctr.append(q); nctr.append(n); prev_q=q; prev_n=n
        ctr=np.asarray(ctr,float); nctr=np.asarray(nctr,float)
        self._centerline=ctr; self._centerline_normals=nctr
        self._centerline_s=np.linspace(0.0,1.0,len(ctr))
        self._anchor_vertex_ids=[]
        self._route_waypoints_xz=route.copy(); self._route_side=side_name

        xs=np.linspace(-0.5*self.width,0.5*self.width,self.grid_nx)
        zs=np.linspace(0.0,self.height,self.grid_nz)
        Q=np.zeros((self.grid_nz,self.grid_nx,3),float); N=np.zeros_like(Q)
        fallback=0; scale_x=self.path_physical_width/max(self.width,1e-12)
        for iz,zchart in enumerate(zs):
            scontact=1.0-float(zchart)/max(self.height,1e-12)
            u=scontact*(len(ctr)-1)
            j=min(int(np.floor(u)),len(ctr)-2); a=u-j
            c=(1-a)*ctr[j]+a*ctr[j+1]
            nc=normalize((1-a)*nctr[j]+a*nctr[j+1])
            jm=max(0,j-2); jp=min(len(ctr)-1,j+3)
            d2=np.array([ctr[jp,0]-ctr[jm,0],ctr[jp,2]-ctr[jm,2]],float)
            d2=d2/max(float(np.linalg.norm(d2)),1e-12)
            side2=np.array([-d2[1],d2[0]],float)
            # Build width outward from center so neighboring samples also stay on
            # the same mesh sheet as the route centerline.
            order=np.argsort(np.abs(xs))
            row_q=[None]*len(xs); row_n=[None]*len(xs)
            for ix in order:
                off=float(xs[ix])*scale_x
                hit=None
                for shrink in (1.0,0.85,0.70,0.50,0.25,0.0):
                    wx=c[0]+shrink*off*side2[0]; wz=c[2]+shrink*off*side2[1]
                    hits=self._ray_hits_smooth(wx,wz)
                    chosen=self._continuous_hit(hits,c,nc) if hits else None
                    if chosen is not None:
                        q,n=chosen
                        if float(np.dot(n,nc))<0.0:n=-n
                        hit=(q,n)
                        if shrink<0.999:fallback+=1
                        break
                if hit is None: raise RuntimeError('failed to construct continuous Bunny route ribbon')
                row_q[ix],row_n[ix]=hit
            Q[iz]=np.asarray(row_q,float); N[iz]=np.asarray(row_n,float)
        # Build a smooth task-frame field on the ribbon.  The Bunny mesh is
        # coarse around the ears; raw per-hit normals/tangents create visible
        # frame jitter even when positions stay on one sheet.  Smoothing here is
        # purely a surface-representation step (analogous to smooth vertex normals).
        niter=max(0,int(bc.get('normal_smooth_iterations',6)))
        Ns=N.copy()
        for iz in range(1,self.grid_nz):
            for ix in range(self.grid_nx):
                if float(np.dot(Ns[iz,ix],Ns[iz-1,ix]))<0.0: Ns[iz,ix]*=-1.0
        for _ in range(niter):
            N2=Ns.copy()
            for iz in range(self.grid_nz):
                for ix in range(self.grid_nx):
                    ref=Ns[iz,ix]; acc=3.0*ref
                    for di,dj,wgt in ((-1,0,1.0),(1,0,1.0),(0,-1,0.35),(0,1,0.35)):
                        ii=min(max(iz+di,0),self.grid_nz-1); jj=min(max(ix+dj,0),self.grid_nx-1)
                        vv=Ns[ii,jj].copy()
                        if float(np.dot(vv,ref))<0.0: vv=-vv
                        acc+=wgt*vv
                    N2[iz,ix]=normalize(acc)
            Ns=N2

        # The along-path tangent is transported from the centerline rather than
        # re-estimated independently on every narrow ribbon column.
        C=Q[:,self.grid_nx//2,:]
        Trow=np.zeros((self.grid_nz,3),float)
        for iz in range(self.grid_nz):
            im=max(0,iz-1); ip=min(self.grid_nz-1,iz+1)
            tt=C[ip]-C[im]; nn=Ns[iz,self.grid_nx//2]
            tt=tt-nn*float(np.dot(tt,nn))
            Trow[iz]=normalize(tt)
        titer=max(0,int(bc.get('tangent_smooth_iterations',5)))
        for _ in range(titer):
            T2=Trow.copy()
            for iz in range(self.grid_nz):
                ref=Trow[iz]; acc=3.0*ref
                for ii in (max(0,iz-1),min(self.grid_nz-1,iz+1)):
                    vv=Trow[ii].copy()
                    if float(np.dot(vv,ref))<0.0: vv=-vv
                    acc+=vv
                T2[iz]=normalize(acc)
            Trow=T2
        T=np.zeros_like(Q)
        for iz in range(self.grid_nz):
            for ix in range(self.grid_nx):
                nn=Ns[iz,ix]; tt=Trow[iz]-nn*float(np.dot(Trow[iz],nn))
                T[iz,ix]=normalize(tt)

        self._Q=Q; self._N=Ns; self._T=T; self._xs=xs; self._zs=zs
        self._chart_points=Q.reshape(-1,3); self._chart_normals=Ns.reshape(-1,3)
        self._tree=cKDTree(self._chart_points); self._fallback_count=int(fallback)

    @staticmethod
    def _closest_point_tri_batch(P,A,B,C):
        """Closest point on each triangle, vectorized over matching rows."""
        AB=B-A; AC=C-A; AP=P-A
        d1=np.einsum('ij,ij->i',AB,AP); d2=np.einsum('ij,ij->i',AC,AP)
        Q=np.empty_like(P); done=np.zeros(len(P),bool)
        m=(d1<=0)&(d2<=0); Q[m]=A[m]; done|=m

        BP=P-B; d3=np.einsum('ij,ij->i',AB,BP); d4=np.einsum('ij,ij->i',AC,BP)
        m=(~done)&(d3>=0)&(d4<=d3); Q[m]=B[m]; done|=m
        vc=d1*d4-d3*d2
        m=(~done)&(vc<=0)&(d1>=0)&(d3<=0)
        v=np.zeros(len(P)); den=np.maximum(d1-d3,1e-12); v[m]=d1[m]/den[m]
        Q[m]=A[m]+v[m,None]*AB[m]; done|=m

        CP=P-C; d5=np.einsum('ij,ij->i',AB,CP); d6=np.einsum('ij,ij->i',AC,CP)
        m=(~done)&(d6>=0)&(d5<=d6); Q[m]=C[m]; done|=m
        vb=d5*d2-d1*d6
        m=(~done)&(vb<=0)&(d2>=0)&(d6<=0)
        w=np.zeros(len(P)); den=np.maximum(d2-d6,1e-12); w[m]=d2[m]/den[m]
        Q[m]=A[m]+w[m,None]*AC[m]; done|=m

        va=d3*d6-d5*d4
        m=(~done)&(va<=0)&((d4-d3)>=0)&((d5-d6)>=0)
        den=np.maximum((d4-d3)+(d5-d6),1e-12); w2=np.zeros(len(P)); w2[m]=(d4[m]-d3[m])/den[m]
        Q[m]=B[m]+w2[m,None]*(C[m]-B[m]); done|=m

        m=~done
        denom=np.maximum(va+vb+vc,1e-12)
        vv=np.zeros(len(P)); ww=np.zeros(len(P)); vv[m]=vb[m]/denom[m]; ww[m]=vc[m]/denom[m]
        Q[m]=A[m]+AB[m]*vv[m,None]+AC[m]*ww[m,None]
        return Q

    def _project_full_batch(self,P,return_face=False):
        P=np.atleast_2d(np.asarray(P,float))
        k=min(max(1,self.full_query_k),len(self._full_tri))
        _,cand=self._face_tree.query(P,k=k)
        if k==1:cand=cand[:,None]
        n=len(P); kk=cand.shape[1]
        Pf=np.repeat(P,kk,axis=0); fi=cand.reshape(-1)
        T=self._full_tri[fi]
        Qf=self._closest_point_tri_batch(Pf,T[:,0],T[:,1],T[:,2])
        D=np.sum((Pf-Qf)**2,axis=1).reshape(n,kk)
        jj=np.argmin(D,axis=1); rows=np.arange(n)
        sel=cand[rows,jj]
        q=Qf.reshape(n,kk,3)[rows,jj]
        # Smooth outward normal from the selected nearest triangle's vertices.
        normals=np.mean(self._full_vertex_normals[self.faces[sel]],axis=1)
        normals=normals/np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
        fn=self._full_face_normals[sel]
        flip=np.einsum('ij,ij->i',normals,fn)<0
        normals[flip]*=-1.0
        if return_face:
            return q,normals,sel
        return q,normals

    def _continuous_normal_at_projection(self, q, selected_faces, n_nearest):
        """M2: continuous (C0 on manifold) mesh-normal extension.

        The normal sources are the three area-weighted vertex normals of the
        closest triangle. Their *barycentric* weights at the closest surface
        point vary continuously across triangle edges (shared vertices). We
        extend that blended direction along the surface normal to the tool
        point; it is *not* a separate infinite line / hard-gated column.

        This local construction has no sampling-density-dependent column gap,
        no tunable Gaussian radius, and does not allow an unrelated sheet to
        vote in the same contact. It is not guaranteed C1 or continuous at
        non-unique nearest projections / medial axis.
        """
        triangles=self._full_tri[selected_faces]
        a=triangles[:,0,:]
        ab=triangles[:,1,:]-a
        ac=triangles[:,2,:]-a
        aq=q-a
        d00=np.einsum('ij,ij->i',ab,ab)
        d01=np.einsum('ij,ij->i',ab,ac)
        d11=np.einsum('ij,ij->i',ac,ac)
        d20=np.einsum('ij,ij->i',aq,ab)
        d21=np.einsum('ij,ij->i',aq,ac)
        det=d00*d11-d01*d01
        ok=np.abs(det)>1e-18
        b=np.zeros(len(q),float)
        c=np.zeros(len(q),float)
        b[ok]=(d11[ok]*d20[ok]-d01[ok]*d21[ok])/det[ok]
        c[ok]=(d00[ok]*d21[ok]-d01[ok]*d20[ok])/det[ok]
        bary=np.stack((1-b-c,b,c),axis=1)
        bary=np.maximum(bary,0.0)
        bary/=np.maximum(bary.sum(axis=1,keepdims=True),1e-12)
        vn=self._full_vertex_normals[self.faces[selected_faces]]
        n=np.einsum('ni,nij->nj',bary,vn)
        norm=np.linalg.norm(n,axis=1)
        bad=(norm<1e-9) | (~ok)
        n=n/np.maximum(norm[:,None],1e-12)
        # A blended normal must remain a valid repulsion direction relative to
        # the original outward face orientation. Fall back to M1 if degenerate.
        bad|=(np.einsum('ij,ij->i',n,n_nearest)<0.20)
        n[bad]=n_nearest[bad]
        return n

    def _interp_p_n(self,x,z):
        ix,iz,tx,tz=self._grid_coords(x,z)
        w00=(1-tx)*(1-tz); w10=tx*(1-tz); w01=(1-tx)*tz; w11=tx*tz
        p=w00*self._Q[iz,ix]+w10*self._Q[iz,ix+1]+w01*self._Q[iz+1,ix]+w11*self._Q[iz+1,ix+1]
        n=w00*self._N[iz,ix]+w10*self._N[iz,ix+1]+w01*self._N[iz+1,ix]+w11*self._N[iz+1,ix+1]
        return p,normalize(n)

    def point(self,x:float,z:float)->np.ndarray:
        return self._interp_p_n(x,z)[0]

    def normal_at(self,x:float,z:float)->np.ndarray:
        return self._interp_p_n(x,z)[1]

    def tangent_up_at(self,x:float,z:float)->np.ndarray:
        # Increasing chart-z direction. Contact executes H->0, so -tangent_up
        # is the actual forward motion direction (matching attack_tangent_sign=-1).
        if hasattr(self,'_T'):
            ix,iz,tx,tz=self._grid_coords(x,z)
            w00=(1-tx)*(1-tz); w10=tx*(1-tz); w01=(1-tx)*tz; w11=tx*tz
            t=w00*self._T[iz,ix]+w10*self._T[iz,ix+1]+w01*self._T[iz+1,ix]+w11*self._T[iz+1,ix+1]
            n=self.normal_at(x,z); t=t-n*float(np.dot(t,n))
            return normalize(t)
        dz=max(self.height/(self.grid_nz-1),1e-4)*0.5
        p0=self.point(x,max(0.0,float(z)-dz)); p1=self.point(x,min(self.height,float(z)+dz))
        n=self.normal_at(x,z); t=p1-p0; t=t-n*float(np.dot(t,n))
        return normalize(t)

    def blade_tangent_at(self,x:float,z:float)->np.ndarray:
        # Complete an orthonormal tangent frame from the path tangent + normal.
        # This is important because the attack-angle construction assumes blade,
        # along-path tangent and normal are mutually orthogonal.
        n=self.normal_at(x,z); t=self.tangent_up_at(x,z)
        b=normalize(np.cross(n,t))
        # Keep the sign consistent with increasing task-x on the ribbon chart.
        dx=max(self.width/(self.grid_nx-1),1e-4)*0.5
        p0=self.point(max(-0.5*self.width,float(x)-dx),z); p1=self.point(min(0.5*self.width,float(x)+dx),z)
        if np.dot(b,p1-p0)<0:b=-b
        return b

    def normal_from_world_batch(self,points:np.ndarray)->np.ndarray:
        # Cone/Tilt geometry should use the smooth task-chart normal, not a raw
        # nearest triangle normal.  This keeps the requested attack angle exact
        # while Guard can still query the full Bunny mesh below.
        P=np.atleast_2d(np.asarray(points,float))
        _,idx=self._tree.query(P,k=1)
        return self._chart_normals[np.asarray(idx,int)]

    def query_batch(self,points:np.ndarray,sigma_n:float,gate_width:float):
        P=np.atleast_2d(np.asarray(points,float))
        if self.continuous_normal_enabled:
            q,nnear,face_id=self._project_full_batch(P,return_face=True)
            nbar=self._continuous_normal_at_projection(q,face_id,nnear)
        else:
            q,nnear=self._project_full_batch(P)
            nbar=nnear.copy()
        delta=P-q
        d=np.linalg.norm(delta,axis=1)
        sgn=np.where(np.sum(delta*nnear,axis=1)>=0.0,1.0,-1.0)
        phi=sgn*d
        # Preserve M1 distance, activation, magnitude and solver logic exactly.
        return phi,q,nnear,nbar

    def stats(self):
        base=super().stats()
        base.update({
            'surface_type':'bunny_path_patch',
            'work_region':self.work_region,
            'anchor_vertex_ids':[int(i) for i in self._anchor_vertex_ids],
            'centerline_length_m':float(np.sum(np.linalg.norm(np.diff(self._centerline,axis=0),axis=1))),
            'full_mesh_guard_query':True,
            'continuous_normal_field_enabled':self.continuous_normal_enabled,
            'path_physical_width_m':self.path_physical_width,
            'route_side':getattr(self,'_route_side',None),
            'route_waypoints_xz_m':getattr(self,'_route_waypoints_xz',np.empty((0,2))).tolist(),
            'route_frame_smoothing':bool(hasattr(self,'_T')),
            'custom_geodesic_length_m':float(getattr(self,'_custom_geodesic_length_m',0.0)),
            'custom_path_vertex_count':int(len(getattr(self,'_custom_path_vertex_ids',[]))),
        })
        return base

def make_surface(cfg: dict) -> SurfaceBase:
    kind = str(cfg.get("curve_type", "bunny_path")).lower()
    if kind not in {"bunny_path", "bunny_head_ears", "bunny_route"}:
        raise ValueError("Final palpation package supports BunnyPathSurface only")
    return BunnyPathSurface(cfg)