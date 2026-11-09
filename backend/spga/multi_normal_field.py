"""Experimental M3 multi-source normal influence field.

This deliberately is NOT M2. Surface triangles contribute smooth outward
normal sources, combined with area quadrature and positive, decaying kernels.
The spatial KDTree only removes negligible terms, not a fixed-k nearest list.
Shadow rays suppress occluded sheets, but open/free-space facing surfaces
can simultaneously contribute (including opposing cave walls).

Lengths are metres. The force is a pseudo-force, NOT physical Newtons.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


def smoothstep01(x):
    x=np.clip(np.asarray(x, float),0.0,1.0)
    return x*x*(3.0-2.0*x)


def exp_soft_force(h, cfg):
    """Smooth safety + steep tolerance repulsion, with stable exponent clipping.

    Clipping only affects the extreme overflow region; smooth tanh saturation
    makes the entire returned force C1 even in extreme penetration.
    """
    h=np.asarray(h,float)
    ds=float(cfg['safety_distance_m']); tol=float(cfg['surface_tolerance_m'])
    ls=float(cfg['safety_decay_m']); lp=float(cfg['penetration_decay_m'])
    if ls<=0 or lp<=0 or tol<0 or ds<=0:raise ValueError('invalid M3 decay/tolerance')
    # Cap exponent before exp only beyond the tanh saturation plateau.
    e1=np.exp(np.clip((ds-h)/ls,-745,80))
    e2=np.exp(np.clip((-tol-h)/lp,-745,80))
    base=float(cfg['safety_gain'])*e1+float(cfg['penetration_gain'])*e2
    cap=float(cfg.get('smooth_force_cap',3.0))
    return cap*np.tanh(base/cap)


class MultiNormalField:
    """Area-weighted quadrature of triangle normal sources.

    Source count can vary after mesh refinement; force normalizer 2*pi*sigma²
    avoids growth with sampling density for planar surfaces.
    """
    def __init__(self, vertices, faces, vertex_normals=None, cfg=None, face_normals=None):
        self.cfg=dict(cfg or {})
        v=np.asarray(vertices,float)
        f=np.asarray(faces,int)
        tri=v[f]
        e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0]
        cross=np.cross(e1,e2)
        lens=np.linalg.norm(cross,axis=1)
        good=lens>1e-14
        tri=tri[good]; f=f[good];lens=lens[good];cross=cross[good]
        self.tri=tri
        self.face_normals=(cross/lens[:,None] if face_normals is None
                           else np.asarray(face_normals,float)[good])
        self.centers=tri.mean(axis=1)
        self.areas=0.5*lens
        if vertex_normals is None:
            vn=np.zeros_like(v)
            np.add.at(vn,f[:,0],cross)
            np.add.at(vn,f[:,1],cross)
            np.add.at(vn,f[:,2],cross)
            vn/=np.maximum(np.linalg.norm(vn,axis=1,keepdims=True),1e-12)
        else: vn=np.asarray(vertex_normals,float)
        n=vn[f].mean(axis=1)
        nn=np.linalg.norm(n,axis=1)
        n[nn<1e-10]=self.face_normals[nn<1e-10]
        n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-12)
        flip=np.einsum('ij,ij->i',n,self.face_normals)<0
        n[flip]*=-1
        self.normals=n
        self.tree=cKDTree(self.centers)
        self._tri_radii=np.linalg.norm(tri-self.centers[:,None],axis=-1).max(axis=1)
        self.max_tri_radius=float(self._tri_radii.max()) if len(tri) else 0
        self.sigma=float(self.cfg.get('lateral_sigma_m',0.012))
        self.radius=self.sigma*float(self.cfg.get('lateral_cutoff_sigmas',4.0))
        self.h_out=float(self.cfg.get('outward_reach_m',0.065))
        self.h_in=float(self.cfg.get('inward_reach_m',0.06))
        self.query_radius=float(np.hypot(self.radius,max(self.h_out,self.h_in)))
        self.area_normalizer=2*np.pi*self.sigma**2
        self.use_visibility=bool(self.cfg.get('visibility_enabled',True))
        self.vis_soft=float(self.cfg.get('visibility_soft_length_m',0.001))
        self._bvh=build_triangle_bvh(self.tri) if _shadow_bvh is not None else None

    def _candidates(self,p, use_index=True):
        if use_index:
            return np.asarray(self.tree.query_ball_point(p,self.query_radius),int)
        return np.arange(len(self.centers),dtype=int)

    def _visibility(self,p,ids):
        """Segment triangle blocker, with soft transition at the end point.

        Rays from a source centroid toward P can be intercepted by unrelated
        surface sheets. Ignore source triangle itself, and hits right at source.
        Gradually fade a source across the occluding sheet over ~1 mm.
        This is a local approximation, not a proof of global visibility C1.
        """
        if not self.use_visibility:return np.ones(len(ids))
        if self._bvh is not None:
            return _shadow_bvh(np.asarray(p,float),self.centers,self.tri,np.asarray(ids,np.int64),self.vis_soft,*self._bvh)
        if _shadow_fast is not None:
            return _shadow_fast(np.asarray(p,float),self.centers,self._tri_radii,self.tri,np.asarray(ids,np.int64),self.vis_soft)
        out=np.ones(len(ids))
        for k,srcid in enumerate(ids):
            q=self.centers[srcid]; d=p-q
            L=np.linalg.norm(d)
            if L<1e-9:continue
            ray=d/L
            center=(p+q)*0.5
            # Conservative broad phase for triangles intersecting segment.
            cand=np.asarray(self.tree.query_ball_point(center,0.5*L+self.max_tri_radius+self.vis_soft),int)
            if len(cand)==0:continue
            cand=cand[cand!=srcid]
            if not len(cand):continue
            tri=self.tri[cand]
            ab=tri[:,1]-tri[:,0];ac=tri[:,2]-tri[:,0]
            # Cheap per-triangle bounding sphere rejects almost all other faces.
            z=self.centers[cand]-q
            along=np.clip(z@ray,0,L)
            sep=np.linalg.norm(z-along[:,None]*ray[None,:],axis=1)
            keep=sep<=self._tri_radii[cand]+self.vis_soft
            tri=tri[keep];cand=cand[keep]
            if not len(cand):continue
            ab=tri[:,1]-tri[:,0];ac=tri[:,2]-tri[:,0]
            aa=np.cross(np.broadcast_to(ray,ab.shape),ac)
            det=np.einsum('ij,ij->i',ab,aa)
            valid=np.abs(det)>1e-12
            inv=np.zeros_like(det);inv[valid]=1/det[valid]
            tvec=q-tri[:,0]
            u=np.einsum('ij,ij->i',tvec,aa)*inv
            qq=np.cross(tvec,ab)
            vv=qq@ray*inv
            dist=np.einsum('ij,ij->i',ac,qq)*inv
            valid&=(u>=-1e-8)&(vv>=-1e-8)&(u+vv<=1+1e-8)&(dist>1e-5)
            if np.any(valid):
                nearest=float(np.min(dist[valid]))
                # Ray hit before P -> suppressed; just beyond P -> fade smoothly.
                x=(nearest-L)/max(self.vis_soft,1e-9)
                out[k]=float(smoothstep01((x+1.0)*0.5))
        return out

    def _deep_penetration_fallback(self,p):
        """Preserve outward response beyond finite M3 source-index depth.

        Fallback applies only for very deep inward points outside the normal
        influence volume, and is a continuous local-source contribution, not
        a time-varying adaptive escape gain. It cannot establish a multi-source
        field for arbitrary very deep positions; that remains a stated limit.
        """
        count=min(16,len(self.centers))
        _,ids=self.tree.query(p,k=count)
        ids=np.atleast_1d(np.asarray(ids,int))
        delta=p-self.centers[ids]
        h=np.einsum('ij,ij->i',delta,self.normals[ids])
        rho=np.linalg.norm(delta-h[:,None]*self.normals[ids],axis=1)
        deep=(h<-.8*self.h_in)&(rho<self.radius)
        if not np.any(deep):return np.zeros(3),0.
        ids=ids[deep];h=h[deep];rho=rho[deep]
        vis=self._visibility(p,ids)
        okay=vis>.5
        if not np.any(okay):return np.zeros(3),0.
        ids=ids[okay];h=h[okay];rho=rho[okay];vis=vis[okay]
        j=int(np.argmin(np.linalg.norm(p-self.centers[ids],axis=1)))
        fac=float(smoothstep01((-h[j]-.8*self.h_in)/(.2*self.h_in)))
        value=fac*vis[j]*float(exp_soft_force(np.array([h[j]]),self.cfg)[0])
        vec=value*self.normals[ids[j]]
        return vec,abs(value)

    def evaluate(self,points, *, spatial_index=True, visibility=None, return_details=False):
        P=np.atleast_2d(np.asarray(points,float))
        if P.shape[1]!=3:raise ValueError('expected (N,3)')
        F=np.zeros((len(P),3)); count=np.zeros(len(P),int); magnitudes=np.zeros(len(P))
        details=[]
        old=self.use_visibility
        if visibility is not None:self.use_visibility=bool(visibility)
        try:
            for pi,p in enumerate(P):
                ids=self._candidates(p,spatial_index)
                if len(ids)==0:
                    if return_details:details.append(dict(source_ids=[],sources=[],magnitudes=[],h=[],rho=[],visibility=[]))
                    continue
                delta=p-self.centers[ids]
                hh=np.einsum('ij,ij->i',delta,self.normals[ids])
                rho=np.linalg.norm(delta-hh[:,None]*self.normals[ids],axis=1)
                valid=(rho<self.radius)&(hh<self.h_out)&(hh> -self.h_in)
                ids=ids[valid]; hh=hh[valid];rho=rho[valid]
                if len(ids)==0:
                    if return_details:details.append(dict(source_ids=[],sources=[],magnitudes=[],h=[],rho=[],visibility=[]))
                    continue
                vis=self._visibility(p,ids)
                # C1 zero at lateral and longitudinal search boundaries, without
                # a sudden jump from index filtering. Taper only at the edges.
                wr=np.exp(-0.5*(rho/self.sigma)**2)
                wr*=smoothstep01((self.radius-rho)/(0.75*self.sigma))
                bh=smoothstep01((self.h_out-hh)/max(self.h_out*0.2,1e-8))
                bh*=smoothstep01((hh+self.h_in)/max(self.h_in*0.2,1e-8))
                coef=self.areas[ids]/self.area_normalizer*wr*bh*vis*exp_soft_force(hh,self.cfg)
                vectors=coef[:,None]*self.normals[ids]
                F[pi]=vectors.sum(axis=0)
                count[pi]=int(np.count_nonzero(coef>1e-12))
                magnitudes[pi]=float(np.sum(np.linalg.norm(vectors,axis=1)))
                if return_details:
                    details.append(dict(source_ids=ids.tolist(),sources=vectors.tolist(),magnitudes=coef.tolist(),h=hh.tolist(),rho=rho.tolist(),visibility=vis.tolist()))
            # Very deep penetration beyond finite source support must never
            # silently become zero repulsion. Ordinary near-surface samples
            # never enter this fallback.
            for pi,p in enumerate(P):
                extra,magnitude=self._deep_penetration_fallback(p)
                if magnitude>0:
                    F[pi]+=extra
                    magnitudes[pi]+=magnitude
                    count[pi]+=1
                    if return_details:details[pi]['deep_fallback_magnitude']=magnitude
        finally:self.use_visibility=old
        diag=dict(source_count=count,source_sum_magnitude=magnitudes,net_magnitude=np.linalg.norm(F,axis=1))
        if return_details:diag['details']=details
        return F,diag

# Optional compiled visibility accelerator: exact segment/triangle intersections,
# without Python-level ray loops. Semantics match the reference code above.
try:
    from numba import njit
    @njit(cache=True)
    def _shadow_fast(p, centers, radii, tri, ids, soft):
        out=np.ones(len(ids),dtype=np.float64)
        for k in range(len(ids)):
            src=ids[k]
            q=centers[src]
            dx=p[0]-q[0];dy=p[1]-q[1];dz=p[2]-q[2]
            L=np.sqrt(dx*dx+dy*dy+dz*dz)
            if L<1e-9:continue
            rx=dx/L;ry=dy/L;rz=dz/L
            best=1e30
            for j in range(len(tri)):
                if j==src:continue
                # Conservative bounding sphere test for potential triangle hit.
                ox=centers[j,0]-q[0];oy=centers[j,1]-q[1];oz=centers[j,2]-q[2]
                t=ox*rx+oy*ry+oz*rz
                t0=min(L,max(0.0,t))
                dd=(ox-t0*rx)**2+(oy-t0*ry)**2+(oz-t0*rz)**2
                if dd>(radii[j]+soft)**2:continue
                a=tri[j,0];b=tri[j,1];c=tri[j,2]
                abx=b[0]-a[0];aby=b[1]-a[1];abz=b[2]-a[2]
                acx=c[0]-a[0];acy=c[1]-a[1];acz=c[2]-a[2]
                hx=ry*acz-rz*acy;hy=rz*acx-rx*acz;hz=rx*acy-ry*acx
                det=abx*hx+aby*hy+abz*hz
                if abs(det)<1e-12:continue
                inv=1.0/det
                sx=q[0]-a[0];sy=q[1]-a[1];sz=q[2]-a[2]
                u=(sx*hx+sy*hy+sz*hz)*inv
                if u<-1e-8 or u>1.0+1e-8:continue
                qx=sy*abz-sz*aby;qy=sz*abx-sx*abz;qz=sx*aby-sy*abx
                v=(rx*qx+ry*qy+rz*qz)*inv
                if v<-1e-8 or u+v>1.0+1e-8:continue
                distance=(acx*qx+acy*qy+acz*qz)*inv
                if distance>1e-5 and distance<best:best=distance
            if best<1e29:
                x=(best-L)/max(soft,1e-9)
                z=max(0.0,min(1.0,(x+1.0)*0.5))
                out[k]=z*z*(3-2*z)
        return out
except ImportError:
    _shadow_fast=None


def build_triangle_bvh(tri,leaf_size=8):
    """Compact flat AABB hierarchy for accelerated shadow-ray intersections."""
    centers=tri.mean(axis=1)
    bounds_min=tri.min(axis=1);bounds_max=tri.max(axis=1)
    all_bmin=[];all_bmax=[];all_left=[];all_right=[];all_start=[];all_count=[]
    order=[]
    def rec(ids):
        node=len(all_left)
        all_bmin.append(bounds_min[ids].min(axis=0))
        all_bmax.append(bounds_max[ids].max(axis=0))
        all_left.append(-1);all_right.append(-1)
        all_start.append(0);all_count.append(0)
        if len(ids)<=leaf_size:
            all_start[node]=len(order);all_count[node]=len(ids)
            order.extend(ids.tolist())
        else:
            length=np.ptp(centers[ids],axis=0)
            axis=int(np.argmax(length));ss=np.argsort(centers[ids,axis],kind='mergesort')
            mid=len(ids)//2
            all_left[node]=rec(ids[ss[:mid]])
            all_right[node]=rec(ids[ss[mid:]])
        return node
    rec(np.arange(len(tri)))
    return (np.asarray(all_bmin),np.asarray(all_bmax),np.asarray(all_left,np.int64),
            np.asarray(all_right,np.int64),np.asarray(all_start,np.int64),
            np.asarray(all_count,np.int64),np.asarray(order,np.int64))

try:
    @njit(cache=True)
    def _shadow_bvh(p,centers,tri,ids,soft,bmin,bmax,left,right,start,count,order):
        out=np.ones(len(ids),dtype=np.float64)
        for k in range(len(ids)):
            src=ids[k];q=centers[src]
            dx=p[0]-q[0];dy=p[1]-q[1];dz=p[2]-q[2]
            L=np.sqrt(dx*dx+dy*dy+dz*dz)
            if L<1e-9:continue
            rx=dx/L;ry=dy/L;rz=dz/L
            ray=np.array([rx,ry,rz])
            best=1e30
            stack=np.empty(128,np.int64);top=1;stack[0]=0
            while top:
                top-=1;node=stack[top]
                lo=-1e20;hi=min(L+soft,best)
                hit=True
                for a in range(3):
                    if abs(ray[a])<1e-15:
                        if q[a]<bmin[node,a] or q[a]>bmax[node,a]:hit=False;break
                    else:
                        t0=(bmin[node,a]-q[a])/ray[a]
                        t1=(bmax[node,a]-q[a])/ray[a]
                        if t0>t1:t0,t1=t1,t0
                        lo=max(lo,t0);hi=min(hi,t1)
                        if lo>hi:hit=False;break
                if not hit or hi<1e-5:continue
                if count[node]:
                    for kk in range(start[node],start[node]+count[node]):
                        j=order[kk]
                        if j==src:continue
                        a=tri[j,0];b=tri[j,1];c=tri[j,2]
                        abx=b[0]-a[0];aby=b[1]-a[1];abz=b[2]-a[2]
                        acx=c[0]-a[0];acy=c[1]-a[1];acz=c[2]-a[2]
                        hx=ry*acz-rz*acy;hy=rz*acx-rx*acz;hz=rx*acy-ry*acx
                        det=abx*hx+aby*hy+abz*hz
                        if abs(det)<1e-12:continue
                        inv=1/det
                        sx=q[0]-a[0];sy=q[1]-a[1];sz=q[2]-a[2]
                        u=(sx*hx+sy*hy+sz*hz)*inv
                        if u<-1e-8 or u>1.0+1e-8:continue
                        qx=sy*abz-sz*aby;qy=sz*abx-sx*abz;qz=sx*aby-sy*abx
                        v=(rx*qx+ry*qy+rz*qz)*inv
                        if v<-1e-8 or u+v>1.0+1e-8:continue
                        distance=(acx*qx+acy*qy+acz*qz)*inv
                        if distance>1e-5 and distance<best:best=distance
                else:
                    if top+2>=len(stack):raise RuntimeError('BVH stack exceeded capacity')
                    stack[top]=left[node];stack[top+1]=right[node];top+=2
            if best<1e29:
                z=max(0.0,min(1.0,((best-L)/max(soft,1e-9)+1)*0.5))
                out[k]=z*z*(3-2*z)
        return out
except (NameError,ImportError):
    _shadow_bvh=None