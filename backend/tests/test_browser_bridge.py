"""Ensure WebAssembly-ready Python adapter computes same research M3 as native API."""
import json
import numpy as np
from app import WarpRequest, solve
from browser_bridge import solve_json

def test_browser_bridge_matches_original_solver():
    n=7; verts=[]; faces=[]
    for j in range(n):
        for i in range(n):
            x=(i/(n-1)-0.5)*4
            z=(j/(n-1)-0.5)*4
            verts.append({'x':x,'y':0.2,'z':z})
    for j in range(n-1):
        for i in range(n-1):
            a=j*n+i;b=a+1;c=a+n;d=c+1
            faces.extend([(a,c,b),(b,c,d)])
    poses=[
        {'a0':{'x':-0.8,'y':0.10,'z':0.0},'a1':{'x':0.2,'y':0.10,'z':0.0},'contact':True,'passId':0},
        {'a0':{'x':-0.8,'y':0.15,'z':0.2},'a1':{'x':0.2,'y':0.15,'z':0.2},'contact':False,'passId':0},
        {'a0':{'x':-0.8,'y':0.10,'z':0.4},'a1':{'x':0.2,'y':0.10,'z':0.4},'contact':True,'passId':0},
    ]
    src=dict(verts=verts,faces=faces,poses=poses,rodLength=1.0,coneAngle=30,
             gains=dict(geometry=1,conic=1,tension=.6,bending=.35),maxIterations=20)
    expected=solve(WarpRequest(**src))
    actual=json.loads(solve_json(json.dumps(src)))
    assert actual['solver']==expected['solver']
    for a,b in zip(actual['warped'],expected['warped']):
        for end in ('a0','a1','axis'):
            assert np.allclose(list(a[end].values()),list(b[end].values()),atol=1e-10)
    assert actual['diagnostics']['iterations']==expected['diagnostics']['iterations']
    assert actual['diagnostics']['mode']==expected['diagnostics']['mode']
