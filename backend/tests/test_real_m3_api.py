from fastapi.testclient import TestClient
import numpy as np
from app import api, WarpRequest, solve


def simple_case():
    n=7
    verts=[]; faces=[]
    for j in range(n):
        for i in range(n):
            x=(i/(n-1)-.5)*4
            z=(j/(n-1)-.5)*4
            y=.2+.6*np.exp(-(x*x+z*z)/1.5)
            verts.append({'x':x,'y':float(y),'z':z})
    for j in range(n-1):
        for i in range(n-1):
            a=j*n+i;b=a+1;c=a+n;d=c+1
            faces.extend([(a,c,b),(b,c,d)])
    poses=[]
    for k in range(3):
        z=(k-1)*.4
        poses.append({'a0':{'x':-.8,'y':.3,'z':z},'a1':{'x':.2,'y':.3,'z':z},'contact':k==0,'passId':0})
    return dict(verts=verts,faces=faces,poses=poses,rodLength=1.0,coneAngle=30,maxIterations=20)


def test_core_constrained_geometry():
    req=WarpRequest(**simple_case())
    out=solve(req)
    assert out['solver']=='SPGA-M3-20261102-original-python'
    assert out['diagnostics']['iterations']>=20
    A0=np.asarray([[p['a0'][coord] for coord in 'xyz'] for p in out['warped']])
    A1=np.asarray([[p['a1'][coord] for coord in 'xyz'] for p in out['warped']])
    assert np.allclose(np.linalg.norm(A1-A0,axis=1),1.0,atol=1e-8)
    assert np.allclose(A0[0],[-.8,.3,-.4],atol=1e-8)
    assert any(abs(p['deviationDeg'])>0.01 for p in out['warped'])


def test_health():
    r=TestClient(api).get('/api/health')
    assert r.status_code==200
    assert 'M3' in r.json()['solver']


def test_reject_bad_rod():
    data=simple_case()
    data['rodLength']=2.0
    assert TestClient(api).post('/api/warp',json=data).status_code==422