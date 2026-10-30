import test from 'node:test';
import assert from 'node:assert/strict';
import {V,len,sub,angle,defaultProfile,defaultStroke,sampleSurface,worldFromLocal,getCut,makeContactPath,makeTilted,warpDemo,rodMetrics,analyzePath,normalizeStroke,atStroke,getMesh} from '../src/geometry.js';
const base={type:'hill',profile:defaultProfile,offsetX:0,offsetZ:0,offsetY:0,rotation:0,scale:1};
const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
for(const type of ['hill','saddle','profile'])test(`${type} heightfield produces finite upward unit normal`,()=>{
 const s={...base,type};for(let x=-2;x<2;x+=.43)for(let z=-2;z<2;z+=.49){const p=sampleSurface(s,x,z);assert.ok(p&&Number.isFinite(p.point.y));close(len(p.normal),1,1e-8);assert.ok(p.normal.y>0);}
});
test('2D profile extrudes unchanged along local z',()=>{
 const s={...base,type:'profile'};close(sampleSurface(s,.35,-2).point.y,sampleSurface(s,.35,1.6).point.y);
});
test('rotation and translation preserve terrain sampling',()=>{
 const s={...base,rotation:53,offsetX:1.3,offsetZ:-1.1,scale:1.2};const w=worldFromLocal(s,.75,-.3);close(sampleSurface(s,w.x,w.z).point.y,base.scale*0+sampleSurface(base,.75,-.3).point.y*1.2,1e-6);
});
test('guide sampled on vertical cutting plane',()=>{
 const cut=getCut(base,34,.3);assert.ok(cut.points.length>50);
 for(const p of cut.points){const q=sampleSurface(base,p.x,p.z);close(p.y,q.point.y);}
});
test('nominal drawing normalization is robust to backwards stroke',()=>{
 const points=normalizeStroke([{x:.9,y:.3},{x:.5,y:.5},{x:.1,y:.6},{x:0,y:.7}]);assert.ok(points&&points.length>3);close(atStroke(points,0),.7,0.04);
});
test('tilt keeps A0 on surface and rod length exactly fixed',()=>{
 const c=makeContactPath(base,defaultStroke,0,0,67,.7),tilted=makeTilted(c,1.2,65);
 assert.equal(tilted.length,67);for(let i=0;i<tilted.length;i++){close(len(sub(tilted[i].a1,tilted[i].a0)),1.2);close(tilted[i].a0.y,sampleSurface(base,tilted[i].a0.x,tilted[i].a0.z).point.y);}
});
test('default Hill demo makes collision changes visible and improves them',()=>{
 const c=makeContactPath(base,defaultStroke,0,0,67,.7),t=makeTilted(c,1.2,65),w=warpDemo(t,base,1.2,40).warped;
 const before=analyzePath(t,base,1.2),after=analyzePath(w,base,1.2);
 assert.ok(before.collisions>=1);assert.ok(after.collisions<before.collisions);assert.ok(after.maxPenetration<=before.maxPenetration+1e-8);
 for(let i=0;i<w.length;i++){close(len(sub(w[i].a0,t[i].a0)),0);close(len(sub(w[i].a1,w[i].a0)),1.2);assert.ok(angle(w[i].axis,t[i].axis)<=40*Math.PI/180+1e-5);}
});
test('zero cone deviation fixes all original tilted axes',()=>{
 const c=makeContactPath(base,defaultStroke,0,0,7),t=makeTilted(c,1.2,65),w=warpDemo(t,base,1.2,0).warped;
 for(let i=0;i<t.length;i++)close(angle(t[i].axis,w[i].axis),0,1e-7);
});
test('mesh has valid triangulation',()=>{const m=getMesh(base,12);assert.equal(m.verts.length,169);assert.equal(m.faces.length,288);});