import test from 'node:test';
import assert from 'node:assert/strict';
import {V,len,sub,angle,defaultProfile,defaultStroke,sampleSurface,worldFromLocal,getCut,makeContactPath,makeTilted,warpDemo,rodMetrics,analyzePath,normalizeStroke,atStroke,getMesh,rad,norm,dot,mul} from '../src/geometry.js';
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

test('transformed mesh does not lose boundary vertices through floating-point error',()=>{
 for(const rot of [-165,-90,15,133])for(const dx of [-2.8,-1.3,0,2.1]){
  const state={...base,rotation:rot,offsetX:dx,offsetZ:1.7,scale:.9};
  const mesh=getMesh(state,41);
  assert.equal(mesh.verts.length,1764);
  assert.ok(mesh.verts.every(p=>Number.isFinite(p.y)));
 }
});
test('Tilted heading follows fixed guide direction, not freehand lateral wiggles',()=>{
 const contacts=makeContactPath(base,defaultStroke,37,.12,67,1.2);
 const arr=makeTilted(contacts,1.2,65,false,37);
 const fwd=V(Math.cos(rad(37)),0,Math.sin(rad(37)));
 for(const p of arr){
  const tang=norm(sub(p.axis,mul(p.normal,dot(p.axis,p.normal))));
  const expected=norm(sub(fwd,mul(p.normal,dot(fwd,p.normal))));
  close(angle(tang,expected),0,1e-7);
  close(angle(p.axis,p.normal),rad(65),1e-7);
 }
});

// v2: strictly distinguish source skill from guide and preserve primitive similarity.
import {sketchToSkill,makeSkillExample,parseNominalCSV,buildGuide,pointOnGuide,placeNominal,placeContactGuide} from '../src/geometry.js';
test('2D handwritten loops retain stroke order instead of sorting by X',()=>{
 const raw=[{x:.10,y:.78},{x:.45,y:.22},{x:.75,y:.38},{x:.33,y:.43},{x:.75,y:.6},{x:.91,y:.77}];
 const pts=sketchToSkill(raw,101);assert.equal(pts.length,101);
 assert.ok(pts.some((p,i)=>i&&p.u<pts[i-1].u-.01),'drawn backward loop should survive resampling');
 close(pts[0].u,0);close(pts[0].v,0);close(pts.at(-1).u,1);close(pts.at(-1).v,0);
 assert.equal(pts.filter(p=>p.contact).length,2);
});
test('spiral is genuinely 3D and source endpoints are contact only',()=>{
 const s=makeSkillExample('spiral');assert.ok(Math.max(...s.map(p=>p.w))>.10);assert.ok(Math.min(...s.map(p=>p.w))<-.10);
 assert.equal(s.filter(p=>p.contact).length,2);
});
test('each lifted primitive tiled with a single similarity, two exact guide foot anchors',()=>{
 const guide=buildGuide(base,Array.from({length:60},(_,i)=>({x:i/59,y:.5})),16,.2);
 const nominal=makeSkillExample('spiral');const tiled=placeNominal(nominal,guide,3,65,1.2);
 assert.equal(tiled.poses.length,nominal.length*3);assert.equal(tiled.poses.filter(p=>p.contact).length,6);
 for(let k=0;k<3;k++){
   const block=tiled.poses.slice(k*nominal.length,(k+1)*nominal.length),gr=tiled.groups[k];
   close(len(sub(block[0].a0,gr.start)),0,1e-7);close(len(sub(block.at(-1).a0,gr.end)),0,1e-7);
   for(let i of [5,20,38,55,75])for(let j of [2,30,70]){
     const a=nominal[i],b=nominal[j],source=Math.hypot(a.u-b.u,a.v-b.v,a.w-b.w);
     close(len(sub(block[i].a0,block[j].a0)),source*gr.scaleFactor,1e-7);
   }
   for(const p of block){close(len(sub(p.a1,p.a0)),1.2);}
   assert.ok(block.slice(1,-1).some(p=>p.a0.y>Math.min(gr.start.y,gr.end.y)+.15),'noncontact interior lifts');
 }
});
test('all-contact guide samples all touch the surface with fixed tool length',()=>{
 const guide=buildGuide(base,defaultStroke,12,0,95,.5);
 const poses=placeContactGuide(guide,1.2,55,false,null,67).poses;
 assert.equal(poses.length,67);assert.ok(poses.every(p=>p.contact));
 for(const p of poses){close(len(sub(p.a1,p.a0)),1.2);close(sampleSurface(base,p.a0.x,p.a0.z).point.y,p.a0.y,1e-4);}
});
test('CSV nominal accepts ordered 3D u/v/w and rejects invalid columns',()=>{
 const poses=parseNominalCSV('u,v,w,contact,ax,ay,az\n0,0,0,1,1,0,0\n.25,.2,.14,0,1,0,0\n.15,.3,-.12,0,1,0,0\n1,0,0,1,1,0,0');
 assert.equal(poses.length,4);close(poses[2].u,.15);assert.ok(poses[2].axis);
 assert.throws(()=>parseNominalCSV('x,y,z\n1,2,3\n2,3,4\n3,4,5'),/missing/);
});
test('warp demo preserves all declared contact A0 anchors and fixed length',()=>{
 const guide=buildGuide(base,defaultStroke,0,0);
 const tilted=placeNominal(makeSkillExample('half',21),guide,1,78,.9).poses;
 const warped=warpDemo(tilted,base,.9,30).warped;
 for(let i=0;i<tilted.length;i++){
   if(tilted[i].contact)close(len(sub(warped[i].a0,tilted[i].a0)),0);
   close(len(sub(warped[i].a1,warped[i].a0)),.9);
 }
});