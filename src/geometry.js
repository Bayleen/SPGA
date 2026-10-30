// SPGA Web Lab geometric interaction model.
// M2-inspired: smooth interpolated surface normal and a collision-aware rod
// orientation search. This is NOT the archived SPGA-M2 force-balance solver.
export const V = (x=0,y=0,z=0)=>({x,y,z});
export const add=(a,b)=>V(a.x+b.x,a.y+b.y,a.z+b.z);
export const sub=(a,b)=>V(a.x-b.x,a.y-b.y,a.z-b.z);
export const mul=(a,s)=>V(a.x*s,a.y*s,a.z*s);
export const dot=(a,b)=>a.x*b.x+a.y*b.y+a.z*b.z;
export const cross=(a,b)=>V(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);
export const len=a=>Math.hypot(a.x,a.y,a.z);
export const norm=a=>mul(a,1/Math.max(len(a),1e-10));
export const mix=(a,b,t)=>add(mul(a,1-t),mul(b,t));
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export const rad=d=>d*Math.PI/180;
export const angle=(a,b)=>Math.acos(clamp(dot(norm(a),norm(b)),-1,1));
export const defaultProfile=Array.from({length:70},(_,i)=>{
  const x=i/69; return {x,y:clamp(0.60-0.30*Math.exp(-Math.pow((x-0.36)/0.20,2))-0.15*Math.exp(-Math.pow((x-0.73)/0.095,2))+0.045*Math.sin(x*16),0.10,0.86)};
});
export const defaultStroke=Array.from({length:80},(_,i)=>({x:i/79,y:0.50+0.26*Math.sin((i/79)*Math.PI*3.6)}));
export function normalizeStroke(raw, forceEndpoints=true){
  if(!Array.isArray(raw)||raw.length<2) return null;
  const ordered=raw.map(p=>({x:clamp(Number(p.x),0,1),y:clamp(Number(p.y),0,1)})).filter(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)).sort((a,b)=>a.x-b.x);
  if(ordered.length<2) return null;
  // Binning also removes accidental backward strokes and repeated X positions.
  const buckets=Array.from({length:90},()=>[]);
  for(const p of ordered) buckets[Math.min(89,Math.floor(p.x*90))].push(p.y);
  let points=buckets.flatMap((v,i)=>v.length?[{x:(i+.5)/90,y:v.reduce((a,b)=>a+b,0)/v.length}]:[]);
  if(points.length<2) return null;
  if(forceEndpoints) points=[{x:0,y:points[0].y},...points,{x:1,y:points.at(-1).y}];
  return points;
}
export function atStroke(points,t){
  t=clamp(t,0,1);
  if(!points?.length)return .5;
  if(t<=points[0].x)return points[0].y;
  for(let i=1;i<points.length;i++)if(t<=points[i].x){
    const a=points[i-1],b=points[i],dx=Math.max(1e-8,b.x-a.x);
    const p=points[Math.max(0,i-2)],q=points[Math.min(points.length-1,i+1)];
    // Shared finite-difference tangents give a C1 continuous Hermite profile.
    const m0=(b.y-p.y)/Math.max(1e-8,b.x-p.x);
    const m1=(q.y-a.y)/Math.max(1e-8,q.x-a.x);
    const h=(t-a.x)/dx,h2=h*h,h3=h2*h;
    const y=(2*h3-3*h2+1)*a.y + (h3-2*h2+h)*dx*m0
      +(-2*h3+3*h2)*b.y +(h3-h2)*dx*m1;
    return clamp(y,0,1);
  }
  return points.at(-1).y;
}
export function smoothStroke(points,count=72,spread=.019){
  const out=[];
  for(let i=0;i<count;i++){
    const x=i/(count-1);let total=0,ws=0;
    for(let j=-9;j<=9;j++){
      const delta=j*spread/5,weight=Math.exp(-.5*(j/5)**2);
      total+=weight*atStroke(points,clamp(x+delta,0,1));ws+=weight;
    }
    out.push({x,y:clamp(total/ws,0,1)});
  }
  return out;
}

export function terrainHeight(type,x,z,profile=defaultProfile){
  if(type==='profile'){
    const t=(x+3)/6;
    return 0.18+(0.76-atStroke(profile,t))*3.65;
  }
  if(type==='saddle'){
    const a=x*.75,b=z*.86;
    return 0.82+0.24*(a*a-b*b)+0.32*Math.sin(1.6*x+.38*z)*Math.cos(1.5*z-.18*x)
      +0.37*Math.exp(-((x+1.30)**2+(z-0.65)**2)/.65);
  }
  // Multi-peak rolling hill, with asymmetric ridges and hollows.
  return .27+1.48*Math.exp(-((x+.70)**2/2.5+(z-.25)**2/2.1))
    +.93*Math.exp(-((x-1.28)**2/.78+(z+1.16)**2/1.35))
    -.63*Math.exp(-((x-.2)**2/.66+(z-1.47)**2/.48))
    +.17*Math.sin(x*2.1+z*.64)*Math.cos(z*1.55);
}
export function sampleSurface(state,x,z){
  const {offsetX=0,offsetZ=0,offsetY=0,rotation=0,scale=1,type='hill',profile=defaultProfile}=state;
  const c=Math.cos(rad(rotation)),s=Math.sin(rad(rotation));
  const xx=x-offsetX, zz=z-offsetZ;
  const lx=(c*xx+s*zz)/scale, lz=(-s*xx+c*zz)/scale;
  if(Math.abs(lx)>3||Math.abs(lz)>3)return null;
  const hh=terrainHeight(type,lx,lz,profile);
  const d=.014;
  const hx=(terrainHeight(type,lx+d,lz,profile)-terrainHeight(type,lx-d,lz,profile))/(2*d);
  const hz=(terrainHeight(type,lx,lz+d,profile)-terrainHeight(type,lx,lz-d,profile))/(2*d);
  const nLocal=norm(V(-hx,1,-hz));
  const normal=norm(V(c*nLocal.x-s*nLocal.z,nLocal.y,s*nLocal.x+c*nLocal.z));
  return {point:V(x,offsetY+scale*hh,z),normal,lx,lz};
}
export function worldFromLocal(state,x,z){
  const a=rad(state.rotation??0),c=Math.cos(a),s=Math.sin(a),k=state.scale??1;
  return V((state.offsetX??0)+k*(x*c-z*s),0,(state.offsetZ??0)+k*(x*s+z*c));
}
export function getMesh(state,N=45){
  const verts=[], faces=[];
  for(let iz=0;iz<=N;iz++)for(let ix=0;ix<=N;ix++){
    const x=-3+ix*6/N,z=-3+iz*6/N;
    const p=worldFromLocal(state,x,z), sample=sampleSurface(state,p.x,p.z);
    verts.push(sample.point);
  }
  for(let j=0;j<N;j++)for(let i=0;i<N;i++){
    const a=j*(N+1)+i,b=a+1,c=a+N+1,d=c+1;
    faces.push([a,c,b],[b,c,d]);
  }
  return {verts,faces,N};
}
export function getCut(state,angleDeg=0,offset=0,count=115){
  const a=rad(angleDeg), tangent=V(Math.cos(a),0,Math.sin(a)), side=V(-Math.sin(a),0,Math.cos(a));
  const center=add(V(state.offsetX??0,0,state.offsetZ??0),mul(side,offset));
  const points=[];
  for(let i=0;i<count;i++){
    const u=(i/(count-1)-.5)*6.4;
    const p=add(add(center,mul(tangent,u)),V(0,0,0));
    const sm=sampleSurface(state,p.x,p.z);
    if(sm)points.push(sm.point);
  }
  return {tangent,side,center,points};
}
export function makeContactPath(state,stroke,angle,offset,count=67,amplitude=1){
  const a=rad(angle),tangent=V(Math.cos(a),0,Math.sin(a)),side=V(-Math.sin(a),0,Math.cos(a));
  const out=[];
  for(let i=0;i<count;i++){
    const u=i/(count-1);
    const lateral=((.5-atStroke(stroke,u))*2)*amplitude;
    const p=add(add(V(state.offsetX??0,0,state.offsetZ??0),mul(tangent,(u-.5)*5.2*(state.scale??1))),mul(side,offset+lateral));
    const surface=sampleSurface(state,p.x,p.z);
    if(surface)out.push({...surface,nominalProgress:u});
  }
  return out;
}
export function makeTilted(contacts,length,tiltDeg,reverse=false){
  return contacts.map((c,i)=>{
    const before=contacts[Math.max(0,i-1)].point,after=contacts[Math.min(i+1,contacts.length-1)].point;
    let t=norm(sub(after,before));
    t=norm(sub(t,mul(c.normal,dot(t,c.normal))));
    if(reverse)t=mul(t,-1);
    const axis=norm(add(mul(c.normal,Math.cos(rad(tiltDeg))),mul(t,Math.sin(rad(tiltDeg)))));
    return {a0:c.point,axis,a1:add(c.point,mul(axis,length)),normal:c.normal,guide:c.nominalProgress};
  });
}
export function rodMetrics(pose,state,length=1){
  let minClearance=Infinity,penetration=0,active=0;
  for(let k=1;k<=24;k++){
    const tau=k/24,p=add(pose.a0,mul(pose.axis,tau*length));
    const s=sampleSurface(state,p.x,p.z);
    if(!s)continue;
    // Signed-height distance approximates the distance measured along the local surface normal.
    const phi=(p.y-s.point.y)*s.normal.y;
    minClearance=Math.min(minClearance,phi);
    penetration=Math.max(penetration,Math.max(0,-phi));
    active++;
  }
  return {minClearance,penetration,active,collides:penetration>0.004};
}
export function warpDemo(poses,state,length,betaDeg){
  const beta=rad(betaDeg),N=poses.length;
  const warped=[];
  let failure=0;
  // Independent pose search, constrained to a cone centered at each tilted pose.
  // Preserve A0 and rod length exactly. This is an interactive approximation,
  // NOT the M2 active-contact + conic + ribbon force-balance algorithm.
  for(let j=0;j<N;j++){
    const p=poses[j],basis1=norm(sub(V(0,1,0),mul(p.axis,dot(V(0,1,0),p.axis))));
    const b1=len(basis1)<.1?norm(cross(p.axis,V(1,0,0))):basis1;
    const b2=norm(cross(p.axis,b1));
    let best=p.axis,bestScore=Infinity,bestMetrics=rodMetrics(p,state,length),bestTheta=0;
    for(let ri=0;ri<=14;ri++){
      const theta=beta*ri/14;
      const steps=ri===0?1:32;
      for(let k=0;k<steps;k++){
        const t=2*Math.PI*k/steps;
        const offset=add(mul(b1,Math.cos(t)),mul(b2,Math.sin(t)));
        const axis=norm(add(mul(p.axis,Math.cos(theta)),mul(offset,Math.sin(theta))));
        const mm=rodMetrics({...p,axis},state,length);
        // Strong penetration penalty, gentle pose-change prior. Small safe margin.
        const deficit=Math.max(0,.022-mm.minClearance);
        const score=mm.penetration*250+Math.min(deficit,.12)*10+theta*.095;
        if(score<bestScore){bestScore=score;best=axis;bestMetrics=mm;bestTheta=theta;}
      }
    }
    if(bestMetrics.penetration>.004)failure++;
    warped.push({...p,axis:best,a1:add(p.a0,mul(best,length)),deviationDeg:bestTheta*180/Math.PI});
  }
  return {warped,failure};
}
export function analyzePath(poses,state,length){
  if(!poses.length)return {count:0,collisions:0,maxPenetration:0,meanPenetration:0,minClearance:null};
  let collisions=0,maxPenetration=0,totalPen=0,minClearance=Infinity;
  for(const pose of poses){
    const m=rodMetrics(pose,state,length);
    collisions+=m.collides?1:0;
    maxPenetration=Math.max(maxPenetration,m.penetration);
    totalPen+=m.penetration;
    minClearance=Math.min(minClearance,m.minClearance);
  }
  return {count:poses.length,collisions,maxPenetration,meanPenetration:totalPen/poses.length,minClearance:Number.isFinite(minClearance)?minClearance:null};
}