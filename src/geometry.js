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
  if(Math.abs(lx)>3+1e-8||Math.abs(lz)>3+1e-8)return null;
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
/**
 * Nominal tilted tool: its in-surface lean follows the *guide* forward axis,
 * NOT the derivative of the user's lateral freehand stroke.  This prevents
 * large tool-heading swings when the nominal stroke wiggles or turns sharply.
 * tiltDeg is measured from the local surface normal (0° = normal).
 */
export function makeTilted(contacts,length,tiltDeg,reverse=false,guideAngleDeg=0){
  const a=rad(guideAngleDeg);
  const forward=V(Math.cos(a),0,Math.sin(a));
  return contacts.map(c=>{
    // Keep the chosen guide direction tangent to the surface at each A0.
    const projected=sub(forward,mul(c.normal,dot(forward,c.normal)));
    let tangent=norm(projected);
    if(reverse)tangent=mul(tangent,-1);
    const axis=norm(add(mul(c.normal,Math.cos(rad(tiltDeg))),mul(tangent,Math.sin(rad(tiltDeg)))));
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
    // Only explicit non-contact A0 samples may move. Contact anchors remain exact.
    // This is a local browser-demo lift heuristic, not M2 ribbon/force balance.
    let resultA0=p.a0;
    if(p.contact===false && bestMetrics.penetration>.004){
      const lift=clamp(bestMetrics.penetration+.025,0,.35);
      const nn=p.normal??V(0,1,0),candidate=add(p.a0,mul(nn,lift));
      const liftedMetrics=rodMetrics({...p,a0:candidate},state,length);
      if(liftedMetrics.penetration<bestMetrics.penetration){resultA0=candidate;bestMetrics=liftedMetrics;}
    }
    if(bestMetrics.penetration>.004)failure++;
    warped.push({...p,a0:resultA0,axis:best,a1:add(resultA0,mul(best,length)),deviationDeg:bestTheta*180/Math.PI});
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

// ----- v2: independent nominal skill (A0 SE(3) source), guide, placement -----
// Sketch samples are ordered by pen time. NEVER sort by x: loops/spirals need order.
export function resampleOrdered(points,count=81){
  if(points.length<2)throw new Error('Need at least two points');
  const src=[points[0]],s=[0];
  for(let i=1;i<points.length;i++){
    const d=Math.hypot(points[i].u-src.at(-1).u,points[i].v-src.at(-1).v,points[i].w-src.at(-1).w);
    if(d<1e-7)continue;src.push(points[i]);s.push(s.at(-1)+d);
  }
  if(src.length<2||s.at(-1)<1e-6)throw new Error('Nominal stroke is too short');
  const out=[];let j=1;
  for(let i=0;i<count;i++){
    const target=i/(count-1)*s.at(-1);
    while(j<s.length-1&&s[j]<target)j++;
    const t=clamp((target-s[j-1])/Math.max(1e-9,s[j]-s[j-1]),0,1);
    const a=src[j-1],b=src[j];out.push({u:a.u+(b.u-a.u)*t,v:a.v+(b.v-a.v)*t,w:a.w+(b.w-a.w)*t,contact:i===0||i===count-1});
  }
  return out;
}
export function sketchToSkill(raw,count=81){
  if(!Array.isArray(raw)||raw.length<2)throw new Error('Draw a path from START to END');
  const first=raw[0],last=raw.at(-1),dx=last.x-first.x;
  if(dx<.22)throw new Error('End point must be to the right of Start (at least 22% canvas width)');
  const pts=raw.map(p=>{
    const u=(p.x-first.x)/dx;
    const baseline=first.y+(last.y-first.y)*u;
    return {u,v:(baseline-p.y)/dx,w:0};
  });
  const out=resampleOrdered(pts,count);
  out[0]={u:0,v:0,w:0,contact:true};out[out.length-1]={u:1,v:0,w:0,contact:true};
  return out;
}
export function makeSkillExample(type='half',count=81){
  return Array.from({length:count},(_,i)=>{
    const t=i/(count-1),theta=Math.PI*t;
    if(type==='spiral'){
      const r=.16*Math.sin(theta),phase=3*Math.PI*t;
      return {u:t,v:.36*Math.sin(theta)+r*Math.sin(phase)*Math.cos(theta),w:r*Math.cos(phase),contact:i===0||i===count-1};
    }
    return {u:t,v:.46*Math.sin(theta),w:0,contact:i===0||i===count-1};
  });
}
export function parseNominalCSV(text){
  const rows=text.trim().split(/\r?\n/).map(l=>l.trim()).filter(l=>l&&!l.startsWith('#'));
  if(rows.length<4)throw new Error('CSV needs header + at least 3 poses');
  const head=rows[0].split(',').map(v=>v.trim().toLowerCase());
  for(const col of ['u','v','w'])if(!head.includes(col))throw new Error('CSV missing '+col+' column');
  if(rows.length>3001)throw new Error('Maximum 3000 input poses');
  const result=rows.slice(1).map((line,i)=>{
    const fields=line.split(',');if(fields.length!==head.length)throw new Error('CSV row '+(i+2)+' has incorrect column count');
    const val=k=>Number(fields[head.indexOf(k)]);
    const u=val('u'),v=val('v'),w=val('w');
    if(![u,v,w].every(Number.isFinite))throw new Error('CSV contains nonnumeric xyz at line '+(i+2));
    const contact=head.includes('contact')?['1','true','yes'].includes(fields[head.indexOf('contact')].trim().toLowerCase()):i===0||i===rows.length-2;
    const p={u,v,w,contact};
    if(['ax','ay','az'].every(x=>head.includes(x))){p.axis=V(val('ax'),val('ay'),val('az'));if(![p.axis.x,p.axis.y,p.axis.z].every(Number.isFinite)||len(p.axis)<1e-8)throw new Error('Invalid axis at line '+(i+2));p.axis=norm(p.axis);}
    return p;
  });
  if(result.length<3||Math.hypot(result.at(-1).u-result[0].u,result.at(-1).v-result[0].v,result.at(-1).w-result[0].w)<1e-8)throw new Error('Start and end must differ');
  // Contact labels are authoritative; always preserve start/end as contact.
  result[0].contact=true;result.at(-1).contact=true;
  if(result.slice(1,-1).some(p=>p.contact))throw new Error('Lifted Skill CSV currently supports contact only at start/end; use All-contact for continuous contact');
  return result;
}
export function buildGuide(state,stroke,cutAngle,cutOffset,count=161,amplitude=.7){
  const a=rad(cutAngle),t=V(Math.cos(a),0,Math.sin(a)),side=V(-Math.sin(a),0,Math.cos(a)),pts=[];
  for(let i=0;i<count;i++){
    const u=i/(count-1),lateral=(.5-atStroke(stroke,u))*2*amplitude;
    const p=add(add(V(state.offsetX??0,0,state.offsetZ??0),mul(t,(u-.5)*5.2*(state.scale??1))),mul(side,cutOffset+lateral));
    const sm=sampleSurface(state,p.x,p.z);
    if(sm)pts.push({...sm,progress:u});
  }
  let arc=0;for(let i=0;i<pts.length;i++){if(i)arc+=len(sub(pts[i].point,pts[i-1].point));pts[i].s=arc;}
  return {points:pts,totalArc:arc,surface:state};
}
export function pointOnGuide(guide,t){
  const arr=guide.points;if(!arr.length)return null;
  const s=clamp(t,0,1)*guide.totalArc;let lo=0,hi=arr.length-1;
  while(lo<hi){const mid=(lo+hi)>>1;if(arr[mid].s<s)lo=mid+1;else hi=mid;}
  if(lo===0)return arr[0];const b=arr[lo],a=arr[lo-1],f=(s-a.s)/Math.max(1e-9,b.s-a.s);
  const approx=mix(a.point,b.point,f);
  const exact=sampleSurface(guide.surface,approx.x,approx.z);
  return {point:exact?.point??approx,normal:exact?.normal??norm(mix(a.normal,b.normal,f)),progress:t,s};
}
export function placementFrame(p0,p1,midNormal){
  const forward=norm(sub(p1,p0));
  let up=sub(midNormal,mul(forward,dot(midNormal,forward)));
  if(len(up)<1e-6)up=sub(V(0,1,0),mul(forward,dot(V(0,1,0),forward)));
  up=norm(up);if(dot(up,midNormal)<0)up=mul(up,-1);
  return {forward,up,side:norm(cross(forward,up)),origin:p0,scale:len(sub(p1,p0))};
}
export function placeNominal(skill,guide,repeats=1,tiltDeg=65,length=1.2,reverse=false){
  if(!skill?.length||guide.points.length<3)throw new Error('Guide and skill required');
  const out=[],groups=[];const maxN=clamp(Math.round(repeats),1,8),minGap=.01;
  const first=skill[0],last=skill.at(-1);
  const disp=sub(V(last.u,last.v,last.w),V(first.u,first.v,first.w));const span=len(disp);
  if(span<1e-6)throw new Error('Nominal start/end cannot coincide');
  // Source u/v/w is defined in the canonical guide-aligned frame; each primitive
  // is transformed by one similarity (not pointwise projected onto terrain).
  for(let k=0;k<maxN;k++){
    const t0=k/maxN+minGap/2,t1=(k+1)/maxN-minGap/2;
    const a=pointOnGuide(guide,t0),b=pointOnGuide(guide,t1),m=pointOnGuide(guide,(t0+t1)/2);
    const frame=placementFrame(a.point,b.point,m.normal),sim=frame.scale/span;
    if(!Number.isFinite(sim)||sim<=0)throw new Error('Invalid guide interval');
    // General canonical input: align source endpoint axis with guide chord.
    const sourceForward=norm(disp), sourceUp0=V(0,1,0);
    let sourceUp=sub(sourceUp0,mul(sourceForward,dot(sourceUp0,sourceForward)));
    if(len(sourceUp)<1e-7)sourceUp=sub(V(0,0,1),mul(sourceForward,sourceForward.z));
    sourceUp=norm(sourceUp);const sourceSide=norm(cross(sourceForward,sourceUp));
    const worldAxis=axis=>norm(add(add(mul(frame.forward,dot(axis,sourceForward)),mul(frame.up,dot(axis,sourceUp))),mul(frame.side,dot(axis,sourceSide))));
    const sg=reverse?-1:1;
    const defaultAxis=norm(add(mul(frame.forward,sg*Math.cos(rad(tiltDeg))),mul(frame.up,Math.sin(rad(tiltDeg)))));
    const frameGroup={...frame,start:a.point,end:b.point,scaleFactor:sim,from:k,contactIndices:[]};
    for(let i=0;i<skill.length;i++){
      const p=skill[i],local=sub(V(p.u,p.v,p.w),V(first.u,first.v,first.w));
      const rel=add(add(mul(frame.forward,dot(local,sourceForward)),mul(frame.up,dot(local,sourceUp))),mul(frame.side,dot(local,sourceSide)));
      const a0=add(frame.origin,mul(rel,sim)),contact=Boolean(p.contact)||i===0||i===skill.length-1;
      const axis=p.axis?worldAxis(p.axis):defaultAxis;
      out.push({a0,a1:add(a0,mul(axis,length)),axis,contact,passId:k,guide:k/maxN+i/Math.max(1,skill.length-1)/maxN,normal:frame.up});
      if(contact)frameGroup.contactIndices.push(out.length-1);
    }
    groups.push(frameGroup);
  }
  return {poses:out,groups};
}
export function placeContactGuide(guide,length,tiltDeg,reverse=false,a1Stroke=null,count=81){
  const contacts=Array.from({length:count},(_,i)=>pointOnGuide(guide,i/(count-1)));
  const poses=contacts.map((c,i)=>{
    const prev=contacts[Math.max(0,i-1)].point,next=contacts[Math.min(count-1,i+1)].point;
    const forward=norm(sub(next,prev)),tangent=norm(sub(forward,mul(c.normal,dot(forward,c.normal))));
    const alpha=a1Stroke?clamp(tiltDeg+(0.5-atStroke(a1Stroke,i/(count-1)))*90,5,88):tiltDeg;
    const axis=norm(add(mul(tangent,(reverse?-1:1)*Math.cos(rad(alpha))),mul(c.normal,Math.sin(rad(alpha)))));
    return {a0:c.point,a1:add(c.point,mul(axis,length)),axis,normal:c.normal,contact:true,passId:0,guide:i/(count-1)};
  });
  return {poses,groups:[]};
}