import {V,add,sub,mul,dot,cross,len,norm,mix,clamp,rad,angle,defaultProfile,defaultStroke,normalizeStroke,atStroke,smoothStroke,sampleSurface,worldFromLocal,getMesh,getCut,makeContactPath,makeTilted,rodMetrics,warpDemo,analyzePath,sketchToSkill,makeSkillExample,deformSkillWithHandles,parseNominalCSV,buildGuide,placeNominal,placeContactGuide} from './geometry.js';

const $ = id=>document.getElementById(id);
const stage=$('stage'),canvas=$('sceneCanvas'),ctx=canvas.getContext('2d',{alpha:false});
const sketch=$('drawCanvas'),dc=sketch.getContext('2d');
const state={surface:{type:'hill',profile:defaultProfile,offsetX:0,offsetZ:0,offsetY:0,rotation:0,scale:1},
 stroke:Array.from({length:70},(_,i)=>({x:i/69,y:.5})),baseNominal:makeSkillExample('half'),nominal:makeSkillExample('half'),editHandles:[{v:0,w:0},{v:0,w:0},{v:0,w:0}],brushTension:.35,skillName:'Half-circle',taskMode:'skill',a1Stroke:null,repeats:3,edit3D:false,selected:null,selectedManually:false,showConic:true,focusPose:true,preview:null,guide:null,cutAngle:0,cutOffset:0,amplitude:.7,rodLength:1.2,tiltAngle:25,coneAngle:40,reverse:false,geometryGain:1,conicGain:1,tensionGain:.6,bendingGain:.35,
 mode:'orbit',viewLocked:false,warpRequested:false,cam:{az:.70,el:.57,distance:14.8,target:V(0,.7,0)},mesh:null,cut:null,contacts:[],tilted:[],warped:[],pose:0,
 playing:false,playClock:0,showBoth:true,planeVisible:true,gridVisible:true,
 layers:{surface:true,plane:true,guide:true,nominal:true,tilt:true,warp:true},drag:null,dirty:true};
let W=800,H=580,dpr=1,drawType='profile',drawRaw=[],drawActive=false,toastTimer=null;
function toast(msg){const el=$('toast');el.textContent=msg;el.classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.classList.remove('show'),3300);}
function refreshPreview(){try{state.preview=state.taskMode==='skill'?placeNominal(state.nominal,state.guide,state.repeats,state.tiltAngle,state.rodLength,state.reverse):placeContactGuide(state.guide,state.rodLength,state.tiltAngle,state.reverse,state.a1Stroke);}catch(e){state.preview=null;}state.dirty=true;}
// Editable at every stage. Geometry changes keep the generated results alive.
// Tilted is rebuilt synchronously, while the more expensive Warp is debounced.
let warpTimer=null,warpGeneration=0;
function cancelQueuedWarp(){clearTimeout(warpTimer);warpGeneration++;}
function scheduleLiveWarp(){
 cancelQueuedWarp();
 if(!state.warpRequested||!state.tilted.length)return;
 const version=warpGeneration;
 $('sceneStatus').textContent='UPDATING';
 warpTimer=setTimeout(()=>{
   if(version!==warpGeneration)return;
   warp(false);
   updateUI();
 },125);
}
function rebuildAll(recreateMesh=false){
 const hadTilt=state.tilted.length>0,hadWarp=state.warpRequested;
 const oldPose=state.pose;
 if(recreateMesh||!state.mesh)state.mesh=getMesh(state.surface,41);
 state.cut=getCut(state.surface,state.cutAngle,state.cutOffset);
 state.guide=buildGuide(state.surface,state.stroke,state.cutAngle,state.cutOffset,181,state.amplitude);
 refreshPreview();
 if(hadTilt){
   generateTilted(false);
   state.pose=clamp(oldPose,0,Math.max(0,state.tilted.length-1));
   updateFrame();
   state.warpRequested=hadWarp;
   if(hadWarp)scheduleLiveWarp();
 }else{
   state.contacts=[];state.warped=[];
   updateMetric();updateFrame();
 }
 state.dirty=true;
}
function invalidate(recreateMesh=false){rebuildAll(recreateMesh);}
function updateGuide(){rebuildAll(false);}
function updateMetricsText(id,v){$(id).textContent=v;}
const editKnots=[.25,.5,.75];
function applyBrush(){state.nominal=deformSkillWithHandles(state.baseNominal,state.editHandles,state.brushTension);rebuildAll(false);updateUI();}
function getHandles(){
 if(!state.preview?.poses?.length||!state.preview.groups?.length)return [];
 const n=state.nominal.length;
 return state.preview.groups.flatMap((grp,k)=>editKnots.map((fraction,controlIndex)=>{
   const localIndex=Math.round(fraction*(n-1)),poseIndex=k*n+localIndex;
   return {passId:k,controlIndex,poseIndex,pose:state.preview.poses[poseIndex]};
 }));
}
function mostRelevantPose(){
 if(!state.tilted.length)return 0;
 let best=0,bestScore=-1;
 for(let i=0;i<state.tilted.length;i++){
   const p=state.tilted[i],m=rodMetrics(p,state.surface,state.rodLength);
   const improvement=state.warped.length?angle(p.axis,state.warped[i].axis)*4:0;
   const score=improvement+m.penetration*3;
   if(score>bestScore){best=i;bestScore=score;}
 }
 return best;
}
function updatePoseInspector(){
 const p=state.tilted[state.pose],w=state.warped[state.pose];
 $('poseInspectIndex').textContent=p?`POSE ${state.pose+1} / ${state.tilted.length}`:'—';
 $('coneBadge').textContent=state.coneAngle+'°';
 $('poseDelta').textContent=p&&w?`${(angle(p.axis,w.axis)*180/Math.PI).toFixed(2)}°`:'—';
 $('posePen').textContent=p&&w?`${rodMetrics(p,state.surface,state.rodLength).penetration.toFixed(3)} → ${rodMetrics(w,state.surface,state.rodLength).penetration.toFixed(3)} u`:'—';
 $('poseA1').textContent=p&&w?`${len(sub(p.a1,w.a1)).toFixed(3)} u`:'—';
}

function updateMetric(){
 const t=state.tilted.length?analyzePath(state.tilted,state.surface,state.rodLength):null;
 const w=state.warped.length?analyzePath(state.warped,state.surface,state.rodLength):null;
 updateMetricsText('metricPoses',state.tilted.length||'—');updateMetricsText('metricLength',state.rodLength.toFixed(2));
 updateMetricsText('metricTilt',t?t.maxPenetration.toFixed(3)+' u':'—');
 updateMetricsText('metricWarp',w?w.maxPenetration.toFixed(3)+' u':'—');
 updateMetricsText('metricCollision',t?(t.collisions+' → '+(w?w.collisions:'—')):'—');
 updateMetricsText('metricAngle',state.warped.length?Math.max(...state.warped.map(p=>p.deviationDeg)).toFixed(1)+'°':'—');
 const hint=$('metricHint'),pill=$('sceneStatus');
 if(!t){pill.textContent='READY';hint.textContent='Click Generate Tilted, then Warp to compare the results.';return;}
 if(!w){pill.textContent='TILTED';hint.textContent='Tilted generated. Click Warp to see how the tool avoids the surface.';return;}
 pill.textContent=w.collisions?'PARTIAL':'COMPLETE';
 hint.textContent=`Sampled penetration: ${t.collisions}/${t.count} Tilted poses → ${w.collisions}/${w.count} Warped poses. ${w.collisions?'The cone limit may prevent full clearance.':'No significant sampled penetration detected.'} This is not a certified collision-safety guarantee.`;
}
function updateFrame(){const max=Math.max(0,state.tilted.length-1,state.warped.length-1);$('frameSlider').max=max;$('frameSlider').value=clamp(state.pose,0,max);$('frameVal').textContent=`Pose ${String(state.pose+1).padStart(2,'0')} / ${Math.max(1,max+1)}`;updatePoseInspector();}
function generateTilted(announce=true){
 if(!state.guide?.points?.length){toast('Select a valid Guide first.');return false;}
 let result;
 try{result=state.taskMode==='skill'?placeNominal(state.nominal,state.guide,state.repeats,state.tiltAngle,state.rodLength,state.reverse):placeContactGuide(state.guide,state.rodLength,state.tiltAngle,state.reverse,state.a1Stroke);}catch(e){toast('Tilted failed: '+e.message);return false;}
 const oldPose=state.pose,oldManual=state.selectedManually,hadWarp=state.warpRequested;
 state.tilted=result.poses;state.contacts=state.tilted.map(p=>({point:p.a0,contact:p.contact}));
 state.warped=[];
 state.pose=announce?0:clamp(oldPose,0,Math.max(0,state.tilted.length-1));
 state.selectedManually=announce?false:oldManual;
 if(announce){state.warpRequested=false;cancelQueuedWarp();}
 else state.warpRequested=hadWarp;
 updateMetric();updateFrame();state.dirty=true;
 if(announce)toast(`Tilted: ${state.tilted.length} poses. Edit the Surface, Guide, Nominal or tool; results update live.`);
 return true;
}
function warp(announce=true){
 if(!state.tilted.length&&!generateTilted(false))return;
 cancelQueuedWarp();state.warpRequested=true;
 const previous=state.warped;
 const result=warpDemo(state.tilted,state.surface,state.rodLength,state.coneAngle,{geometry:state.geometryGain,conic:state.conicGain,tension:state.tensionGain,bending:state.bendingGain});
 state.warped=result.warped;
 if(announce&&!state.selectedManually)state.pose=mostRelevantPose();
 const n=Math.min(previous.length,state.warped.length);
 let changed=0,largest=0,largestI=0;
 for(let i=0;i<n;i++){
  const shift=angle(previous[i].axis,state.warped[i].axis)*180/Math.PI + len(sub(previous[i].a0,state.warped[i].a0))*15;
  if(shift>.01)changed++;
  if(shift>largest){largest=shift;largestI=i;}
 }
 if(!announce&&n&&!state.selectedManually&&largest>.01)state.pose=largestI;
 updateMetric();updateFrame();updateForceDiagram();state.dirty=true;
 if(n){$('sliderEffectHint').textContent=changed?`Since the last Warp: ${changed}/${n} poses changed (largest combined axis/position change: ${largest.toFixed(2)}). Showing the most affected pose.`:'No pose changed: the discrete search can have plateaus. Try a preset or select a colliding pose.';}
 else if(announce)$('sliderEffectHint').textContent='Showing a pose with a substantial correction. Change the balance weights or cone angle to compare it.';
 if(announce)toast(`Warp complete: ${state.warped.length} poses; ${result.failure} still have sampled penetration. Sliders remain live.`);
}
function updateEditorLock(){
 state.viewLocked=false;
 $('editModeHint').textContent='Always editable: Orbit rotates only the camera; Move / Rotate edits the surface; 3D edit drags three skill control points. Changes recompute generated results.';
}
function resize(){const r=stage.getBoundingClientRect();dpr=Math.min(2,window.devicePixelRatio||1);W=Math.max(240,r.width);H=Math.max(260,r.height);canvas.width=Math.round(W*dpr);canvas.height=Math.round(H*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);state.dirty=true;}
new ResizeObserver(resize).observe(stage);
function makeCamera(){const c=state.cam,t=c.target,cos=Math.cos(c.el);
 const eye=add(t,V(c.distance*cos*Math.sin(c.az),c.distance*Math.sin(c.el),c.distance*cos*Math.cos(c.az)));
 const f=norm(sub(t,eye)),r=norm(cross(f,V(0,1,0))),u=norm(cross(r,f)),focal=Math.min(W*1.18,H*1.85);
 return {eye,f,r,u,focal};
}
function proj(v,cam){const a=sub(v,cam.eye),z=dot(a,cam.f);if(z<.13)return null;const k=cam.focal/z;return {x:W*.5+dot(a,cam.r)*k,y:H*.52-dot(a,cam.u)*k,z};}
function segment3D(a,b,color,width=1,dash=[],opacity=1,cam=makeCamera()){const A=proj(a,cam),B=proj(b,cam);if(!A||!B)return;ctx.save();ctx.globalAlpha=opacity;ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineCap='round';ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);ctx.stroke();ctx.restore();}
function poly3D(points,color,width=2,alpha=1,cam=makeCamera(),dash=[]){
 if(points.length<2)return;
 ctx.save();ctx.strokeStyle=color;ctx.globalAlpha=alpha;ctx.lineWidth=width;ctx.lineJoin='round';ctx.lineCap='round';ctx.setLineDash(dash);ctx.beginPath();let active=false;
 for(const p of points){const xy=proj(p,cam);if(!xy){active=false;continue;}if(!active){ctx.moveTo(xy.x,xy.y);active=true;}else ctx.lineTo(xy.x,xy.y);}
 ctx.stroke();ctx.restore();
}
function circle3D(p,r,color,cam=makeCamera(),outline=null){const q=proj(p,cam);if(!q)return;ctx.beginPath();ctx.arc(q.x,q.y,Math.max(2.3,r*cam.focal/q.z),0,Math.PI*2);ctx.fillStyle=color;ctx.fill();if(outline){ctx.lineWidth=1.4;ctx.strokeStyle=outline;ctx.stroke();}}
function drawGrid(cam){
 for(let i=-8;i<=8;i++){
  const major=i%2===0;
  segment3D(V(i,0,-8),V(i,0,8),major?'#38516e':'#283b51',major?1:.7,[],major?.45:.23,cam);
  segment3D(V(-8,0,i),V(8,0,i),major?'#38516e':'#283b51',major?1:.7,[],major?.45:.23,cam);
 }
 segment3D(V(-.04,0,0),V(0,2.5,0),'#55cda8',2,[],.78,cam);
 segment3D(V(0,0,0),V(2.7,0,0),'#f09c89',2,[],.7,cam);
 segment3D(V(0,0,0),V(0,0,2.7),'#75a5ff',2,[],.7,cam);
}
function drawMesh(cam){
 if(!state.mesh||!state.layers.surface)return;
 const verts=state.mesh.verts, projected=verts.map(v=>proj(v,cam)),faces=[];
 for(const ids of state.mesh.faces){
   const [a,b,c]=ids,pa=projected[a],pb=projected[b],pc=projected[c];if(!pa||!pb||!pc)continue;
   const A=verts[a],B=verts[b],C=verts[c],normal=norm(cross(sub(B,A),sub(C,A)));
   const brightness=Math.max(.08,Math.abs(dot(normal,norm(V(.35,.87,.6)))));
   const y=(A.y+B.y+C.y)/3;
   faces.push({pa,pb,pc,depth:(pa.z+pb.z+pc.z)/3,brightness,y});
 }
 faces.sort((a,b)=>b.depth-a.depth);
 const isSaddle=state.surface.type==='saddle';
 for(const f of faces){
  const light=clamp(f.brightness*.66+.18,0,1);
  const elev=clamp((f.y+.4)/3.2,0,1);
  let rr=42+light*36+elev*27,gg=70+light*65+elev*33,bb=112+light*68+elev*44;
  if(isSaddle){rr+=22;gg-=9;bb+=7;}
  ctx.beginPath();ctx.moveTo(f.pa.x,f.pa.y);ctx.lineTo(f.pb.x,f.pb.y);ctx.lineTo(f.pc.x,f.pc.y);ctx.closePath();
  ctx.fillStyle=`rgb(${rr|0},${gg|0},${bb|0})`;ctx.fill();
  ctx.strokeStyle='rgba(128,173,213,0.095)';ctx.lineWidth=.55;ctx.stroke();
 }
}
function drawPlane(cam){
 if(!state.layers.plane||!state.planeVisible)return;
 const {tangent,side,center}=state.cut;
 const center2=add(center,V(0,.04,0));const a=add(center2,mul(tangent,-3.35)),b=add(center2,mul(tangent,3.35));
 const vertices=[add(a,V(0,-.6,0)),add(b,V(0,-.6,0)),add(b,V(0,3.7,0)),add(a,V(0,3.7,0))].map(v=>proj(v,cam));
 if(vertices.every(Boolean)){
  ctx.save();ctx.fillStyle='rgba(141,144,230,.105)';ctx.strokeStyle='rgba(166,167,234,.44)';ctx.lineWidth=1;ctx.setLineDash([6,6]);
  ctx.beginPath();ctx.moveTo(vertices[0].x,vertices[0].y);for(const p of vertices.slice(1))ctx.lineTo(p.x,p.y);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
 }
}
function rodDraw(pose,color,cam,main=false,compare=false){
 const pa=pose.a0,pb=pose.a1;
 const a=proj(pa,cam),b=proj(pb,cam);if(!a||!b)return;
 const metrics=rodMetrics(pose,state.surface,state.rodLength);
 ctx.save();ctx.lineCap='round';
 ctx.strokeStyle='rgba(6,12,22,.90)';ctx.lineWidth=main?12:6;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
 ctx.strokeStyle=color;ctx.lineWidth=main?6:2.2;ctx.globalAlpha=main?1:.48;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.globalAlpha=1;ctx.restore();
 if(main){
  if(metrics.collides){
    for(let k=1;k<=26;k++){
     const t=k/26,p=add(pa,mul(pose.axis,t*state.rodLength));const sf=sampleSurface(state.surface,p.x,p.z);
     if(sf&&p.y<sf.point.y-.01)circle3D(p,.055,'#ff595f',cam);
    }
  }
  circle3D(pa,.11,pose.contact?'#8bffbf':'#ffe2a4',cam,'#0e1626');circle3D(pb,.075,color,cam,'#0e1626');
  const q=proj(pa,cam);if(q){ctx.save();ctx.font='700 11px system-ui';ctx.fillStyle='#ffffff';ctx.fillText('A0',q.x+12,q.y+12);ctx.restore();}
 }
}
/** Selected Tilted tool's conic feasible directions, apex A0. The outward-only
 * half-space (red/grey when rejected) comes from the browser demo constraints.
 */
function drawSelectedCone(cam){
 if(!state.showConic||!state.tilted.length)return;
 const p=state.tilted[clamp(state.pose,0,state.tilted.length-1)],axis=p.axis;
 const beta=rad(state.coneAngle),L=state.rodLength;
 const n=sampleSurface(state.surface,p.a0.x,p.a0.z)?.normal??p.normal??V(0,1,0);
 let u=sub(n,mul(axis,dot(axis,n)));
 if(len(u)<1e-5){const seed=Math.abs(axis.y)<.9?V(0,1,0):V(1,0,0);u=sub(seed,mul(axis,dot(seed,axis)));}
 u=norm(u);const v=norm(cross(axis,u));
 const apex=proj(p.a0,cam);if(!apex)return;
 const rim=[],segments=36;
 for(let i=0;i<=segments;i++){
   const phi=2*Math.PI*i/segments;
   const a=norm(add(mul(axis,Math.cos(beta)),mul(add(mul(u,Math.cos(phi)),mul(v,Math.sin(phi))),Math.sin(beta))));
   rim.push({point:add(p.a0,mul(a,L)),valid:dot(sub(a,axis),n)>=-1e-9});
 }
 for(let i=0;i<segments;i++){
   const a=proj(rim[i].point,cam),b=proj(rim[i+1].point,cam);
   if(!a||!b)continue;
   const valid=rim[i].valid&&rim[i+1].valid;
   ctx.save();ctx.beginPath();ctx.moveTo(apex.x,apex.y);ctx.lineTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.closePath();
   ctx.fillStyle=valid?'rgba(107,221,245,.14)':'rgba(240,142,142,.038)';ctx.fill();ctx.restore();
   segment3D(rim[i].point,rim[i+1].point,valid?'#8addf7':'#df9c9c',valid?1.9:1,[...(valid?[]:[3,5])],valid?.88:.36,cam);
   if(i%6===0)segment3D(p.a0,rim[i].point,valid?'#69bccc':'#cc9292',.8,valid?[]:[3,5],valid?.33:.21,cam);
 }
 segment3D(p.a0,add(p.a0,mul(axis,L)),'#92dff7',2,[4,4],.85,cam);
 const apexLabel=proj(add(p.a0,mul(axis,L*.75)),cam);
 if(apexLabel){ctx.save();ctx.font='700 12px system-ui';ctx.fillStyle='#9fe7ff';ctx.fillText(`β ${state.coneAngle}°`,apexLabel.x+8,apexLabel.y-9);ctx.restore();}
}
// A visual-only Catmull-Rom brush stroke: smooths the rendered curve without
// altering the sampled A0 used for collision queries, transfers, or exports.
function brushCurve(points,steps=4){
 if(points.length<3)return points;
 const out=[];
 for(let i=0;i<points.length-1;i++){
  const a=points[Math.max(0,i-1)],b=points[i],c=points[i+1],d=points[Math.min(points.length-1,i+2)];
  for(let k=0;k<steps;k++){
   const t=k/steps,t2=t*t,t3=t2*t;
   out.push(V(...['x','y','z'].map(coord=>{
     const p0=a[coord],p1=b[coord],p2=c[coord],p3=d[coord];
     return .5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3);
   })));
  }
 }
 out.push(points.at(-1));return out;
}
function drawScene(){
 state.dirty=false;
 ctx.setTransform(dpr,0,0,dpr,0,0);
 ctx.clearRect(0,0,W,H);
 const grad=ctx.createRadialGradient(W*.53,H*.4,H*.04,W*.52,H*.45,W*.9);
 grad.addColorStop(0,'#182b40');grad.addColorStop(.65,'#0d1a2a');grad.addColorStop(1,'#08111f');ctx.fillStyle=grad;ctx.fillRect(0,0,W,H);
 const cam=makeCamera();if(state.gridVisible)drawGrid(cam);
 drawMesh(cam);drawPlane(cam);
 if(state.layers.guide&&state.guide?.points?.length)poly3D(state.guide.points.map(x=>add(x.point,V(0,.025,0))),'#f7c77d',3.0,.99,cam);
 if(state.layers.nominal){
   const poses=state.tilted.length?state.tilted:state.preview?.poses;
   if(poses?.length){
     const groups=new Map();for(const p of poses){if(!groups.has(p.passId))groups.set(p.passId,[]);groups.get(p.passId).push(p);}
     for(const arr of groups.values())poly3D(brushCurve(arr.map(p=>p.a0)),'#e3e8ff',2.4,.92,cam,[5,3]);
     for(const p of poses){if(p.contact)circle3D(p.a0,.055,'#8bffbf',cam,'#103128');}
     if(state.edit3D&&state.taskMode==='skill'){
       for(const h of getHandles()){
         const active=state.selected?.passId===h.passId&&state.selected?.controlIndex===h.controlIndex;
         circle3D(h.pose.a0,active?.15:.115,active?'#ffd68e':'#f7f2fa',cam,'#233c52');
       }
     }
   }
 }
 drawSelectedCone(cam);
 const tiltEnabled=state.layers.tilt&&state.tilted.length,warpEnabled=state.layers.warp&&state.warped.length;
 if(tiltEnabled){
  const arr=state.tilted;const step=7;
  if(!state.focusPose)for(let i=0;i<arr.length;i+=step)rodDraw(arr[i],'#ee9687',cam,false);
  poly3D(arr.map(p=>p.a1),'#ff9b89',1.6,.75,cam);
 }
 if(warpEnabled){
  const arr=state.warped;const step=7;
  if(!state.focusPose)for(let i=0;i<arr.length;i+=step)rodDraw(arr[i],'#65dce3',cam,false);
  poly3D(arr.map(p=>p.a1),'#65dce3',1.8,.90,cam);
 }
 const idx=clamp(state.pose,0,Math.max(0,state.tilted.length-1));
 if(tiltEnabled&&(!warpEnabled||state.showBoth))rodDraw(state.tilted[idx],'#ffa08b',cam,true);
 if(warpEnabled)rodDraw(state.warped[idx],'#75fff1',cam,true,true);
 ctx.save();ctx.textAlign='center';ctx.font='10px system-ui';ctx.fillStyle='#6989a5';ctx.fillText('Interactive 3D scene · arbitrary display units',W/2,H-18);ctx.restore();
}
function loop(now){
 if(state.playing&&state.tilted.length){
  if(now-state.playClock>95){state.pose=(state.pose+1)%state.tilted.length;state.playClock=now;updateFrame();state.dirty=true;}
 }
 if(state.dirty)drawScene();requestAnimationFrame(loop);
}
function updateUI(){
 updateForceDiagram();
 for(const [id,value,suffix] of [['xVal',state.surface.offsetX.toFixed(2),''],['zVal',state.surface.offsetZ.toFixed(2),''],['rotVal',state.surface.rotation.toFixed(0),'°'],['scaleVal',state.surface.scale.toFixed(2),'×'],['cutAngleVal',state.cutAngle,'°'],['cutOffsetVal',state.cutOffset.toFixed(2),''],['amplitudeVal',state.amplitude.toFixed(2),''],['repeatsVal',state.repeats,''],['lengthVal',state.rodLength.toFixed(2),''],['tiltVal',state.tiltAngle,'°'],['coneVal',state.coneAngle,'°']])$(id).textContent=value+suffix;
 $('brushTensionVal').textContent=state.brushTension.toFixed(2);
 $('sceneName').textContent={hill:'Complex hill',saddle:'Saddle surface',profile:'Extruded 2D profile'}[state.surface.type];
 $('surfaceOptions').querySelectorAll('button').forEach(b=>b.classList.toggle('active',b.dataset.surface===state.surface.type));
 updateMetric();paintStrokeThumbnail();$('nominalSummary').textContent=state.taskMode==='skill'?`Nominal: ${state.skillName} · ${state.nominal.length} samples · ${state.nominal.filter(p=>p.contact).length} contact points / primitive`:'All-contact: A0 follows Guide, A1 uses fixed rod length';
}
function paintStrokeThumbnail(){const el=$('smallStroke'),c=el.getContext('2d');c.clearRect(0,0,el.width,el.height);c.fillStyle='#101d2a';c.fillRect(0,0,el.width,el.height);c.strokeStyle='#294359';c.lineWidth=1;c.beginPath();c.moveTo(0,el.height*.5);c.lineTo(el.width,el.height*.5);c.stroke();
 c.strokeStyle='#f6cc8b';c.lineWidth=2.3;c.beginPath();for(let i=0;i<90;i++){const x=i/89,p={x:12+x*(el.width-24),y:8+atStroke(state.stroke,x)*(el.height-20)};i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y);}c.stroke();}
function applyTerrain(type){state.surface.type=type;invalidate(true);updateUI();toast(type==='profile'?'Switched to your extruded profile.':'Loaded the '+type+' preset surface.');}
$('surfaceOptions').addEventListener('click',e=>{const b=e.target.closest('button[data-surface]');if(b)applyTerrain(b.dataset.surface);});
for(const [id,prop,field,regen] of [['surfaceX','offsetX','surface',true],['surfaceZ','offsetZ','surface',true],['surfaceRot','rotation','surface',true],['surfaceScale','scale','surface',true],['cutAngle','cutAngle','root',false],['cutOffset','cutOffset','root',false],['strokeAmp','amplitude','root',false],['repeats','repeats','root',false]]){
 $(id).addEventListener('input',e=>{const val=Number(e.target.value);if(field==='surface')state.surface[prop]=val;else state[prop]=val;invalidate(regen);updateUI();});
}
// Tool and surface settings remain live even after Generate Tilted/Warp.
function recomputeToolLive(){
 if(state.tilted.length){
   const old=state.pose,hadWarp=state.warpRequested;
   generateTilted(false);state.pose=clamp(old,0,state.tilted.length-1);
   state.warpRequested=hadWarp;
   if(hadWarp)scheduleLiveWarp();
   updateFrame();state.dirty=true;
 }else refreshPreview();
}
for(const [id,prop] of [['rodLength','rodLength'],['tiltAngle','tiltAngle']]){
 $(id).addEventListener('input',e=>{state[prop]=Number(e.target.value);recomputeToolLive();updateUI();});
}
function updateForceDiagram(){
 for(const [out,val,unit] of [['geometryVal',state.geometryGain,'×'],['conicVal',state.conicGain,'×'],['tensionVal',state.tensionGain,'×'],['bendingVal',state.bendingGain,'×']]) $(out).textContent=val.toFixed(2)+unit;
 $('coneVal').textContent=state.coneAngle+'°';
 // Each diagram is a schematic. These are visualization lengths, not solved physical forces.
 $('forceGeo').setAttribute('y2',String(58-38*Math.min(1,state.geometryGain/4)));
 // Fan is a sector around the upward tilted axis; the explicit A0 apex never moves.
 const beta=state.coneAngle*Math.PI/180,R=132,cx=150,cy=164;
 const lx=cx-R*Math.sin(beta),rx=cx+R*Math.sin(beta),y=cy-R*Math.cos(beta);
 const fmt=n=>n.toFixed(2);
 const left=[fmt(lx),fmt(y)],right=[fmt(rx),fmt(y)];
 $('conicFan').setAttribute('d',`M${cx} ${cy} L${left.join(' ')} A${R} ${R} 0 0 1 ${right.join(' ')} Z`);
 for(const [id,x] of [['conicLeft',lx],['conicRight',rx]]){$(id).setAttribute('x2',fmt(x));$(id).setAttribute('y2',fmt(y));}
 const betaArcR=30,bx=cx+betaArcR*Math.sin(beta),by=cy-betaArcR*Math.cos(beta);
 $('coneBetaArc').setAttribute('d',`M${cx} ${cy-betaArcR} A${betaArcR} ${betaArcR} 0 0 1 ${fmt(bx)} ${fmt(by)}`);
 $('coneBetaLabel').textContent=`β ${state.coneAngle}°`;
 $('coneBetaLabel').setAttribute('x',fmt(Math.min(231,cx+40*Math.sin(beta/2)+7)));
 $('coneBetaLabel').setAttribute('y',fmt(Math.max(110,cy-40*Math.cos(beta/2))));
 // Higher Conic restoring strength increases the arrow pointing back toward the cone's central axis.
 const conicArrowEnd=215-58*Math.min(1,state.conicGain/4);
 $('forceCone').setAttribute('x2',fmt(conicArrowEnd));
 $('forceCone').style.opacity=state.conicGain===0?'.20':'1';
 $('forceConeLabel').setAttribute('x',fmt(Math.max(151,conicArrowEnd-8)));
 // Visually tug an initially semicircular stroke into a tauter (shallower) arc.
 const tight=Math.min(1,state.tensionGain/3),controlY=4+103*tight;
 $('ribbonArc').setAttribute('d',`M36 121 C36 ${fmt(controlY)} 264 ${fmt(controlY)} 264 121`);
 const pull=5+17*tight;
 $('ribbonPullLeft').setAttribute('x2',fmt(36-pull));
 $('ribbonPullRight').setAttribute('x2',fmt(264+pull));
 $('ribbonShapeLabel').textContent=tight<.20?'Loose arch':tight>.74?'Taut arch':'Moderate tension';
 $('ribbonShapeLabel').setAttribute('y',fmt(108-21*(1-tight)));
 $('balanceCaption').textContent=state.warpRequested?'Warp updates shortly after you change a slider. Inspect Selected Pose to compare changes; some poses stay unchanged because the search is discrete.':'After Warp, changing these weights recomputes the result; inspect the selected pose above.';
 updatePoseInspector();
}
for(const [id,prop] of [['coneAngle','coneAngle'],['geometryGain','geometryGain'],['conicGain','conicGain'],['tensionGain','tensionGain'],['bendingGain','bendingGain']]){
 $(id).addEventListener('input',e=>{
  state[prop]=Number(e.target.value);
  if(state.warpRequested&&state.tilted.length)scheduleLiveWarp();
  updateUI();
 });
}
$('showConic').addEventListener('change',e=>{state.showConic=e.target.checked;state.dirty=true;});
$('focusPose').addEventListener('change',e=>{state.focusPose=e.target.checked;state.dirty=true;});
for(const [id,values] of [['presetAvoid',{geometry:4,conic:.05,tension:0,bending:0}],['presetPreserve',{geometry:.1,conic:4,tension:3,bending:3}]]){
 $(id).addEventListener('click',()=>{
   state.geometryGain=values.geometry;state.conicGain=values.conic;state.tensionGain=values.tension;state.bendingGain=values.bending;
   for(const [field,value] of [['geometryGain',values.geometry],['conicGain',values.conic],['tensionGain',values.tension],['bendingGain',values.bending]])$(field).value=value;
   if(state.tilted.length){state.warpRequested=true;warp(false);}
   state.pose=mostRelevantPose();state.selectedManually=false;updateFrame();updateUI();
   $('sliderEffectHint').textContent=id==='presetAvoid'?'Avoid collision preset: strong Geometry, weak restoring. Compare the selected pose angle and penetration.':'Preserve skill preset: strong Conic and Ribbon. Some collisions may remain.';
 });
}
$('reverseTool').addEventListener('change',e=>{state.reverse=e.target.checked;recomputeToolLive();updateUI();});
$('btnGuide').addEventListener('click',()=>{updateGuide();toast(`Guide ready: ${state.guide.points.length} samples. Nominal is independent of the Guide.`);});
$('taskModes').addEventListener('click',e=>{const b=e.target.closest('[data-task]');if(!b)return;state.taskMode=b.dataset.task;$('taskModes').querySelectorAll('button').forEach(x=>x.classList.toggle('active',x===b));$('liftSkillControls').hidden=state.taskMode!=='skill';$('contactSkillControls').hidden=state.taskMode!=='contact';state.edit3D=false;$('btnNominal3D').classList.remove('active');invalidate();updateUI();});
for(const [id,type,name] of [['btnHalf','half','Half-circle'],['btnSpiral','spiral','3D Spiral']])$(id).addEventListener('click',()=>{state.baseNominal=makeSkillExample(type);state.editHandles=[{v:0,w:0},{v:0,w:0},{v:0,w:0}];state.nominal=deformSkillWithHandles(state.baseNominal,state.editHandles,state.brushTension);state.skillName=name;invalidate();updateUI();toast(name+' nominal ready: interior A0 may lift; two anchors.');});
$('btnImportCSV').addEventListener('click',()=>$('fileImport').click());
$('fileImport').addEventListener('change',async e=>{const file=e.target.files?.[0];if(!file)return;try{if(file.size>250000)throw new Error('CSV exceeds 250 KB');state.baseNominal=parseNominalCSV(await file.text());state.editHandles=[{v:0,w:0},{v:0,w:0},{v:0,w:0}];state.nominal=deformSkillWithHandles(state.baseNominal,state.editHandles,state.brushTension);state.skillName=file.name;invalidate();updateUI();toast(`CSV imported: ${state.nominal.length} ordered poses.`);}catch(err){toast('CSV error: '+err.message);}finally{e.target.value='';}});
$('brushTension').addEventListener('input',e=>{state.brushTension=Number(e.target.value);applyBrush();});
$('btnNominal3D').addEventListener('click',()=>{if(state.taskMode!=='skill')return;state.edit3D=!state.edit3D;$('btnNominal3D').classList.toggle('active',state.edit3D);setMode('orbit');$('skillEditHint').textContent=state.edit3D?'3D editing: just three control points per repeated pass. Changes apply to all passes using a C¹-smooth curve. Contact endpoints remain fixed.':'Click 3D edit to adjust control points, or drag empty space to orbit the camera.';state.dirty=true;});
$('btnTilted').addEventListener('click',()=>generateTilted());
$('btnWarp').addEventListener('click',()=>warp());
$('frameSlider').addEventListener('input',e=>{state.pose=Number(e.target.value);state.selectedManually=true;updateFrame();state.dirty=true;});
$('btnPlay').addEventListener('click',()=>{if(!state.tilted.length){toast('Generate Tilted first.');return;}state.playing=!state.playing;state.playClock=performance.now();$('btnPlay').textContent=state.playing?'❚❚':'▶';});
$('showBoth').addEventListener('change',e=>{state.showBoth=e.target.checked;state.dirty=true;});
$('btnHome').addEventListener('click',()=>{state.cam={az:.70,el:.57,distance:14.8,target:V(0,.7,0)};state.dirty=true;});
$('btnGrid').addEventListener('click',e=>{state.gridVisible=!state.gridVisible;e.currentTarget.classList.toggle('on',state.gridVisible);state.dirty=true;});
$('btnPlane').addEventListener('click',e=>{state.planeVisible=!state.planeVisible;e.currentTarget.classList.toggle('on',state.planeVisible);state.dirty=true;});
for(const [id,key] of [['layerSurface','surface'],['layerPlane','plane'],['layerGuide','guide'],['layerNominal','nominal'],['layerTilt','tilt'],['layerWarp','warp']])$(id).addEventListener('change',e=>{state.layers[key]=e.target.checked;state.dirty=true;});
function setMode(mode){
 state.mode=mode;
 if(mode!=='orbit'&&state.edit3D){state.edit3D=false;$('btnNominal3D').classList.remove('active');}
 for(const [id,m] of [['btnModeOrbit','orbit'],['btnModeMove','move'],['btnModeRotate','rotate']])$(id).classList.toggle('active',mode===m);
 $('stageHelp').textContent={orbit:'DRAG TO ORBIT · CLICK ROD TO INSPECT CONIC · SCROLL TO ZOOM',move:'MOVE MODE · DRAG TO MOVE SURFACE · SELECT ORBIT TO VIEW',rotate:'ROTATE MODE · DRAG TO ROTATE SURFACE · SELECT ORBIT TO VIEW'}[mode];
 state.dirty=true;
}
for(const [id,mode] of [['btnModeOrbit','orbit'],['btnModeMove','move'],['btnModeRotate','rotate']])$(id).addEventListener('click',()=>setMode(mode));
canvas.addEventListener('pointerdown',e=>{
 canvas.setPointerCapture(e.pointerId);state.drag={x:e.clientX,y:e.clientY,button:e.button,shift:e.shiftKey};
 if(state.edit3D&&state.taskMode==='skill'&&!e.shiftKey){
   const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top,cam=makeCamera();
   let best=19,chosen=null;
   for(const h of getHandles()){
     const xy=proj(h.pose.a0,cam);if(!xy)continue;
     const dist=Math.hypot(x-xy.x,y-xy.y);
     if(dist<best){best=dist;chosen=h;}
   }
   if(chosen){state.selected={passId:chosen.passId,controlIndex:chosen.controlIndex};state.drag.handle=chosen;state.dirty=true;}
 }
 if(!state.drag.handle&&state.tilted.length&&!e.shiftKey&&state.mode==='orbit'){
   const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top,cam=makeCamera();
   let best=13,picked=-1;
   const closest=(a,b)=>{const vx=b.x-a.x,vy=b.y-a.y,t=clamp(((x-a.x)*vx+(y-a.y)*vy)/(vx*vx+vy*vy+1e-8),0,1);return Math.hypot(x-a.x-vx*t,y-a.y-vy*t);};
   for(let i=0;i<state.tilted.length;i++){
     const pose=state.tilted[i],a=proj(pose.a0,cam),b=proj(pose.a1,cam);
     if(!a||!b)continue;
     const dist=Math.min(Math.hypot(x-a.x,y-a.y),closest(a,b));
     if(dist<best){best=dist;picked=i;}
   }
   if(picked>=0)state.drag.posePick=picked;
 }
 canvas.classList.add('dragging');
});
canvas.addEventListener('pointermove',e=>{
 const d=state.drag;if(!d)return;const dx=e.clientX-d.x,dy=e.clientY-d.y;d.x=e.clientX;d.y=e.clientY;
 const cam=state.cam;
 if(d.handle&&state.preview?.groups?.length){d.moved=true;
   const h=d.handle,grp=state.preview.groups[h.passId],pose=state.preview.poses[h.poseIndex],camera=makeCamera();
   const p=proj(pose.a0,camera),q1=proj(add(pose.a0,mul(grp.up,grp.scaleFactor)),camera),q2=proj(add(pose.a0,mul(grp.side,grp.scaleFactor)),camera);
   if(p&&q1&&q2){const ux=q1.x-p.x,uy=q1.y-p.y,vx=q2.x-p.x,vy=q2.y-p.y,det=ux*vy-uy*vx;
     const dv=Math.abs(det)>1e-3?(dx*vy-dy*vx)/det:-dy*.008;
     const dw=Math.abs(det)>1e-3?(ux*dy-uy*dx)/det:dx*.008;
     const control=state.editHandles[h.controlIndex];
     control.v=clamp(control.v+dv,-1.4,1.8);
     control.w=clamp(control.w+dw,-1.7,1.7);
     applyBrush();
   }
   return;
 }
 if(Math.abs(dx)+Math.abs(dy)>3){d.moved=true;d.posePick=undefined;}
 if(state.mode==='move'){
  const scale=.018*cam.distance/14.8;
  state.surface.offsetX=clamp(state.surface.offsetX+(dx*Math.cos(cam.az)+dy*Math.sin(cam.az))*scale,-2.8,2.8);
  state.surface.offsetZ=clamp(state.surface.offsetZ+(-dx*Math.sin(cam.az)+dy*Math.cos(cam.az))*scale,-2.8,2.8);
  $('surfaceX').value=state.surface.offsetX;$('surfaceZ').value=state.surface.offsetZ;invalidate(true);updateUI();
 }else if(state.mode==='rotate'){
  state.surface.rotation=clamp(state.surface.rotation+dx*.35,-180,180);$('surfaceRot').value=state.surface.rotation;invalidate(true);updateUI();
 }else if(e.shiftKey||d.shift){const basis=makeCamera();cam.target=add(cam.target,add(mul(basis.r,-dx*.012*cam.distance/14),mul(basis.u,dy*.012*cam.distance/14)));state.dirty=true;
 }else{cam.az-=dx*.006;cam.el=clamp(cam.el+dy*.005,.15,1.43);state.dirty=true;}
});
function stopDrag(){
 const d=state.drag;
 if(d&&d.posePick!==undefined&&!d.moved&&!d.handle){state.pose=d.posePick;state.selectedManually=true;state.playing=false;$('btnPlay').textContent='▶';updateFrame();state.dirty=true;}
 state.drag=null;canvas.classList.remove('dragging');
}
canvas.addEventListener('pointerup',stopDrag);canvas.addEventListener('pointercancel',stopDrag);
canvas.addEventListener('wheel',e=>{e.preventDefault();state.cam.distance=clamp(state.cam.distance*Math.exp(e.deltaY*.001),5.5,30);state.dirty=true;},{passive:false});
function drawingCanvas(){
 dc.clearRect(0,0,sketch.width,sketch.height);dc.fillStyle='#0d1929';dc.fillRect(0,0,sketch.width,sketch.height);
 dc.lineWidth=1;dc.strokeStyle='#263b50';for(let x=0;x<=sketch.width;x+=sketch.width/10){dc.beginPath();dc.moveTo(x,0);dc.lineTo(x,sketch.height);dc.stroke();}for(let y=0;y<=sketch.height;y+=sketch.height/6){dc.beginPath();dc.moveTo(0,y);dc.lineTo(sketch.width,y);dc.stroke();}
 let points;
 if(drawRaw.length>1)points=drawRaw;
 else if(drawType==='profile')points=state.surface.profile;
 else if(drawType==='guide')points=state.stroke;
 else if(drawType==='a1')points=state.a1Stroke??[{x:0,y:.5},{x:1,y:.5}];
 else points=state.nominal.map(p=>({x:.08+.84*p.u,y:.76-.6*p.v}));
 if(points.length>1){dc.lineWidth=4;dc.lineJoin='round';dc.lineCap='round';dc.strokeStyle=drawType==='profile'?'#73dee6':drawType==='trajectory'?'#e2e8ff':'#f5c079';dc.beginPath();for(let i=0;i<points.length;i++){const p=points[i],x=p.x*sketch.width,y=p.y*sketch.height;i?dc.lineTo(x,y):dc.moveTo(x,y);}dc.stroke();}
 if(drawType==='trajectory'){
  for(const [text,x] of [['START',.08],['END',.92]]){
   const px=x*sketch.width,py=.76*sketch.height;dc.beginPath();dc.arc(px,py,9,0,Math.PI*2);dc.fillStyle='#7af0a9';dc.fill();dc.fillStyle='#c4f4dd';dc.font='bold 13px system-ui';dc.fillText(text,px+(x<.5?14:-43),py+24);
  }
 }
 dc.fillStyle='#89a7be';dc.font='12px system-ui';dc.fillText(({profile:'SURFACE HEIGHT',guide:'GUIDE LATERAL OFFSET',trajectory:'SKILL LIFT (A0 FREE BETWEEN CONTACT FEET)',a1:'A1 TOOL-ANGLE PROFILE'})[drawType],16,23);
}
function openModal(type){drawType=type;drawRaw=[];
 const labels={profile:'Draw 2D surface profile',guide:'Draw guide A0 path',trajectory:'Draw 2D Nominal Skill (A0)',a1:'Draw A1 angle profile'};
 const helps={profile:'Draw a left-to-right surface height profile. It is extruded along Z.',guide:'Sketch lateral offsets along the Guide. This path stays on the surface; it is not the Nominal Skill.',trajectory:'Sketch A0 in a plane. Green START / END anchors indicate contact. Intermediate points may lift or loop. Click 3D edit afterwards to add depth and build a spiral.',a1:'In all-contact mode, horizontal position follows the Guide and vertical position adjusts the A1 angle relative to the surface tangent (rod length is fixed).'};
 $('modalTitle').textContent=labels[type];$('modalHelp').textContent=helps[type];$('drawStatus').textContent='Draw from START to END';$('drawModal').hidden=false;drawingCanvas();
}
$('btnDrawSurface').addEventListener('click',()=>openModal('profile'));
$('btnDrawGuide').addEventListener('click',()=>openModal('guide'));
$('btnDrawTraj').addEventListener('click',()=>openModal('trajectory'));
$('btnDrawA1').addEventListener('click',()=>openModal('a1'));
$('btnCloseModal').addEventListener('click',()=>$('drawModal').hidden=true);
$('drawModal').addEventListener('click',e=>{if(e.target===$('drawModal'))$('drawModal').hidden=true;});
window.addEventListener('keydown',e=>{if(e.key==='Escape')$('drawModal').hidden=true;});
function toSketchPt(e){const r=sketch.getBoundingClientRect();return {x:clamp((e.clientX-r.left)/r.width,0,1),y:clamp((e.clientY-r.top)/r.height,0,1)};}
sketch.addEventListener('pointerdown',e=>{drawRaw=[toSketchPt(e)];drawActive=true;sketch.setPointerCapture(e.pointerId);drawingCanvas();});
sketch.addEventListener('pointermove',e=>{if(!drawActive)return;const p=toSketchPt(e),last=drawRaw.at(-1);if(Math.abs(p.x-last.x)+Math.abs(p.y-last.y)>.002){drawRaw.push(p);drawingCanvas();}});
sketch.addEventListener('pointerup',()=>{drawActive=false;$('drawStatus').textContent=`${drawRaw.length} ordered points drawn`;});
sketch.addEventListener('pointercancel',()=>drawActive=false);
$('btnDrawClear').addEventListener('click',()=>{drawRaw=[];drawingCanvas();$('drawStatus').textContent='Cleared.';});
$('btnDrawDefault').addEventListener('click',()=>{drawRaw=drawType==='profile'?defaultProfile.map(p=>({...p})):drawType==='trajectory'?makeSkillExample('half').map(p=>({x:.08+.84*p.u,y:.76-.6*p.v})):Array.from({length:80},(_,i)=>({x:i/79,y:.5}));drawingCanvas();});
$('btnDrawApply').addEventListener('click',()=>{
 try{
   if(drawType==='trajectory'){
    if(drawRaw.length<3)throw new Error('Draw a stroke with START on left and END on right');
    state.baseNominal=sketchToSkill(drawRaw,81);state.editHandles=[{v:0,w:0},{v:0,w:0},{v:0,w:0}];state.nominal=deformSkillWithHandles(state.baseNominal,state.editHandles,state.brushTension);state.skillName='Freehand 2D';
   }else{
    const base=drawType==='profile'?state.surface.profile:drawType==='guide'?state.stroke:state.a1Stroke??[{x:0,y:.5},{x:1,y:.5}];
    const pts=normalizeStroke(drawRaw.length>1?drawRaw:base);
    if(!pts)throw new Error('Draw a longer stroke');
    const result=smoothStroke(pts,100,drawType==='profile'?.045:.013);
    if(drawType==='profile'){state.surface.profile=result;state.surface.type='profile';invalidate(true);}
    else if(drawType==='guide')state.stroke=result;
    else state.a1Stroke=result;
   }
   invalidate();$('drawModal').hidden=true;updateUI();toast(({profile:'Surface updated',guide:'Guide updated (Nominal unchanged)',trajectory:'Nominal skill created; click 3D edit to lift/drag lateral points',a1:'A1 direction profile applied'})[drawType]);
 }catch(err){$('drawStatus').textContent=err.message;}
});
function exportProject(){
 const data={format:'SPGA-Interactive-Demo-v5',notice:'Browser demo, not the archived research SPGA-M2 solver.',units:'arbitrary display units',surface:state.surface,guideStroke:state.stroke,guide:state.guide.points.map(c=>c.point),nominal:state.nominal,brushControls:{handles:state.editHandles,tension:state.brushTension},nominalSkillName:state.skillName,taskMode:state.taskMode,repeats:state.repeats,contactMask:state.tilted.map(p=>p.contact),cut:{angleDeg:state.cutAngle,offset:state.cutOffset,amplitude:state.amplitude},tool:{length:state.rodLength,angleFromSurfaceDeg:state.tiltAngle,coneHalfAngleDeg:state.coneAngle,reverse:state.reverse},demoBalance:{geometryGain:state.geometryGain,conicGain:state.conicGain,ribbonTensionGain:state.tensionGain,ribbonBendingGain:state.bendingGain},tilted:state.tilted.map(p=>({A0:p.a0,A1:p.a1,contact:p.contact,passId:p.passId})),warped:state.warped.map(p=>({A0:p.a0,A1:p.a1,contact:p.contact,passId:p.passId})),metrics:{tilted:analyzePath(state.tilted,state.surface,state.rodLength),warped:analyzePath(state.warped,state.surface,state.rodLength)}};
 const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='spga_interactive_demo.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2500);toast('Exported surface settings, trajectories and Tilted / Warped poses (JSON).');
}
$('btnExport').addEventListener('click',exportProject);
$('btnReset').addEventListener('click',()=>{cancelQueuedWarp();state.warpRequested=false;state.tilted=[];state.warped=[];Object.assign(state.surface,{type:'hill',profile:defaultProfile,offsetX:0,offsetZ:0,offsetY:0,rotation:0,scale:1});Object.assign(state,{stroke:Array.from({length:70},(_,i)=>({x:i/69,y:.5})),baseNominal:makeSkillExample('half'),nominal:makeSkillExample('half'),editHandles:[{v:0,w:0},{v:0,w:0},{v:0,w:0}],brushTension:.35,selected:null,selectedManually:false,showConic:true,skillName:'Half-circle',taskMode:'skill',repeats:3,a1Stroke:null,edit3D:false,cutAngle:0,cutOffset:0,amplitude:.7,rodLength:1.2,tiltAngle:25,coneAngle:40,reverse:false,geometryGain:1,conicGain:1,tensionGain:.6,bendingGain:.35,cam:{az:.70,el:.57,distance:14.8,target:V(0,.7,0)}});
 for(const [id,n] of [['surfaceX',0],['surfaceZ',0],['surfaceRot',0],['surfaceScale',1],['cutAngle',0],['cutOffset',0],['strokeAmp',.7],['repeats',3],['rodLength',1.2],['tiltAngle',25],['coneAngle',40],['geometryGain',1],['conicGain',1],['tensionGain',.6],['bendingGain',.35],['brushTension',.35]])$(id).value=n;
 $('showConic').checked=true;$('focusPose').checked=true;state.focusPose=true;$('reverseTool').checked=false;$('taskModes').querySelectorAll('button').forEach(x=>x.classList.toggle('active',x.dataset.task==='skill'));$('liftSkillControls').hidden=false;$('contactSkillControls').hidden=true;$('btnNominal3D').classList.remove('active');invalidate(true);updateEditorLock();setMode('orbit');updateUI();toast('Restored the default Hill scene. Editing stays live, even after generating trajectories.');});

// Non-blocking onboarding: highlights workflow controls while keeping the scene interactive.
const tutorialSteps=[
 {target:'tutorialSurface',title:'1 · Create your surface',text:'Start with Hill or Saddle, or click Draw 2D profile to make your own extruded surface. Use Orbit to inspect it and Move / Rotate only when you want to change its placement.',tip:'Try it: select Saddle, then switch back to Hill. Your current scene will not be reset by this tutorial.'},
 {target:'tutorialGuide',title:'2 · Choose a Guide curve',text:'The yellow Guide is a placement path on the surface. Adjust Cut angle and Plane offset, then click Select cut guide. Draw guide A0 lets you sketch an alternative surface-following guide.',tip:'The Guide is NOT the Nominal Skill: it tells SPGA where to place that skill.'},
 {target:'tutorialNominal',title:'3 · Draw your Nominal Skill',text:'Choose Lifted skill and draw A0, use a Half-circle / 3D spiral preset, or import a CSV. START and END are contact anchors; intermediate points can lift. Click 3D edit to drag the three smooth-curve control points.',tip:'For full contact, choose All-contact. A0 follows Guide and you can sketch the A1 orientation.'},
 {target:'btnTilted',title:'4 · Generate Tilted',text:'Choose Tool length and Tool angle vs surface in the Tool & Warping card, then click Generate Tilted. The nominal shape is placed along the Guide without warping its interior points onto the surface.',tip:'0° points forward along the surface tangent, 90° points along the outward normal, and 180° points backward. Tool settings remain live.'},
 {target:'btnWarp',title:'5 · Apply Warp',text:'Click Warp to run the browser demonstration of outward-only geometry correction, cone-limited tool rotation and ribbon-style preservation.',tip:'Some Tilted poses may penetrate the surface; Warp tries to reduce those penetrations while keeping contact anchors and tool length fixed.'},
 {target:'tutorialPlayback',title:'6 · Inspect a pose and its cone',text:'Use Play or drag the pose slider, or click a rod in the scene. Enable Show selected conic to visualize the selected pose’s feasible cone. Compare the orange Tilted rod with the cyan Warped rod.',tip:'Use Focus one pose when other tool instances make the cone hard to see.'},
 {target:'tutorialBalance',title:'7 · Explore the force balance',text:'Change Geometry, Cone half-angle, Conic restoring, Ribbon tension and Ribbon bending. Once Warp is generated, the browser recomputes it as you move the sliders.',tip:'Watch the large cone: higher Fᶜ lengthens the arrow toward its axis; β widens its range. The semicircle flattens with more ribbon tension. These are schematic visual cues, not measured forces or literal path deformation.'},
];
let tutorialIndex=-1;
const tutorialConfirm=$('tutorialConfirm'),tutorialCoach=$('tutorialCoach'),tutorialShade=$('tutorialShade');
function showTutorialConfirmation(){
 if(tutorialIndex>=0)stopTutorial();
 tutorialConfirm.hidden=false;
 $('tutorialStart').focus();
}
function placeTutorialCoach(){
 if(tutorialIndex<0)return;
 const t=$(tutorialSteps[tutorialIndex].target);
 if(!t)return;
 const r=t.getBoundingClientRect();
 const width=tutorialCoach.offsetWidth||324;
 const height=tutorialCoach.offsetHeight||300;
 let left=r.right+16,top=r.top+Math.min(20,Math.max(0,r.height-height));
 if(left+width>window.innerWidth-12)left=r.left-width-16;
 if(left<12)left=Math.max(12,(window.innerWidth-width)/2);
 top=clamp(top,12,Math.max(12,window.innerHeight-height-12));
 if(window.innerWidth<760){left=Math.max(12,(window.innerWidth-width)/2);top=Math.max(12,window.innerHeight-height-14);}
 tutorialCoach.style.left=left+'px';tutorialCoach.style.top=top+'px';
}
function renderTutorialStep(){
 document.querySelectorAll('.tutorial-focus').forEach(el=>el.classList.remove('tutorial-focus'));
 if(tutorialIndex<0)return;
 const item=tutorialSteps[tutorialIndex],node=$(item.target);
 if(!node)return;
 node.scrollIntoView({block:'center',inline:'nearest',behavior:'instant'});
 node.classList.add('tutorial-focus');
 $('tutorialProgress').textContent=`STEP ${tutorialIndex+1} OF ${tutorialSteps.length}`;
 $('tutorialStepTitle').textContent=item.title;
 $('tutorialStepText').textContent=item.text;
 $('tutorialStepTip').textContent=item.tip;
 $('tutorialBack').disabled=tutorialIndex===0;
 $('tutorialNext').textContent=tutorialIndex===tutorialSteps.length-1?'Finish ✓':'Next →';
 $('tutorialDots').innerHTML=tutorialSteps.map((_,i)=>`<span class="${i===tutorialIndex?'active':i<tutorialIndex?'visited':''}"></span>`).join('');
 // Wait until scrolling and text layout have settled.
 requestAnimationFrame(()=>{if(tutorialIndex>=0)placeTutorialCoach();});
}
function startTutorial(){tutorialConfirm.hidden=true;tutorialIndex=0;tutorialShade.hidden=false;tutorialCoach.hidden=false;renderTutorialStep();}
function stopTutorial(){tutorialIndex=-1;tutorialShade.hidden=true;tutorialCoach.hidden=true;tutorialConfirm.hidden=true;document.querySelectorAll('.tutorial-focus').forEach(el=>el.classList.remove('tutorial-focus'));}
$('btnTutorial').addEventListener('click',showTutorialConfirmation);
$('tutorialNotNow').addEventListener('click',()=>{tutorialConfirm.hidden=true;$('btnTutorial').focus();});
$('tutorialStart').addEventListener('click',startTutorial);
$('tutorialExit').addEventListener('click',stopTutorial);
$('tutorialSkip').addEventListener('click',stopTutorial);
$('tutorialBack').addEventListener('click',()=>{tutorialIndex=Math.max(0,tutorialIndex-1);renderTutorialStep();});
$('tutorialNext').addEventListener('click',()=>{if(tutorialIndex===tutorialSteps.length-1){stopTutorial();return;}tutorialIndex++;renderTutorialStep();});
window.addEventListener('keydown',e=>{if(e.key==='Escape'&&(tutorialIndex>=0||!tutorialConfirm.hidden)){stopTutorial();}});
window.addEventListener('resize',()=>{if(tutorialIndex>=0)requestAnimationFrame(placeTutorialCoach);});
// Reposition when user scrolls either panel, without disabling editing.
document.querySelectorAll('.sidebar').forEach(panel=>panel.addEventListener('scroll',()=>{if(tutorialIndex>=0)requestAnimationFrame(placeTutorialCoach);},{passive:true}));
// Start with ready-to-use Hill + Guide, continuously editable even after generation.
state.mesh=getMesh(state.surface,41);updateGuide();updateEditorLock();updateUI();resize();requestAnimationFrame(loop);
window.__spga={state,generateTilted,warp,analyzePath,exportProject,updateGuide,refreshPreview,projectPose:p=>proj(p,makeCamera()),getHandles,rebuildAll,scheduleLiveWarp};