import {V,add,sub,mul,dot,cross,len,norm,mix,clamp,rad,angle,defaultProfile,defaultStroke,normalizeStroke,atStroke,smoothStroke,sampleSurface,worldFromLocal,getMesh,getCut,makeContactPath,makeTilted,rodMetrics,warpDemo,analyzePath} from './geometry.js';

const $ = id=>document.getElementById(id);
const stage=$('stage'),canvas=$('sceneCanvas'),ctx=canvas.getContext('2d',{alpha:false});
const sketch=$('drawCanvas'),dc=sketch.getContext('2d');
const state={surface:{type:'hill',profile:defaultProfile,offsetX:0,offsetZ:0,offsetY:0,rotation:0,scale:1},
 stroke:defaultStroke,cutAngle:0,cutOffset:0,amplitude:.7,rodLength:1.2,tiltAngle:65,coneAngle:40,reverse:false,
 mode:'orbit',viewLocked:false,cam:{az:.70,el:.57,distance:14.8,target:V(0,.7,0)},mesh:null,cut:null,contacts:[],tilted:[],warped:[],pose:0,
 playing:false,playClock:0,showBoth:true,planeVisible:true,gridVisible:true,
 layers:{surface:true,plane:true,guide:true,nominal:false,tilt:true,warp:true},drag:null,dirty:true};
let W=800,H=580,dpr=1,drawType='profile',drawRaw=[],drawActive=false,toastTimer=null;
function toast(msg){const el=$('toast');el.textContent=msg;el.classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.classList.remove('show'),3300);}
function invalidate(recreateMesh=false){if(recreateMesh)state.mesh=getMesh(state.surface,41);state.cut=getCut(state.surface,state.cutAngle,state.cutOffset);state.contacts=[];state.tilted=[];state.warped=[];state.pose=0;state.playing=false;$('btnPlay').textContent='▶';updateMetric();updateFrame();state.dirty=true;}
function updateGuide(){state.cut=getCut(state.surface,state.cutAngle,state.cutOffset);state.dirty=true;}
function updateMetricsText(id,v){$(id).textContent=v;}
function updateMetric(){
 const t=state.tilted.length?analyzePath(state.tilted,state.surface,state.rodLength):null;
 const w=state.warped.length?analyzePath(state.warped,state.surface,state.rodLength):null;
 updateMetricsText('metricPoses',state.tilted.length||'—');updateMetricsText('metricLength',state.rodLength.toFixed(2));
 updateMetricsText('metricTilt',t?t.maxPenetration.toFixed(3)+' u':'—');
 updateMetricsText('metricWarp',w?w.maxPenetration.toFixed(3)+' u':'—');
 updateMetricsText('metricCollision',t?(t.collisions+' → '+(w?w.collisions:'—')):'—');
 updateMetricsText('metricAngle',state.warped.length?Math.max(...state.warped.map(p=>p.deviationDeg)).toFixed(1)+'°':'—');
 const hint=$('metricHint'),pill=$('sceneStatus');
 if(!t){pill.textContent='READY';hint.textContent='点击 Generate Tilted，再点击 Warp 查看对比。';return;}
 if(!w){pill.textContent='TILTED';hint.textContent='已生成 Tilted。现在点击 Warp，观察工具如何避让曲面。';return;}
 pill.textContent=w.collisions?'PARTIAL':'COMPLETE';
 hint.textContent=`当前示例中 ${t.collisions}/${t.count} 个 Tilted 姿态发生采样穿透；Warp 后 ${w.collisions}/${w.count} 个。${w.collisions?'锥角限制可能阻止完全避障。':'采样检查未发现显著穿透。'} 不等于严格碰撞安全保证。`;
}
function updateFrame(){const max=Math.max(0,state.tilted.length-1,state.warped.length-1);$('frameSlider').max=max;$('frameSlider').value=clamp(state.pose,0,max);$('frameVal').textContent=`Pose ${String(state.pose+1).padStart(2,'0')} / ${Math.max(1,max+1)}`;}
function generateTilted(announce=true){
 state.contacts=makeContactPath(state.surface,state.stroke,state.cutAngle,state.cutOffset,67,state.amplitude);
 if(state.contacts.length<3){toast('路径超出了曲面范围。调整切割平面或缩小 Stroke width。');return false;}
 state.tilted=makeTilted(state.contacts,state.rodLength,state.tiltAngle,state.reverse,state.cutAngle);
 state.warped=[];state.pose=0;state.playing=false;$('btnPlay').textContent='▶';updateMetric();updateFrame();state.dirty=true;
 if(announce)lockScene();
 if(announce)toast(`生成 ${state.tilted.length} 个 Tilted poses。A0 位于曲面接触路径上。`);
 return true;
}
function warp(announce=true){
 if(!state.tilted.length&&!generateTilted(false))return;
 const result=warpDemo(state.tilted,state.surface,state.rodLength,state.coneAngle);
 state.warped=result.warped;updateMetric();updateFrame();state.dirty=true;
 if(announce)lockScene();
 if(announce)toast(`Warp 完成：${state.warped.length} 个 poses，剩余 ${result.failure} 个碰撞姿态（采样近似）。`);
}
const editControls=['surfaceX','surfaceZ','surfaceRot','surfaceScale','cutAngle','cutOffset','strokeAmp',
 'rodLength','tiltAngle','coneAngle','reverseTool','btnDrawSurface','btnGuide','btnDrawTraj'];
function updateEditorLock(){
 const locked=state.viewLocked;
 for(const id of editControls)$(id).disabled=locked;
 for(const b of $('surfaceOptions').querySelectorAll('button'))b.disabled=locked;
 $('btnModeMove').disabled=locked;
 $('btnModeRotate').disabled=locked;
 $('btnEditScene').hidden=!locked;
 $('editModeHint').textContent=locked?
  '🔒 已生成：拖动只调整视角；如需移动模型或修改参数，点 Edit scene。':
  '编辑阶段：可以 Move / Rotate 曲面。生成 Tilted 后自动进入只读查看。';
}
function lockScene(){
 state.viewLocked=true;
 state.mode='orbit';
 setMode('orbit');
 updateEditorLock();
 state.drag=null;
 canvas.classList.remove('dragging');
}
function unlockScene(){
 state.viewLocked=false;
 updateEditorLock();
 setMode('orbit');
 invalidate(); // Old poses are tied to the previous surface/nominal placement.
 toast('已进入编辑模式；原 Tilted/Warped 已清除，修改后重新生成。');
}
$('btnEditScene').addEventListener('click',unlockScene);
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
  circle3D(pa,.11,'#ffe2a4',cam,'#0e1626');circle3D(pb,.075,color,cam,'#0e1626');
  const q=proj(pa,cam);if(q){ctx.save();ctx.font='700 11px system-ui';ctx.fillStyle='#ffffff';ctx.fillText('A0',q.x+12,q.y+12);ctx.restore();}
 }
}
function drawScene(){
 state.dirty=false;
 ctx.setTransform(dpr,0,0,dpr,0,0);
 ctx.clearRect(0,0,W,H);
 const grad=ctx.createRadialGradient(W*.53,H*.4,H*.04,W*.52,H*.45,W*.9);
 grad.addColorStop(0,'#182b40');grad.addColorStop(.65,'#0d1a2a');grad.addColorStop(1,'#08111f');ctx.fillStyle=grad;ctx.fillRect(0,0,W,H);
 const cam=makeCamera();if(state.gridVisible)drawGrid(cam);
 drawMesh(cam);drawPlane(cam);
 if(state.layers.guide&&state.cut?.points?.length)poly3D(state.cut.points.map(p=>add(p,V(0,.028,0))),'#f7c77d',3.0,.99,cam);
 if(state.layers.nominal&&state.contacts.length){
  poly3D(state.contacts.map(c=>add(c.point,V(0,.053,0))),'#dfd7ff',2.2,.95,cam,[4,4]);
 }
 const tiltEnabled=state.layers.tilt&&state.tilted.length,warpEnabled=state.layers.warp&&state.warped.length;
 if(tiltEnabled){
  const arr=state.tilted;const step=7;
  for(let i=0;i<arr.length;i+=step)rodDraw(arr[i],'#ee9687',cam,false);
  poly3D(arr.map(p=>p.a1),'#ff9b89',1.6,.75,cam);
 }
 if(warpEnabled){
  const arr=state.warped;const step=7;
  for(let i=0;i<arr.length;i+=step)rodDraw(arr[i],'#65dce3',cam,false);
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
 for(const [id,value,suffix] of [['xVal',state.surface.offsetX.toFixed(2),''],['zVal',state.surface.offsetZ.toFixed(2),''],['rotVal',state.surface.rotation.toFixed(0),'°'],['scaleVal',state.surface.scale.toFixed(2),'×'],['cutAngleVal',state.cutAngle,'°'],['cutOffsetVal',state.cutOffset.toFixed(2),''],['amplitudeVal',state.amplitude.toFixed(2),''],['lengthVal',state.rodLength.toFixed(2),''],['tiltVal',state.tiltAngle,'°'],['coneVal',state.coneAngle,'°']])$(id).textContent=value+suffix;
 $('sceneName').textContent={hill:'Complex hill',saddle:'Saddle surface',profile:'Extruded 2D profile'}[state.surface.type];
 $('surfaceOptions').querySelectorAll('button').forEach(b=>b.classList.toggle('active',b.dataset.surface===state.surface.type));
 updateMetric();paintStrokeThumbnail();
}
function paintStrokeThumbnail(){const el=$('smallStroke'),c=el.getContext('2d');c.clearRect(0,0,el.width,el.height);c.fillStyle='#101d2a';c.fillRect(0,0,el.width,el.height);c.strokeStyle='#294359';c.lineWidth=1;c.beginPath();c.moveTo(0,el.height*.5);c.lineTo(el.width,el.height*.5);c.stroke();
 c.strokeStyle='#f6cc8b';c.lineWidth=2.3;c.beginPath();for(let i=0;i<90;i++){const x=i/89,p={x:12+x*(el.width-24),y:8+atStroke(state.stroke,x)*(el.height-20)};i?c.lineTo(p.x,p.y):c.moveTo(p.x,p.y);}c.stroke();}
function applyTerrain(type){state.surface.type=type;invalidate(true);updateUI();toast(type==='profile'?'已切换到手绘拉伸曲面。':'已加载 '+type+' 示例曲面。');}
$('surfaceOptions').addEventListener('click',e=>{const b=e.target.closest('button[data-surface]');if(b)applyTerrain(b.dataset.surface);});
for(const [id,prop,field,regen] of [['surfaceX','offsetX','surface',true],['surfaceZ','offsetZ','surface',true],['surfaceRot','rotation','surface',true],['surfaceScale','scale','surface',true],['cutAngle','cutAngle','root',false],['cutOffset','cutOffset','root',false],['strokeAmp','amplitude','root',false],['rodLength','rodLength','root',false],['tiltAngle','tiltAngle','root',false],['coneAngle','coneAngle','root',false]]){
 $(id).addEventListener('input',e=>{const val=Number(e.target.value);if(field==='surface')state.surface[prop]=val;else state[prop]=val;invalidate(regen);updateUI();});
}
$('reverseTool').addEventListener('change',e=>{state.reverse=e.target.checked;invalidate();});
$('btnGuide').addEventListener('click',()=>{updateGuide();toast(`已生成 ${state.cut.points.length} 个 Guide 截面点。调整 Plane angle / offset 可重新选择。`);});
$('btnTilted').addEventListener('click',()=>generateTilted());
$('btnWarp').addEventListener('click',()=>warp());
$('frameSlider').addEventListener('input',e=>{state.pose=Number(e.target.value);updateFrame();state.dirty=true;});
$('btnPlay').addEventListener('click',()=>{if(!state.tilted.length){toast('请先生成 Tilted。');return;}state.playing=!state.playing;state.playClock=performance.now();$('btnPlay').textContent=state.playing?'❚❚':'▶';});
$('showBoth').addEventListener('change',e=>{state.showBoth=e.target.checked;state.dirty=true;});
$('btnHome').addEventListener('click',()=>{state.cam={az:.70,el:.57,distance:14.8,target:V(0,.7,0)};state.dirty=true;});
$('btnGrid').addEventListener('click',e=>{state.gridVisible=!state.gridVisible;e.currentTarget.classList.toggle('on',state.gridVisible);state.dirty=true;});
$('btnPlane').addEventListener('click',e=>{state.planeVisible=!state.planeVisible;e.currentTarget.classList.toggle('on',state.planeVisible);state.dirty=true;});
for(const [id,key] of [['layerSurface','surface'],['layerPlane','plane'],['layerGuide','guide'],['layerNominal','nominal'],['layerTilt','tilt'],['layerWarp','warp']])$(id).addEventListener('change',e=>{state.layers[key]=e.target.checked;state.dirty=true;});
function setMode(mode){if(state.viewLocked&&mode!=='orbit'){toast('结果已锁定。点 Edit scene 后才能移动/旋转曲面。');return;}state.mode=mode;for(const [id,m] of [['btnModeOrbit','orbit'],['btnModeMove','move'],['btnModeRotate','rotate']])$(id).classList.toggle('active',mode===m);$('stageHelp').textContent={orbit:'DRAG TO ORBIT · SCROLL TO ZOOM · SHIFT + DRAG TO PAN',move:'DRAG TO MOVE SURFACE · SCROLL TO ZOOM',rotate:'DRAG HORIZONTALLY TO ROTATE SURFACE'}[mode];}
for(const [id,mode] of [['btnModeOrbit','orbit'],['btnModeMove','move'],['btnModeRotate','rotate']])$(id).addEventListener('click',()=>setMode(mode));
canvas.addEventListener('pointerdown',e=>{canvas.setPointerCapture(e.pointerId);state.drag={x:e.clientX,y:e.clientY,button:e.button,shift:e.shiftKey};canvas.classList.add('dragging');});
canvas.addEventListener('pointermove',e=>{
 const d=state.drag;if(!d)return;const dx=e.clientX-d.x,dy=e.clientY-d.y;d.x=e.clientX;d.y=e.clientY;
 const cam=state.cam;
 if(!state.viewLocked&&state.mode==='move'){
  const scale=.018*cam.distance/14.8;
  state.surface.offsetX=clamp(state.surface.offsetX+(dx*Math.cos(cam.az)+dy*Math.sin(cam.az))*scale,-2.8,2.8);
  state.surface.offsetZ=clamp(state.surface.offsetZ+(-dx*Math.sin(cam.az)+dy*Math.cos(cam.az))*scale,-2.8,2.8);
  $('surfaceX').value=state.surface.offsetX;$('surfaceZ').value=state.surface.offsetZ;invalidate(true);updateUI();
 }else if(!state.viewLocked&&state.mode==='rotate'){
  state.surface.rotation=clamp(state.surface.rotation+dx*.35,-180,180);$('surfaceRot').value=state.surface.rotation;invalidate(true);updateUI();
 }else if(e.shiftKey||d.shift){const basis=makeCamera();cam.target=add(cam.target,add(mul(basis.r,-dx*.012*cam.distance/14),mul(basis.u,dy*.012*cam.distance/14)));state.dirty=true;
 }else{cam.az-=dx*.006;cam.el=clamp(cam.el+dy*.005,.15,1.43);state.dirty=true;}
});
function stopDrag(){state.drag=null;canvas.classList.remove('dragging');}
canvas.addEventListener('pointerup',stopDrag);canvas.addEventListener('pointercancel',stopDrag);
canvas.addEventListener('wheel',e=>{e.preventDefault();state.cam.distance=clamp(state.cam.distance*Math.exp(e.deltaY*.001),5.5,30);state.dirty=true;},{passive:false});
function drawingCanvas(){
 dc.clearRect(0,0,sketch.width,sketch.height);dc.fillStyle='#0d1929';dc.fillRect(0,0,sketch.width,sketch.height);
 dc.lineWidth=1;dc.strokeStyle='#263b50';for(let x=0;x<=sketch.width;x+=sketch.width/10){dc.beginPath();dc.moveTo(x,0);dc.lineTo(x,sketch.height);dc.stroke();}for(let y=0;y<=sketch.height;y+=sketch.height/6){dc.beginPath();dc.moveTo(0,y);dc.lineTo(sketch.width,y);dc.stroke();}
 const points=drawRaw.length>1?drawRaw:(drawType==='profile'?state.surface.profile:state.stroke);
 if(points.length>1){dc.lineWidth=4;dc.lineJoin='round';dc.lineCap='round';dc.strokeStyle=drawType==='profile'?'#73dee6':'#f5c079';dc.beginPath();for(let i=0;i<points.length;i++){const p=points[i],x=clamp(p.x,0,1)*sketch.width,y=clamp(p.y,0,1)*sketch.height;i?dc.lineTo(x,y):dc.moveTo(x,y);}dc.stroke();}
 dc.fillStyle='#89a7be';dc.font='12px system-ui';dc.fillText(drawType==='profile'?'HEIGHT ↑':'LATERAL OFFSET ↑',16,23);
}
function openModal(type){drawType=type;drawRaw=[];$('modalTitle').textContent=type==='profile'?'Draw a 2D surface profile':'Draw a 2D nominal trajectory';$('modalHelp').textContent=type==='profile'?'从左向右手绘一条侧面高度轮廓（不是闭合形状）。网站会沿 Z 方向拉伸成 3D 曲面。':'从左向右画路径；横轴为 Guide 上的进度，纵轴为左右偏移。导入 3D 时路径会贴到曲面。';$('drawStatus').textContent='Drag to draw from left to right';$('drawModal').hidden=false;drawingCanvas();}
$('btnDrawSurface').addEventListener('click',()=>openModal('profile'));
$('btnDrawTraj').addEventListener('click',()=>openModal('trajectory'));
$('btnCloseModal').addEventListener('click',()=>$('drawModal').hidden=true);
$('drawModal').addEventListener('click',e=>{if(e.target===$('drawModal'))$('drawModal').hidden=true;});
window.addEventListener('keydown',e=>{if(e.key==='Escape')$('drawModal').hidden=true;});
function toSketchPt(e){const r=sketch.getBoundingClientRect();return {x:clamp((e.clientX-r.left)/r.width,0,1),y:clamp((e.clientY-r.top)/r.height,0,1)};}
sketch.addEventListener('pointerdown',e=>{drawRaw=[toSketchPt(e)];drawActive=true;sketch.setPointerCapture(e.pointerId);drawingCanvas();});
sketch.addEventListener('pointermove',e=>{if(!drawActive)return;const p=toSketchPt(e),last=drawRaw.at(-1);if(Math.abs(p.x-last.x)+Math.abs(p.y-last.y)>.002){drawRaw.push(p);drawingCanvas();}});
sketch.addEventListener('pointerup',()=>{drawActive=false;$('drawStatus').textContent=`${drawRaw.length} points drawn`;});
sketch.addEventListener('pointercancel',()=>drawActive=false);
$('btnDrawClear').addEventListener('click',()=>{drawRaw=[];drawingCanvas();$('drawStatus').textContent='Cleared. Draw left to right.';});
$('btnDrawDefault').addEventListener('click',()=>{drawRaw=(drawType==='profile'?defaultProfile:defaultStroke).map(p=>({...p}));drawingCanvas();});
$('btnDrawApply').addEventListener('click',()=>{
 const pts=normalizeStroke(drawRaw.length>1?drawRaw:(drawType==='profile'?state.surface.profile:state.stroke));
 if(!pts){$('drawStatus').textContent='Please draw a longer left-to-right line.';return;}
 if(drawType==='profile'){state.surface.profile=smoothStroke(pts,72,.045);state.surface.type='profile';invalidate(true);}else{state.stroke=smoothStroke(pts,72,.013);invalidate();}
 $('drawModal').hidden=true;updateUI();toast(drawType==='profile'?'2D 轮廓已生成拉伸曲面。':'新的 Nominal Stroke 已应用，请重新生成 Tilted。');
});
function exportProject(){
 const data={format:'SPGA-WebLab-Demo-v1',notice:'Browser approximation, NOT the archived research SPGA-M2 solver.',units:'arbitrary 3D display units',surface:state.surface,stroke:state.stroke,cut:{angleDeg:state.cutAngle,offset:state.cutOffset,amplitude:state.amplitude},tool:{length:state.rodLength,tiltAngleDeg:state.tiltAngle,coneHalfAngleDeg:state.coneAngle,reverse:state.reverse},guide:state.cut.points,contact:state.contacts.map(p=>p.point),tilted:state.tilted.map(p=>({A0:p.a0,A1:p.a1})),warped:state.warped.map(p=>({A0:p.a0,A1:p.a1})),metrics:{tilted:analyzePath(state.tilted,state.surface,state.rodLength),warped:analyzePath(state.warped,state.surface,state.rodLength)}};
 const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='spga_web_lab_demo.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),2500);toast('已导出曲面参数、路径和 Tilted / Warped poses (JSON)。');
}
$('btnExport').addEventListener('click',exportProject);
$('btnReset').addEventListener('click',()=>{Object.assign(state.surface,{type:'hill',profile:defaultProfile,offsetX:0,offsetZ:0,offsetY:0,rotation:0,scale:1});Object.assign(state,{stroke:defaultStroke,cutAngle:0,cutOffset:0,amplitude:.7,rodLength:1.2,tiltAngle:65,coneAngle:40,reverse:false,cam:{az:.70,el:.57,distance:14.8,target:V(0,.7,0)}});
 for(const [id,n] of [['surfaceX',0],['surfaceZ',0],['surfaceRot',0],['surfaceScale',1],['cutAngle',0],['cutOffset',0],['strokeAmp',.7],['rodLength',1.2],['tiltAngle',65],['coneAngle',40]])$(id).value=n;
 $('reverseTool').checked=false;invalidate(true);state.viewLocked=false;updateEditorLock();setMode('orbit');updateUI();toast('已恢复默认 Hill 场景；可编辑后 Generate Tilted → Warp。');});

// Start in edit mode with a ready-to-use Hill & Guide, but NO generated poses.
// Once the user clicks Generate Tilted or Warp, geometry is locked for viewing.
state.mesh=getMesh(state.surface,41);updateGuide();updateEditorLock();updateUI();resize();requestAnimationFrame(loop);
window.__spga={state,generateTilted,warp,analyzePath,exportProject,updateGuide};