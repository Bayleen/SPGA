import {getMesh,add,sub,mul,norm,mix,clamp} from './geometry.js';
import {SPGA_M3_BACKEND_URL} from './m3-config.js';

// Default: run the ORIGINAL Python M3 method in a browser Web Worker.
// Optionally configure an HTTPS Python API for high-throughput full research runs.
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let active=null,cachedWorker=null,serial=0;

function chooseIndices(poses,limit){
  if(poses.length<=limit)return poses.map((_,i)=>i);
  const selected=new Set([0,poses.length-1]);
  // Fixed A0 anchors must ALWAYS be solved, not approximated.
  poses.forEach((p,i)=>{if(p.contact)selected.add(i);});
  const n=Math.max(limit,selected.size);
  for(let j=0;j<n;j++)selected.add(Math.round(j*(poses.length-1)/(n-1)));
  return [...selected].sort((a,b)=>a-b);
}

function renderPreview(poses,solved,ids,L){
  // Only for explicitly labelled FAST PREVIEW. M3 solves the selected poses;
  // corrections for intervening rendered poses are interpolated, not simulated.
  const byIndex=new Map(ids.map((id,i)=>[id,solved[i]]));
  const out=[];let k=0;
  for(let i=0;i<poses.length;i++){
    if(byIndex.has(i)){out.push(byIndex.get(i));continue;}
    while(k<ids.length-2&&ids[k+1]<i)k++;
    const il=ids[k],ir=ids[k+1],t=(i-il)/(ir-il);
    const p=poses[i],a=poses[il],b=poses[ir],wa=byIndex.get(il),wb=byIndex.get(ir);
    const a0=p.contact?p.a0:add(p.a0,mix(sub(wa.a0,a.a0),sub(wb.a0,b.a0),t));
    const axis=norm(mix(wa.axis,wb.axis,t));
    const a1=add(a0,mul(axis,L));
    const deviationDeg=Math.acos(clamp(axis.x*p.axis.x+axis.y*p.axis.y+axis.z*p.axis.z,-1,1))*180/Math.PI;
    out.push({a0,a1,axis,contact:p.contact,passId:p.passId,deviationDeg});
  }
  return out;
}

function sceneRequest(state){
  const full=state.m3Quality==='full';
  const resolution=full?21:9;
  const mesh=getMesh(state.surface,resolution);
  const ids=full?state.tilted.map((_,i)=>i):chooseIndices(state.tilted,18);
  const poses=ids.map(i=>state.tilted[i]);
  const payload={
    verts:mesh.verts,faces:mesh.faces,
    poses:poses.map(p=>({a0:p.a0,a1:p.a1,contact:p.contact,passId:p.passId})),
    rodLength:state.rodLength,coneAngle:state.coneAngle,
    gains:{geometry:state.geometryGain,conic:state.conicGain,
      tension:state.tensionGain,bending:state.bendingGain},
    maxIterations:full?3000:20
  };
  return {payload,ids,full,resolution};
}

async function viaServer(base,payload,onStatus,isStale){
  const startResp=await fetch(base+'/api/warp',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
  const job=await startResp.json();
  if(!startResp.ok)throw Error(job.detail||'Server error: '+startResp.status);
  for(let poll=0;poll<1800;poll++){
    if(isStale())return null;
    await sleep(1200);
    if(isStale())return null;
    const r=await fetch(base+'/api/warp/'+encodeURIComponent(job.jobId));
    const response=await r.json();
    if(!r.ok)throw Error(response.detail||'Server query error: '+r.status);
    if(response.status==='complete')return response.result;
    if(response.status==='error')throw Error(response.error||'M3 solver error');
    if(poll%5===0)onStatus('Computing original Python M3 force balance on server…');
  }
  throw Error('Server polling timed out');
}

function viaBrowser(payload,onStatus,isStale){
  const id=++serial;
  if(active){active.worker.terminate();active.done(null);active=null;cachedWorker=null;}
  const worker=cachedWorker??new Worker(new URL('./m3-browser-worker.js',import.meta.url));
  cachedWorker=worker;
  return new Promise((resolve,reject)=>{
    let finished=false;
    const finish=(fn,value)=>{
      if(finished)return;
      finished=true;
      clearInterval(timer);
      if(active?.id===id)active=null;
      fn(value);
    };
    active={id,worker,done:v=>finish(resolve,v)};
    worker.onmessage=ev=>{
      if(ev.data.id!==id)return;
      if(ev.data.type==='progress')onStatus(ev.data.message);
      if(ev.data.type==='result'){finish(resolve,ev.data.result);}
      if(ev.data.type==='error'){worker.terminate();cachedWorker=null;finish(reject,Error(ev.data.message));}
    };
    worker.onerror=ev=>{worker.terminate();cachedWorker=null;finish(reject,Error(ev.message||'Python worker failed'));};
    const timer=setInterval(()=>{if(isStale()){worker.terminate();cachedWorker=null;finish(resolve,null);}},250);
    worker.postMessage({id,payload});
  });
}

export async function computeM3(state,onStatus=()=>{},isStale=()=>false){
  const {payload,ids,full,resolution}=sceneRequest(state);
  const base=SPGA_M3_BACKEND_URL.trim().replace(/\/$/,'');
  onStatus((full?'FULL':'FAST PREVIEW')+': original SPGA-M3 on '+ids.length+'/'+state.tilted.length+' poses; '+payload.maxIterations+' max iterations.');
  const solved=base?await viaServer(base,payload,onStatus,isStale):await viaBrowser(payload,onStatus,isStale);
  if(!solved||isStale())return null;
  if(!Array.isArray(solved.warped)||solved.warped.length!==ids.length)throw Error('Incorrect M3 pose count');
  const previewInterpolated=!full&&ids.length<state.tilted.length;
  return {
    ...solved,
    warped:previewInterpolated?renderPreview(state.tilted,solved.warped,ids,state.rodLength):solved.warped,
    diagnostics:{...solved.diagnostics,
      executor:base?'Native Python HTTPS':'Python WebAssembly in browser',
      meshResolution:resolution,solvedPoses:ids.length,totalPoses:state.tilted.length,
      previewInterpolated,fullSolve:full}
  };
}
