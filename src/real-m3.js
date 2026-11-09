import { getMesh } from './geometry.js';
import { SPGA_M3_BACKEND_URL } from './m3-config.js';

const sleep = ms => new Promise(resolve=>setTimeout(resolve,ms));

/** Call the actual Python SPGA-M3 solver; never use the JS approximation. */
export async function computeM3(state, onStatus=()=>{}, isStale=()=>false) {
  const base=SPGA_M3_BACKEND_URL.trim().replace(/\/$/,'');
  if (!/^https?:\/\//.test(base)) {
    throw new Error('M3 backend not configured. Set SPGA_M3_BACKEND_URL in src/m3-config.js.');
  }
  // The research solver receives real triangles and real Tilted dual-arc poses.
  // N=21 (800 faces) is the web scene discretization; shown in exported metadata.
  const mesh=getMesh(state.surface,21);
  const payload={
    verts: mesh.verts,
    faces: mesh.faces,
    poses:state.tilted.map(p=>({a0:p.a0,a1:p.a1,contact:p.contact,passId:p.passId})),
    rodLength:state.rodLength,
    coneAngle:state.coneAngle,
    gains:{
      geometry:state.geometryGain,
      conic:state.conicGain,
      tension:state.tensionGain,
      bending:state.bendingGain,
    },
    maxIterations:3000,
  };
  onStatus('Submitting actual SPGA-M3 job…');
  const start=await fetch(base+'/api/warp', {
    method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),
  });
  const startData=await start.json().catch(()=>({}));
  if(!start.ok)throw new Error(startData.detail || `M3 API error ${start.status}`);
  if(!startData.jobId)throw new Error('M3 API did not return a job ID');
  for(let polls=0; polls<1800;polls++){
    if(isStale())return null;
    await sleep(1200);
    if(isStale())return null;
    const r=await fetch(base+'/api/warp/'+encodeURIComponent(startData.jobId));
    const job=await r.json().catch(()=>({}));
    if(!r.ok)throw new Error(job.detail||`M3 polling error ${r.status}`);
    if(job.status==='complete'){
      if(!job.result?.warped?.length)throw new Error('M3 returned no poses');
      return job.result;
    }
    if(job.status==='error')throw new Error(job.error||'M3 solver failed');
    if(polls%4===0)onStatus('M3 computing actual force balance…');
  }
  throw new Error('M3 job polling timed out');
}