/*
 * Recorded original SPGA force-balance solution for the exact public welcome scene.
 * Source: uploaded SPGA_M3_COMPREHENSIVE_EXPERIMENTS_20261102 Python solver,
 * 9x9 mesh, 81 all-contact poses, 20 actual iterations.
 * NOT CONVERGED (max_iterations). This is a 20-step preview, NOT a safety claim.
 * Signed 16-bit scene coordinates quantized at 1/8192 display unit.
 * Original solver input is verified against ALL active Tilted A0/A1 before use.
 */
const DEFAULT_TILTED="i8gWBvK/rtk0Gw/b0MmTBibC5tlhHqHbD8tIB07E7dmCIRbcRswuCGjG9Nl5JHrcc81ACXLIC9o3J+bcmM54Cm3KPtq/KWrdtM/MC1nMkdoWLBHeydA5DTjOB9tGLt7e19G4Dg3Qn9tXMNLf4dJFENjRV9xMMuvg5tPdEZ3TLd0sNCji6dR7E13VH973NYbj6tUdFRrXLN+wNwXl6tbCFtbYU+BWOaPm69dlGJPakeHoOl7o7NgGGlHc5uJnPDbq79mjGxLeUOTRPSns9do5HdffzOUlPzXu/tvIHqLhV+djQFnwCt1OIHLj7+iJQZHyGt7KIUjljuqbQtv0Ld86IyXnMOyYQzP3ROCgJAnp0e2ERJX5X+H5JfPqa+9jRf37feJHJ+Ls+/A4Rmn+n+OJKNjuf/IGR9cAw+TBKdLw9PPOR0gD6uXuKtLyXPWRSLwFFOcRLNX0uPZLSTgIQOgqLd32Dvj2ScEKb+k2Lur4YfmGSmANoeo1L/z6u/ruSh8Q1uskMBP9IfwaSwkTD+39MDH/nv3uSikWTe67MVgBOP9HSowZj+9UMoYD8wD0SDgd1vC8Mr0FzgK+RichIfLnMvkHtgRtQzwla/PGMjYKiwboPjgpsfROMmoMIghZOcUs6/V8MYoOWgk4M50vFPdYMIwQMQobLbIxKvjuLm0SwgppJywzK/lNLSoUKAtPIkM0GvqCK8gVfQvGHSU1+fqXKUoXzgu3GfA1yvuVJ7UYIgwLFrQ2kPyCJQsafAytEno3Tf1hI1Mb3gyJD0g4Av43IY0cRw2XDB45sv4EH70duA3MCf85Xf/NHOUeMg4lB+w6BQCRGgggtA6bBOc7qgBTGCchPg8wAvA8TwETFkQi0w/i/wo+9AHUE2EjchCy/TY/mQKWEYAkHhGk+3lAQQNbD6Ml1xG/+dVB7QMjDc4mohIJ+FFDngT0CgAofxOR9vFEVwXMCEApdhRq9cFGGga0BpEqiRW19MZI6AavBPcrvhaj9A1LyQfGAnstGRiB9Z1NvggKASUvkBnL92dQzwmS//0w/xoh/ChT/AqC/gczAhzdAixVQAz9/Tg1LhwTC55Viw0Q/nQ3pxudEqpUzg6i/qQ5Ehs4GIRTBBCJ/7071RoZHARTLxGnAMM9BRvZHllTUxLhAbw/lBvgIG1UcxMpA7BBdRxkIiJWkxRxBKJDnR15I15YtRWwBZpFAx8lJA1b3RbcBplHoyBjJB9eCxjtB6RJdCIpJH9hQBncCLtLcCRvIxhlfBqiCd9NiyY1Is5ovhs5Cg5Q6iiOIGFs";
const DEFAULT_WARPED="i8gWBvK/c9cOIdzW0MmTBibCutc9I5XXD8tIB07E6tdlJV3YRswuCGjGHNiUJxPZc81ACXLIX9i3Kc7ZmM54Cm3KudjKK5natM/MC1nMMdnILYHbydA5DTjOyNmzL4nc19G4Dg3Qf9qNMbTd4dJFENjRUdtaM/ze5tPdEZ3TP9wZNWTg6dR7E13VRt3MNujh6tUdFRrXZd5yOIfj6tbCFtbYmt8LOj7l69dlGJPa4+CWOwzn7NgGGlHcQeIRPfHo79mjGxLesON7Pu3q9do5HdffMOXSPwDt/tvIHqLhv+YSQSrvCt1OIHLjWug9QmvxGt7KIUjl/elQQ8DzLd86IyXnoutPRCT2ROCgJAnpRu0+RZP4X+H5JfPq4u4fRgf7feJHJ+LscvD3Rn39n+OJKNju9fHIR/T/w+TBKdLwa/OTSG0C6uXuKtLy1/RTSe0EFOcRLNX0O/YHSngHQOgqLd32nPemShQKb+k2Lur4/fgoS8kMoeo1L/z6Zfp/S54P1uskMBP92/uZS54SD+39MDH/Zf1bS9QVTe67MVgBC/+mSkkZj+9UMoYDzwBMSQId1vC8Mr0FrgIYR/kgIfLnMvkHmQTVQxIla/PGMjYKcwZrPxMpsfROMmoMEQj6Oass6/V8MYoOVAkJNJMvFPdYMIwQPAoiLsExKvjuLm0S4wqoKFwzK/lNLSoUYwvPI5s0GvqCK8gV1guNH6w1+fqXKUoXSAzPG602yvuVJ7UYwAyAGK43kPyCJQsaQQ2QFbs4Tf1hI1Mbyw3iEtM5Av43IY0cVQ5SEO06sv4EH70d6Q4KDiA8Xf/NHOUeiA8RDHE9BQCRGgggLhBnCuQ+qgBTGCch2RApCYBATwETFkQihBF+CFJC9AHUE2EjKxJyCEZEmQKWEYAkyhLJCDFGQQNbD6MlXhNVCfhH7QMjDc4m6RPvCZJJngT0CgAocRRoCvxKVwXMCEAp+hTeCkNMGga0BpEqfhV8C2tN6AavBPcr/xVMDHJOyQfGAnstghZaDVZPvggKASUvCxezDg9QzwmS//0wixejEIdQ/AqC/gczExjgEtVQQAz9/Tg1qBhGFR9Riw0Q/nQ3/BhQGApRzg6i/qQ5JBluG9dQBBCJ/707ZhkXHgVRLxGnAMM95hk9IMlRUxLhAbw/rRrvISlTcxMpA7BBuhs+IxdVkxRxBKJDAh02JHxXtRWwBZpFfh7TJEha3RbcBplHLCAOJWtdCxjtB6RJCCLYJNlgQBncCLtLDSQnJH9kfBqiCd9NMCb2IkJovhs5Cg5QmShTIedr";
const QUANT=8192;
function unpack(b64){
 const raw=atob(b64);
 if(raw.length!==972)throw Error('Invalid recorded SPGA data');
 const view=new DataView(Uint8Array.from(raw,ch=>ch.charCodeAt(0)).buffer);
 return Array.from({length:486},(_,i)=>view.getInt16(i*2,true)/QUANT);
}
let decoded=null;
const point=(v,i)=>({x:v[i],y:v[i+1],z:v[i+2]});
const near=(a,b,tol=0.00028)=>Math.abs(a.x-b.x)<tol&&Math.abs(a.y-b.y)<tol&&Math.abs(a.z-b.z)<tol;
const equals=(x,y)=>Math.abs(x-y)<1e-9;

/** Only serves the exact default preset; ANY edit falls back to the original solver. */
export function getDefaultRecordedPreview(state,payload,ids,full){
 if(full||state.m3Quality!=='preview'||payload.maxIterations!==20||payload.poses.length!==81||ids.length!==81)return null;
 if(state.taskMode!=='contact'||state.a1Stroke!==null||state.reverse!==false)return null;
 const surf=state.surface;
 if(surf.type!=='hill'||!equals(surf.offsetX,0)||!equals(surf.offsetY,0)||
    !equals(surf.offsetZ,0)||!equals(surf.rotation,0)||!equals(surf.scale,1))return null;
 if(!equals(state.cutAngle,60)||!equals(state.cutOffset,0.5)||!equals(state.amplitude,0.7)||
    !equals(state.rodLength,1.2)||!equals(state.tiltAngle,25)||!equals(state.coneAngle,30)||
    !equals(state.geometryGain,1)||!equals(state.conicGain,1)||
    !equals(state.tensionGain,0.6)||!equals(state.bendingGain,0.35))return null;
 if(state.stroke.length!==70||state.stroke.some(p=>!equals(p.y,.5)))return null;
 if(!decoded)decoded={tilted:unpack(DEFAULT_TILTED),warped:unpack(DEFAULT_WARPED)};
 for(let i=0;i<81;i++){
  const k=i*6,pose=payload.poses[i],t=decoded.tilted;
  if(ids[i]!==i||!pose.contact||pose.passId!==0||!near(pose.a0,point(t,k))||!near(pose.a1,point(t,k+3)))return null;
 }
 const warped=payload.poses.map((p,i)=>{
  const k=i*6,v=decoded.warped;
  const a0=p.a0; // original research: all A0 anchors exactly fixed
  const target=point(v,k+3),dx=target.x-a0.x,dy=target.y-a0.y,dz=target.z-a0.z;
  const n=Math.hypot(dx,dy,dz);
  if(n<1e-8)throw Error('Recorded SPGA axis is invalid');
  const axis={x:dx/n,y:dy/n,z:dz/n};
  const a1={x:a0.x+axis.x*state.rodLength,y:a0.y+axis.y*state.rodLength,z:a0.z+axis.z*state.rodLength};
  const u={x:(p.a1.x-p.a0.x)/state.rodLength,y:(p.a1.y-p.a0.y)/state.rodLength,z:(p.a1.z-p.a0.z)/state.rodLength};
  const deviationDeg=Math.acos(Math.max(-1,Math.min(1,axis.x*u.x+axis.y*u.y+axis.z*u.z)))*180/Math.PI;
  return {a0,a1,axis,contact:true,passId:0,deviationDeg};
 });
 return {
  solver:'SPGA original Python · recorded example',
  units:{scene:'display_unit',metres_per_display_unit:0.05},
  warped,
  diagnostics:{
   converged:false,mode:'max_iterations',iterations:20,
   force_residual:0.12786547002867013,max_anchor_error_m:0,
   final_min_phi_full_tool_m:-0.002917123149729796,
   elapsed_seconds:0,geometry_field:'SPGA continuous multi-normal field',
   executor:'Recorded original Python calculation',meshResolution:9,
   solvedPoses:81,totalPoses:81,previewInterpolated:false,fullSolve:false,
   recordedPreset:true
  }
 };
}
