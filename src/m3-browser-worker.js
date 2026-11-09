/* Run original research SPGA M3 solver in the browser. No Python server required.
 * NumPy and SciPy run in WebAssembly. When Numba is absent, the original
 * research code automatically chooses its NumPy-based visibility fallback.
 */
const ROOT='https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const MODULES=['conic.py','dual_arc.py','geometry.py','geometry_field.py','mobility.py','multi_normal_field.py','ribbon_energy.py','surface.py','warping.py'];
let runtime;
const tell=(id,message,phase='preparing',iteration=null,maxIterations=null)=>self.postMessage({id,type:'progress',message,phase,iteration,maxIterations});
async function init(id){
  tell(id,'Loading Python WebAssembly runtime…','runtime');
  importScripts(ROOT+'pyodide.js');
  const py=await loadPyodide({indexURL:ROOT});
  tell(id,'Loading NumPy and SciPy…','packages');
  await py.loadPackage(['numpy','scipy']);
  tell(id,'Fetching original M3 Python modules and configuration…','sources');
  const names=[...MODULES.map(x=>'spga/'+x),'browser_bridge.py','configs/milestone_m3.json'];
  const loaded=await Promise.all(names.map(async name=>{
    const r=await fetch(new URL('../backend/'+name,self.location.href));
    if(!r.ok)throw new Error(name+': HTTP '+r.status);
    return [name,await r.text()];
  }));
  py.FS.mkdirTree('/home/pyodide/backend/spga');
  py.FS.mkdirTree('/home/pyodide/backend/configs');
  // GitHub Pages/Jekyll may omit underscore-prefixed Python files.
  // __init__.py is a standard package marker; create it inside Pyodide.
  py.FS.writeFile('/home/pyodide/backend/spga/__init__.py', '');
  for(const [path,value] of loaded)py.FS.writeFile('/home/pyodide/backend/'+path,value);
  tell(id,'Initializing original M3 modules…','sources');
  // Make the research Numba decorators transparent in Pyodide. This executes
  // the exact same research BVH ray-intersection function, without JIT.
  py.runPython("import sys,types; stub=types.ModuleType('numba'); stub.njit=lambda *a,**kw: lambda f:f; sys.modules['numba']=stub; sys.path.insert(0,'/home/pyodide/backend'); import browser_bridge");
  tell(id,'SPGA-M3 Python ready.','sources');
  return py;
}
self.onmessage=async ev=>{
  const {id,payload}=ev.data||{};
  if(!id||!payload)return;
  try{
    const py=await(runtime??=init(id));
    tell(id,'Computing M3 force balance: '+payload.poses.length+' poses (up to '+payload.maxIterations+' iterations)…','solving',0,payload.maxIterations);
    py.globals.set('_spga_payload',JSON.stringify(payload));
    // Progress is counted from real calls inside the original M3 solver.
    // This does not change geometry, forces, constraints or convergence.
    py.globals.set('_spga_progress_js',(iteration,maxIterations)=>{
      tell(id,'M3 force balance · iteration '+iteration+' / '+maxIterations+' maximum','solving',iteration,maxIterations);
    });
    let raw;
    try{
      raw=py.runPython('browser_bridge.solve_json(_spga_payload, _spga_progress_js)');
    }finally{
      py.globals.delete('_spga_payload');
      py.globals.delete('_spga_progress_js');
    }
    tell(id,'M3 solved; returning warped poses and diagnostics…','rendering');
    self.postMessage({id,type:'result',result:JSON.parse(raw)});
  }catch(error){self.postMessage({id,type:'error',message:error?.message||String(error)});}
};
