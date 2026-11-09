/* Run original research SPGA M3 solver in the browser. No Python server required.
 * NumPy and SciPy run in WebAssembly. When Numba is absent, the original
 * research code automatically chooses its NumPy-based visibility fallback.
 */
const ROOT='https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const MODULES=['__init__.py','conic.py','dual_arc.py','geometry.py','geometry_field.py','mobility.py','multi_normal_field.py','ribbon_energy.py','surface.py','warping.py'];
let runtime;
const tell=(id,message)=>self.postMessage({id,type:'progress',message});
async function init(id){
  tell(id,'Loading Python + NumPy + SciPy in the browser…');
  importScripts(ROOT+'pyodide.js');
  const py=await loadPyodide({indexURL:ROOT});
  await py.loadPackage(['numpy','scipy']);
  tell(id,'Loading original Python SPGA-M3 research method…');
  const names=[...MODULES.map(x=>'spga/'+x),'browser_bridge.py','configs/milestone_m3.json'];
  const loaded=await Promise.all(names.map(async name=>{
    const r=await fetch(new URL('../backend/'+name,self.location.href));
    if(!r.ok)throw new Error(name+': HTTP '+r.status);
    return [name,await r.text()];
  }));
  py.FS.mkdirTree('/home/pyodide/backend/spga');
  py.FS.mkdirTree('/home/pyodide/backend/configs');
  for(const [path,value] of loaded)py.FS.writeFile('/home/pyodide/backend/'+path,value);
  py.runPython("import sys; sys.path.insert(0,'/home/pyodide/backend'); import browser_bridge");
  tell(id,'SPGA-M3 Python ready.');
  return py;
}
self.onmessage=async ev=>{
  const {id,payload}=ev.data||{};
  if(!id||!payload)return;
  try{
    const py=await(runtime??=init(id));
    tell(id,'Actual M3 force balance: '+payload.poses.length+' poses, maximum '+payload.maxIterations+' iterations…');
    py.globals.set('_spga_payload',JSON.stringify(payload));
    const raw=py.runPython('browser_bridge.solve_json(_spga_payload)');
    py.globals.delete('_spga_payload');
    self.postMessage({id,type:'result',result:JSON.parse(raw)});
  }catch(error){self.postMessage({id,type:'error',message:error?.message||String(error)});}
};
