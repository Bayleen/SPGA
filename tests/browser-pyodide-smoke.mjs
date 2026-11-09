// End-to-end smoke test: starts the static website and runs original Python M3
// inside Chromium's Web Worker (Pyodide). Requires playwright + chromium.
import {chromium} from 'playwright';
import {spawn} from 'node:child_process';
import assert from 'node:assert/strict';

const server=spawn('python',['-m','http.server','8765','--bind','127.0.0.1'],
  {cwd:process.cwd(),stdio:'ignore'});
let browser;
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
try{
  await sleep(1800);
  browser=await chromium.launch({headless:true});
  const page=await browser.newPage();
  const errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto('http://127.0.0.1:8765/',{waitUntil:'domcontentloaded'});
  const result=await page.evaluate(async()=>{
    const n=7,verts=[],faces=[],poses=[];
    for(let j=0;j<n;j++)for(let i=0;i<n;i++){
      const x=(i/(n-1)-.5)*4,z=(j/(n-1)-.5)*4;
      verts.push({x,y:0.2,z});
    }
    for(let j=0;j<n-1;j++)for(let i=0;i<n-1;i++){
      const a=j*n+i,b=a+1,c=a+n,d=c+1;
      faces.push([a,c,b],[b,c,d]);
    }
    for(let k=0;k<3;k++)poses.push({
      a0:{x:-.8,y:.15,z:(k-1)*.2},
      a1:{x:.2,y:.15,z:(k-1)*.2},
      contact:k!==1,passId:0
    });
    const payload={verts,faces,poses,rodLength:1,coneAngle:30,
      gains:{geometry:1,conic:1,tension:.6,bending:.35},
      maxIterations:20};
    return await new Promise((resolve,reject)=>{
      const w=new Worker('./src/m3-browser-worker.js');
      const timer=setTimeout(()=>{w.terminate();reject(Error('M3 browser test timeout'));},240000);
      w.onmessage=e=>{
        const {type,result,message}=e.data||{};
        if(type==='error'){clearTimeout(timer);w.terminate();reject(Error(message));}
        if(type==='result'){clearTimeout(timer);w.terminate();resolve(result);}
      };
      w.onerror=e=>{clearTimeout(timer);w.terminate();reject(Error(e.message));};
      w.postMessage({id:1,payload});
    });
  });
  assert.equal(result.solver,'SPGA-M3-20261102-original-python');
  assert.equal(result.warped.length,3);
  assert.ok(result.diagnostics.iterations>=20);
  for(const p of result.warped){
    const L=Math.hypot(p.a1.x-p.a0.x,p.a1.y-p.a0.y,p.a1.z-p.a0.z);
    assert.ok(Math.abs(L-1)<1e-6,'tool length changed');
  }
  assert.deepEqual(errors,[],'page errors');
  console.log('PASS: chromium GitHub Pages worker executes genuine M3 Python and preserves length');
}finally{
  if(browser)await browser.close();
  server.kill('SIGTERM');
}
