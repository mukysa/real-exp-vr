// Browser-independent smoke test: execute real viewer code against DOM/canvas stubs.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const data=JSON.parse(html.match(/<script id="report-data" type="application\/json">([\s\S]*?)<\/script>/)[1]);
const source=html.match(/<\/script><script>([\s\S]*?)<\/script>/)[1];
let strokes=0;
const ctx=new Proxy({}, {get:(o,k)=>o[k]||((...args)=>{if(k==='lineTo'||k==='moveTo'||k==='arc'){assert(args.every(Number.isFinite),`Nonfinite canvas ${k}`);strokes++;}}),set:(o,k,v)=>(o[k]=v,true)});
const defaults={view:'iso',metric:'q',speed:'1'};const elements={};
for(const id of [...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]))elements[id]={value:defaults[id]||'',checked:['desired','actual'].includes(id),textContent:id==='report-data'?JSON.stringify(data):'',clientWidth:640,clientHeight:420,append(o){if(!this.value)this.value=o.value;},getContext(){return ctx;}};
const context={document:{getElementById:id=>{assert(elements[id],`Unknown element ${id}`);return elements[id];},createElement:()=>({})},window:{devicePixelRatio:1,addEventListener(){}},requestAnimationFrame(){},console};
vm.createContext(context);vm.runInContext(source,context);
for(const name of Object.keys(data.summary.scenarios)){elements.scenario.value=name;elements.scenario.onchange();elements.scrub.value=20;elements.scrub.oninput();for(const metric of ['q','velocity','acceleration','position_error_m','rotation_error_rad','solve_time_s']){elements.metric.value=metric;elements.metric.onchange();}for(const view of ['iso','top','side','front']){elements.view.value=view;elements.view.onchange();}elements.zoom.checked=true;elements.zoom.onchange();elements.candidate.checked=true;elements.candidate.onchange();elements.failure.onclick();elements.next.onclick();elements.prev.onclick();elements.play.onclick();vm.runInContext('animate(0);animate(100);',context);}
assert(strokes>1000);assert(elements.details.textContent.includes('state'));console.log(`Viewer smoke passed: ${Object.keys(data.summary.scenarios).length} scenarios, ${strokes} finite drawing operations`);
