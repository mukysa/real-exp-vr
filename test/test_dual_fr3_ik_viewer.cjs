const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const requestedTime=process.argv[3];
const data=JSON.parse(html.match(/<script id="report-data" type="application\/json">([\s\S]*?)<\/script>/)[1]);
const source=html.match(/<script id="report-data" type="application\/json">[\s\S]*?<\/script><script>([\s\S]*?)<\/script>/)[1];
let draws=0,robotStrokes=0;
const ctx=new Proxy({}, {get:(o,k)=>o[k]||((...args)=>{if(k==='stroke'&&o.lineWidth===5)robotStrokes++;if(k==='lineTo'||k==='moveTo'||k==='arc'){assert(args.every(Number.isFinite),`Nonfinite canvas ${k}`);draws++;}}),set:(o,k,v)=>(o[k]=v,true)});
const defaults={view:'iso',metric:'symmetry',speed:'1',arm:'right'};const elements={};
for(const id of [...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]))elements[id]={value:defaults[id]||'',checked:['left','right','desired','actual'].includes(id),textContent:'',clientWidth:640,clientHeight:420,getContext(){return ctx;}};
elements['report-data'].textContent=JSON.stringify(data);
const context={document:{getElementById:id=>{if(id==='show_hold'&&!elements[id])return null;assert(elements[id],`Unknown element ${id}`);return elements[id];}},window:{devicePixelRatio:1,addEventListener(){}},location:{hash:requestedTime?`#t=${requestedTime}&view=rear-right`:''},URLSearchParams,requestAnimationFrame(){}};
vm.createContext(context);vm.runInContext(source,context);
if(requestedTime){assert(Number(elements.scrub.value)>0,`Hash time ${requestedTime} did not seek into the replay`);if(html.includes('后方看向前（图 1 方向）'))assert.strictEqual(elements.view.value,'rear-right','Hash view did not select the requested camera');}
for(const metric of ['symmetry','q','q_desired_rad','joint_lag','velocity','acceleration','position_error_m','rotation_error_rad','grasp','solve_time_s']){elements.metric.value=metric;elements.metric.onchange();}
for(const view of ['rear','rear-right','front','iso','top','side']){elements.view.value=view;elements.view.onchange();}
if(html.includes('后方看向前（图 1 方向）')){
 elements.view.value='rear';
 const rear=vm.runInContext('project([0,1,0])',context);assert(rear[0]<0,'Left must project to screen left from rear');
 const up=vm.runInContext('project([0,0,1])',context);assert(up[1]<0);
 elements.view.value='front';assert(vm.runInContext('project([0,1,0])[0]',context)>0,'Front must reverse screen left');
 elements.view.value='top';assert(vm.runInContext('project([1,0,0])[1]',context)<0,'Forward must point up on top view');
 elements.view.value='rear-right';assert(vm.runInContext('project([0,1,0])[0]',context)<0);
 console.log('Rear/front/top projection directions passed');
}
elements.mirror.checked=true;elements.mirror.onchange();elements.scrub.value='725';elements.scrub.oninput();elements.failure.onclick();elements.next.onclick();elements.prev.onclick();elements.play.onclick();vm.runInContext('animate(0);animate(100);',context);
assert(draws>1000);assert(elements.details.textContent.includes('left'));
if(data.strict){assert(elements.summary.textContent.includes('engage_mode'));for(const e of data.summary.control_events)assert(elements.summary.textContent.includes(e.action));}
else assert(elements.summary.textContent.includes('symmetry'));
console.log(`Dual viewer smoke passed: ${data.timeline.length} timeline events, ${draws} finite drawing operations`);

if(data.summary.robot_display_windows){
  const windows=data.summary.robot_display_windows;
  function seek(index){robotStrokes=0;elements.scrub.value=String(index);elements.scrub.oninput();return JSON.parse(elements.details.textContent);}
  const times=[0,data.timeline.length-1];
  for(const w of windows){for(const t of [w.start_s-0.01,w.start_s,w.start_s+1,(w.end_s??data.timeline.at(-1).time)-0.01,w.end_s]){if(t!==null){let i=data.timeline.findIndex(f=>f.time>=t);if(i>=0)times.push(i);}}}
  for(const i of times){const d=seek(i),expected=windows.some(w=>d.time_s>=w.start_s&&(w.end_s===null||d.time_s<w.end_s));assert.strictEqual(d.robot_visible,expected);assert.strictEqual(robotStrokes>0,expected,`Unexpected arm strokes at ${d.time_s}`);assert(d.arms.left.controller_world&&d.arms.right.controller_world);}
  elements.left.checked=false;elements.right.checked=false;elements.left.onchange();
  elements.left.checked=true;elements.right.checked=true;elements.right.onchange();
  console.log('Engagement visibility boundaries and live controllers passed');
  if(elements.show_hold){elements.show_hold.checked=true;elements.show_hold.onchange();const d=seek(data.timeline.length-1);assert(d.robot_visible);assert(robotStrokes>0);assert('q_reference_rad' in d.arms.left);assert.strictEqual(d.arms.left.measured_joint_state,null);console.log('Stop inspection and desired/reference/absent feedback fields passed');}
}
