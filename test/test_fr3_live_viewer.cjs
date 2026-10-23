const fs=require('fs'),vm=require('vm'),assert=require('assert');
const ctx=new Proxy({}, {get:(o,k)=>o[k]||(()=>{}),set:(o,k,v)=>(o[k]=v,true)});
const elements={};function el(id){return elements[id]||(elements[id]={value:'right-rear',style:{},clientWidth:1000,clientHeight:600,getContext:()=>ctx,addEventListener(){},replaceChildren(){},appendChild(){}});}
const arm={ip:'172.16.0.3',feedback_age_s:0,pose_age_s:0,measured:{q:[0,0,0,0,0,0,0],points:[[0,0,0],[0,0,1]],tcp:[[1,0,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]]}};arm.virtual=arm.measured;
const state={status:'READY',reason:'test',robot_source:'mock',quest_source:'LIVE TCP',arms:{left:arm,right:{...arm,ip:'172.16.0.2'}},events:[]};
const sandbox={document:{getElementById:el,createElement:()=>({appendChild(){}})},window:{devicePixelRatio:1,addEventListener(){}},AbortSignal,fetch:()=>new Promise(()=>{}),setTimeout(){},Math};
vm.createContext(sandbox);vm.runInContext(fs.readFileSync('data_collection/fr3_ik/live/viewer.js','utf8'),sandbox);
sandbox.fixture=state;for(const view of ['right-rear','rear','top']){el('view').value=view;vm.runInContext('render(fixture)',sandbox);}
assert(el('source').textContent.includes('MOCK'));assert(el('status').textContent.includes(state.status));console.log('Live viewer: three projections and mock/status labels passed');
sandbox.fixture={...state,mode:'single-left',position_scale:.5,arms:{left:arm},events:[{action:'engage'}]};
vm.runInContext('render(fixture)',sandbox);
assert(el('title').textContent.includes('左臂'));
assert(el('instructions').textContent.includes('左手 X'));
assert(el('instructions').textContent.includes('0.5'));
assert(!el('legend').textContent.includes('right'));
assert(el('events').textContent.includes('X 接管'));
console.log('Single-left viewer: X, half scale and left-only labels passed');
