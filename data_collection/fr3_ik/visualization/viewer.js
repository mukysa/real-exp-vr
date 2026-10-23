'use strict';
const DATA=JSON.parse(document.getElementById('report-data').textContent);
const $=id=>document.getElementById(id), colors=['#60a5fa','#fbbf24','#a78bfa','#f87171','#34d399','#fb923c','#e879f9'];
let rows=[],index=0,playing=false,lastTime=null,elapsed=0;
for(const name of Object.keys(DATA.summary.scenarios)){const o=document.createElement('option');o.value=name;o.textContent=name+(DATA.summary.scenarios[name].passed?' ✓':' ✗');$('scenario').append(o);}
function setupCanvas(id){const canvas=$(id),dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=w*dpr;canvas.height=h*dpr;const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);return {c,w,h};}
function setScene(){rows=DATA.samples.filter(r=>r.scenario===$('scenario').value).map(r=>r.desired_world?{...r,desired:r.desired_world,actual:r.actual_world,points:r.points_world,candidate_points:r.candidate_points_world||null}:r);index=0;elapsed=0;playing=false;$('play').textContent='播放';$('scrub').max=rows.length-1;draw();}
function line(c,points,color,width=2,dash=[]){if(!points.length)return;c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);points.forEach((p,i)=>i?c.lineTo(...p):c.moveTo(...p));c.stroke();c.setLineDash([]);}
function position(T){return T.slice(0,3).map(r=>r[3]);}
function projectRaw(p){const [x,y,z]=p;switch($('view').value){case'top':return [x,-y];case'side':return[x,-z];case'front':return[DATA.summary.display_frame==='tabletop_world'?-y:y,-z];default:return[(x-y)*.7071,(x+y)*.35-z*.86];}}
function robot(){const {c,w,h}=setupCanvas('robot'),r=rows[index],zoom=$('zoom').checked,world=DATA.summary.display_frame==='tabletop_world',base=DATA.summary.world_from_base;
 const bx=world?base[0][3]:0,by=world?base[1][3]:0;
 let cloud=rows.flatMap(s=>[position(s.actual),position(s.desired)]).filter(p=>p.every(Number.isFinite)&&Math.hypot(...p)<1.5);
 if(!zoom)cloud=world?cloud.concat(rows.flatMap(s=>s.points),[[bx-.4,by-.4,0],[bx+.4,by+.4,0],[bx,by,.8]]):cloud.concat(r.points,[[0,0,0],[.8,0,0],[0,.8,0],[0,0,1]]);
 const projected=cloud.map(projectRaw),xs=projected.map(p=>p[0]),ys=projected.map(p=>p[1]);
 const lo=[Math.min(...xs),Math.min(...ys)],hi=[Math.max(...xs),Math.max(...ys)],scale=Math.min((w-90)/Math.max(hi[0]-lo[0],zoom?.045:.4),(h-90)/Math.max(hi[1]-lo[1],zoom?.045:.4));
 const center=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2];const p=v=>{const t=projectRaw(v);return[w/2+(t[0]-center[0])*scale,h/2+(t[1]-center[1])*scale];};
 function axes(T,len,width=2){const origin=position(T);for(let j=0;j<3;j++){const end=origin.map((v,i)=>v+T[i][j]*len);line(c,[p(origin),p(end)],['#f87171','#4ade80','#60a5fa'][j],width);}}
 if(!zoom){if(world){const xmin=Math.floor((bx-.4)*10)/10,xmax=xmin+.8,ymin=Math.floor((by-.4)*10)/10,ymax=ymin+.8;for(let n=0;n<=8;n++){line(c,[p([xmin+n*.1,ymin,0]),p([xmin+n*.1,ymax,0])],'#22354b',1);line(c,[p([xmin,ymin+n*.1,0]),p([xmax,ymin+n*.1,0])],'#22354b',1);}axes([[1,0,0,bx],[0,1,0,by],[0,0,1,0],[0,0,0,1]],.12);axes(base,.12,3);line(c,[p([bx,by,0]),p(position(base))],'#8798ad',3,[4,3]);}else{for(let n=-2;n<=8;n++){line(c,[p([n*.1,-.2,0]),p([n*.1,.8,0])],'#22354b',1);line(c,[p([-.2,n*.1,0]),p([.8,n*.1,0])],'#22354b',1);}axes([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],.15);}}
 if($('desired').checked)line(c,rows.map(s=>p(position(s.desired))),'#f9ba55',2,[5,4]);
 if($('actual').checked)line(c,rows.map(s=>p(position(s.actual))),'#4adacb',2);
 line(c,r.points.map(p),'#b3c8e3',5);for(const joint of r.points){const xy=p(joint);c.beginPath();c.arc(...xy,4,0,Math.PI*2);c.fillStyle='#d8e6f7';c.fill();}
 if($('candidate').checked&&r.candidate_points)line(c,r.candidate_points.map(p),'#f087d4',3,[4,4]);
 axes(r.actual,zoom?.009:.06,3);axes(r.desired,zoom?.012:.08,1);
 for(const [T,color]of [[r.desired,'#f9ba55'],[r.actual,'#4adacb']]){c.beginPath();c.arc(...p(position(T)),6,0,Math.PI*2);c.strokeStyle=color;c.lineWidth=2;c.stroke();}
 // Schematic parallel fingers, always attached to the FK TCP orientation.
 const t=position(r.actual),f=(a,b)=>t.map((x,i)=>x+r.actual[i][0]*a+r.actual[i][2]*b);
 line(c,[p(f(-.018,-.025)),p(f(.018,-.025))],'#e5edf5',2);for(const a of [-.018,.018])line(c,[p(f(a,-.025)),p(f(a,0))],'#e5edf5',2);
 c.fillStyle='#9dadc3';c.font='12px system-ui';c.fillText(zoom?'TCP 放大 · 目标坐标轴细线 / FK 坐标轴粗线':world?'桌面坐标 · 网格 0.1 m · 基座坐标轴随安装倾斜':'模型骨架 · 网格 0.1 m',12,20);
 if(Math.hypot(...position(r.desired))>=1.5)c.fillText('目标在视野外：不可达测试',12,40);
}
function chart(){const {c,w,h}=setupCanvas('chart'),key=$('metric').value,vector=['q','velocity','acceleration'].includes(key),factor=key==='position_error_m'||key==='solve_time_s'?1000:key==='rotation_error_rad'?180/Math.PI:1;
 const series=vector?Array.from({length:7},(_,j)=>rows.map(r=>r[key][j])):[rows.map(r=>r[key]*factor)];
 let limits=[];const L=DATA.summary.limits;if(key==='q')limits=[...L.lower,...L.upper];if(key==='velocity'||key==='acceleration')limits=[-Math.max(...L[key]),-Math.min(...L[key]),Math.min(...L[key]),Math.max(...L[key])];if(key==='position_error_m')limits=[L.position_error_m*1000];if(key==='rotation_error_rad')limits=[L.rotation_error_rad*180/Math.PI];if(key==='solve_time_s')limits=[1000/DATA.summary.hz];
 const vals=series.flat().concat(limits),lo=Math.min(0,...vals),hi=Math.max(...vals,lo+1e-6),pad=(hi-lo)*.08;
 const x=t=>55+(t-rows[0].time)/(rows.at(-1).time-rows[0].time||1)*(w-75),y=v=>h-35-(v-lo+pad)/(hi-lo+2*pad)*(h-60);
 c.font='11px system-ui';for(let n=0;n<5;n++){const val=lo+(hi-lo)*n/4;line(c,[[55,y(val)],[w-20,y(val)]],'#293b52',1);c.fillStyle='#9dadc3';c.fillText(val.toFixed(3),1,y(val)+4);}
 limits.forEach((v,j)=>line(c,[[55,y(v)],[w-20,y(v)]],key==='q'?colors[j%7]:'#ff9976',1,[3,5]));
 series.forEach((s,j)=>line(c,s.map((v,i)=>[x(rows[i].time),y(v)]),colors[j],1.5));
 const cursor=x(rows[index].time);line(c,[[cursor,15],[cursor,h-35]],'#fff',1);c.fillStyle='#9dadc3';c.fillText(rows[0].time.toFixed(2)+' s',55,h-10);c.fillText(rows.at(-1).time.toFixed(2)+' s',w-65,h-10);
 $('chartLegend').innerHTML=vector?colors.map((color,j)=>`<span style="color:${color};margin-right:12px">J${j+1}</span>`).join(''):'实线：逐帧值 · 虚线：阈值';
}
function draw(){if(!rows.length)return;index=Math.max(0,Math.min(rows.length-1,index));const r=rows[index];$('scrub').value=index;$('frame').textContent=`${index+1}/${rows.length} · ${r.time.toFixed(3)} s`;
 $('details').textContent=JSON.stringify({state:r.state,reason:r.reason,detail:r.detail,event:r.event,control_eligible:r.control_eligible,all_control_checks_passed:r.all_control_checks_passed,display_frame:DATA.summary.display_frame||'robot_base',display_target_position_m:position(r.desired),display_fk_position_m:position(r.actual),position_error_mm:r.position_error_m*1000,rotation_error_deg:r.rotation_error_rad*180/Math.PI,solve_ms:r.solve_time_s*1000,period_exceeded:r.period_exceeded,checks:r.checks,q_rad:r.q},null,2);
 const s=DATA.summary;$('summary').textContent=JSON.stringify({overall_passed:s.passed,scenario:s.scenarios[$('scenario').value],hz:s.hz,solver_time_ms:s.solver_time_ms,solve_period_overruns:s.solve_period_overruns,attempted_solves:s.attempted_solves,scope:s.timing_scope},null,2);robot();chart();}
$('scenario').onchange=setScene;$('scrub').oninput=()=>{index=+$('scrub').value;draw();};$('prev').onclick=()=>{index--;draw();};$('next').onclick=()=>{index++;draw();};$('failure').onclick=()=>{let next=rows.findIndex((r,i)=>i>index&&(!r.success||r.all_control_checks_passed===false));if(next<0)next=rows.findIndex(r=>!r.success||r.all_control_checks_passed===false);if(next>=0)index=next;draw();};$('play').onclick=()=>{if(index===rows.length-1)index=0;playing=!playing;elapsed=0;$('play').textContent=playing?'暂停':'播放';draw();};
for(const id of ['view','desired','actual','candidate','zoom','metric'])$(id).onchange=draw;
window.addEventListener('resize',draw);
function animate(now){if(lastTime!==null&&playing){elapsed+=(now-lastTime)/1000*+$('speed').value;while(index<rows.length-1&&elapsed>=rows[index+1].time-rows[index].time){elapsed-=rows[index+1].time-rows[index].time;index++;}if(index===rows.length-1){playing=false;$('play').textContent='播放';}draw();}lastTime=now;requestAnimationFrame(animate);}
setScene();requestAnimationFrame(animate);
