'use strict';
const $=id=>document.getElementById(id),colors={left:'#55d8df',right:'#ffbf69'};
let state=null;
function project(v){const[x,y,z]=v;switch($('view').value){case 'top':return[-y,-x];case 'rear':return[-y,-.35*x-.93675*z];default:return[(x-y)*Math.SQRT1_2,-(x+y)*.31368-z*.89622];}}
function draw(){if(!state?.arms)return;const cv=$('robot'),w=cv.clientWidth,h=cv.clientHeight,dpr=window.devicePixelRatio||1;cv.width=w*dpr;cv.height=h*dpr;const c=cv.getContext('2d');c.scale(dpr,dpr);
 const bases=Object.values(state.arms).filter(a=>a.measured).map(a=>a.measured.points[0]);if(!bases.length)return;
 const mid=bases[0].map((v,i)=>bases.reduce((sum,b)=>sum+b[i],0)/bases.length);
 const center=project([mid[0]+.3,mid[1],.4]),scale=Math.min(w/2.5,h/1.8);
 const screen=v=>{const p=project(v);return[w/2+(p[0]-center[0])*scale,h/2+(p[1]-center[1])*scale];};
 function line(pts,color,width,dash=[]){c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);pts.map(screen).forEach((p,i)=>i?c.lineTo(...p):c.moveTo(...p));c.stroke();c.setLineDash([]);}
 for(let n=-10;n<=10;n++){const v=n/10;line([[mid[0]-.5,mid[1]+v,0],[mid[0]+1,mid[1]+v,0]],'#21354b',1);line([[mid[0]+v,mid[1]-1,0],[mid[0]+v,mid[1]+1,0]],'#21354b',1);}
 const origin=[mid[0]-.15,mid[1],0];[[.6,0,0,'#f87171','前 +X'],[0,.4,0,'#4ade80','左 +Y'],[0,0,.35,'#60a5fa','上 +Z']].forEach(a=>{const end=origin.map((v,i)=>v+a[i]);line([origin,end],a[3],3);c.fillStyle=a[3];c.fillText(a[4],...screen(end));});
 for(const [side,a] of Object.entries(state.arms)){for(const kind of ['measured','virtual']){if(!a[kind])continue;c.globalAlpha=kind==='measured'?.4:1;line(a[kind].points,colors[side],kind==='measured'?10:3,kind==='measured'?[7,5]:[]);}c.globalAlpha=1;if(a.measured){c.fillStyle=colors[side];const p=screen(a.measured.points[0]);c.fillText(side+' / '+a.ip,p[0]+8,p[1]+20);}}
}
function render(s){state=s;const single=s.mode==='single-left';
 $('title').textContent=single?'左臂实时预览 · 仅虚拟运动':'双臂实时预览 · 仅虚拟运动';
 $('legend').textContent=(single?'　青色 left / .3':'　青色 left / .3　橙色 right / .2')+'　粗虚线：实测　细实线：虚拟';
 $('instructions').textContent=single?`左手 X：接管左臂虚拟目标 → 再按 X：减速停止并平滑归位到本次接管起点。松开 X 不停止；停止和归位期间 X 锁定，完成后释放再按 X 才能重新接管。平移缩放 ${s.position_scale}，姿态固定。`:'右手 B：接管双臂虚拟目标 → 再按 B：减速停止，随后平滑归位；STOPPING / RETURNING 期间 B 锁定，归位后松开再按 B 接管。松开 B 不停止。左手控制左臂，右手控制右臂。';
 $('source').textContent=`机器人来源：${s.robot_source==='mock'?'MOCK 模拟快照（不是真机实时反馈）':'LIVE 真机只读反馈'} ｜ Quest：${s.quest_source} ｜ 真机命令：禁用`;
 $('status').textContent=s.status+' · '+(s.status==='RETURNING'?(single?'虚拟左臂平滑归位中，X 接管锁定 · ':'虚拟双臂平滑归位中，B 接管锁定 · '):'')+s.reason;$('status').style.background=s.active?'#185044':s.status==='FAULT'?'#752a35':'#24364d';
 $('arms').replaceChildren();for(const [side,a] of Object.entries(s.arms)){const tr=document.createElement('tr');const age=v=>v==null?'无数据':(v*1000).toFixed(0)+' ms';let distance='—';if(a.measured&&a.virtual)distance=(Math.hypot(...a.measured.tcp.slice(0,3).map((r,i)=>r[3]-a.virtual.tcp[i][3]))*1000).toFixed(1)+' mm';
 for(const value of [side+' / '+a.ip+(single?' · X '+(a.clutch_pressed?'按下':'松开'):''),age(a.feedback_age_s)+' / '+age(a.pose_age_s),distance,a.virtual?.q.map(v=>v.toFixed(3)).join('  ')||'—']){const td=document.createElement('td');td.textContent=value;tr.appendChild(td);}$('arms').appendChild(tr);}
 $('events').textContent=(s.events||[]).slice(-5).map(e=>e.action+': '+(e.reason||(single?'X 接管':'B 接管'))).join('\n')+(Object.keys(s.reader_errors||{}).length?'\n'+JSON.stringify(s.reader_errors):'');draw();}
async function poll(){try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(1000)});if(!response.ok)throw Error(response.status);render(await response.json());}catch(e){$('status').textContent='服务断开 / 数据已失效 · 当前画面仅为最后一帧';$('status').style.background='#752a35';}finally{setTimeout(poll,40);}}
$('view').addEventListener('change',draw);window.addEventListener('resize',draw);poll();
