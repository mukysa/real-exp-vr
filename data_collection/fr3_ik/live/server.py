"""Local visualization server. There is deliberately no robot command path."""
import argparse
from dataclasses import fields
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from queue import Queue, Empty
import subprocess
import threading
import time
import numpy as np
from .engine import PreviewEngine, SIDES
from data_collection.meta_quest.receiver import PoseReceiver, ReceivedPose
from data_collection.meta_quest.protocol import PoseSample

ROOT=Path(__file__).resolve().parents[3]

class FeedbackReader:
    def __init__(self, helper, side, ip, queue):
        self.side=side; self.queue=queue
        self.process=subprocess.Popen([str(helper),ip],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()
        threading.Thread(target=self.errors,daemon=True).start()
    def errors(self):
        for line in self.process.stderr:self.queue.put(('error',self.side,line.strip(),time.monotonic()))
    def read(self):
        try:
            for line in self.process.stdout:
                self.queue.put(('state',self.side,json.loads(line),time.monotonic()))
        except Exception as exc:self.queue.put(('error',self.side,str(exc),time.monotonic()))
        finally:self.queue.put(('error',self.side,'read-only reader exited',time.monotonic()))
    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-source',choices=['mock','franka'],default='mock')
    p.add_argument('--single-left',action='store_true',help='left arm only; never open a right-arm reader')
    p.add_argument('--position-scale',type=float,default=.5)
    p.add_argument('--connect-readonly',action='store_true',help='Permit read connections for selected arms (left only with --single-left)')
    p.add_argument('--demo-recording',type=Path,help='Replay recorded Quest poses in real time; mock feedback only')
    p.add_argument('--snapshot',type=Path,default=ROOT/'recordings/robot_state/initial-20261021-103516/snapshot.json')
    p.add_argument('--helper',type=Path,default=ROOT/'outputs/fr3_ik/live-tools/franka_readonly')
    p.add_argument('--pose-port',type=int,default=8001);p.add_argument('--http-port',type=int,default=8765)
    p.add_argument('--duration',type=float,default=0)
    args=p.parse_args()
    if args.robot_source=='franka' and not args.connect_readonly:p.error('franka requires --connect-readonly')
    if args.robot_source=='franka' and args.demo_recording:p.error('recorded Quest input is restricted to mock feedback')
    if args.robot_source=='franka' and not args.helper.is_file():p.error('Build the read-only helper first')
    if not 0 < args.position_scale <= 1:p.error('--position-scale must be in (0, 1]')
    engine=PreviewEngine(ROOT/'calibration/matrix.md',single_left=args.single_left,position_scale=args.position_scale);queue=Queue(maxsize=256);readers=[];receiver=None
    snapshot=json.loads(args.snapshot.read_text())['arms'] if args.robot_source=='mock' else None
    rows=[]
    if args.demo_recording:
        rows=[r for r in map(json.loads,args.demo_recording.read_text().splitlines()) if r.get('record_type')=='pose']
        if not rows:p.error('Recording contains no poses')
    latest={'body':b'{}'}
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path=='/api/state':body=latest['body'];mime='application/json'
            elif self.path in ('/','/viewer.js'):
                path=Path(__file__).with_name('viewer.html' if self.path=='/' else 'viewer.js')
                body=path.read_bytes();mime='text/html; charset=utf-8' if self.path=='/' else 'text/javascript'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def log_message(self,*args):pass
    http=ThreadingHTTPServer(('127.0.0.1',args.http_port),Handler)
    threading.Thread(target=http.serve_forever,daemon=True).start()
    logdir=ROOT/'outputs/fr3_ik/live-sessions';logdir.mkdir(parents=True,exist_ok=True)
    logpath=logdir/(time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns())+'.jsonl')
    start=time.monotonic();cursor=0;fatal=None;reader_errors={};last_logged=None
    sample_fields={f.name for f in fields(PoseSample)}
    try:
        if not rows:
            receiver=PoseReceiver(port=args.pose_port);subscription=receiver.subscribe(512);receiver.start()
        if args.robot_source=='franka':
            targets=[('left','172.16.0.3')] if args.single_left else [('left','172.16.0.3'),('right','172.16.0.2')]
            for s,ip in targets:readers.append(FeedbackReader(args.helper,s,ip,queue))
        print(f'Preview: http://127.0.0.1:{http.server_port} | robot={args.robot_source} | mode={"single-left" if args.single_left else "dual"} | position_scale={args.position_scale:g} | commands=DISABLED | log={logpath}',flush=True)
        with logpath.open('x') as log:
            log.write(json.dumps(dict(type='metadata',robot_source=args.robot_source,mode='single-left' if args.single_left else 'dual',position_scale=args.position_scale,recording=str(args.demo_recording),hardware_commands_enabled=False))+'\n')
            while not args.duration or time.monotonic()-start<args.duration:
                now=time.monotonic()
                try:
                    if snapshot:
                        for s in engine.sides:engine.update_feedback(s,snapshot[s]['q_rad'],now,'MOCK snapshot (not live robot)')
                    for _ in range(512):
                        try:kind,s,data,stamp=queue.get_nowait()
                        except Empty:break
                        if kind=='error':reader_errors[s]=data;engine.feedback.pop(s,None);engine.stop('robot reader error')
                        elif s not in reader_errors:
                            tool=np.asarray(data['F_T_EE'],float).reshape(4,4,order='F')
                            if not np.allclose(tool,engine.kin[s].tool,atol=1e-5):raise ValueError(s+' robot TCP differs from model')
                            engine.update_feedback(s,data['q'],stamp,'LIVE read-only libfranka')
                    if rows:
                        while cursor<len(rows) and rows[cursor]['received_monotonic_s']-rows[0]['received_monotonic_s']<=now-start:
                            r=rows[cursor];stamp=start+r['received_monotonic_s']-rows[0]['received_monotonic_s']
                            if fatal is None:engine.pose(ReceivedPose(PoseSample(**{k:v for k,v in r.items() if k in sample_fields}),stamp,r['sequence'],r['connection_id']),now)
                            cursor+=1
                    else:
                        if subscription.dropped_records:fatal='Quest queue overflow: restart preview'
                        for _ in range(512):
                            try:r=subscription.get_nowait()
                            except Empty:break
                            if now-r.received_monotonic_s>engine.stale_s:fatal='Quest backlog: restart preview'
                            if fatal is None:engine.pose(r,time.monotonic())
                    if fatal:engine.stop(fatal)
                    now=time.monotonic()
                    engine.tick(now)
                except Exception as exc:
                    fatal=f'{type(exc).__name__}: {exc}';engine.stop(fatal)
                state=engine.state(now);state.update(robot_source=args.robot_source,quest_source='REPLAY' if rows else 'LIVE TCP',reader_errors=reader_errors,fatal_error=fatal)
                if fatal:state['status']='FAULT';state['reason']=fatal
                latest['body']=json.dumps(state,allow_nan=False).encode()
                key=(state['status'],state['reason'])
                if key!=last_logged:
                    log.write(json.dumps(dict(type='transition',elapsed_s=now-start,**state))+'\n');log.flush();last_logged=key
                if args.single_left:
                    log.write(json.dumps(dict(type='sample',elapsed_s=now-start,**state),allow_nan=False)+'\n')
                time.sleep(max(0,.02-(time.monotonic()-now)))
    except KeyboardInterrupt:pass
    finally:
        for reader in readers:reader.close()
        if receiver:receiver.close()
        http.shutdown();http.server_close()

if __name__=='__main__':main()
