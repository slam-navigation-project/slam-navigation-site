import sys, os, math, tempfile, types, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from npz_grid import save_aligned_npz, load_saved_npz, install_map_bundle
from astar_planner import GridMap, plan
from amr_server import Core, Conflict


def make_test_map():
    n=24
    data=np.zeros((n,n), dtype=np.int8)
    data[0,:]=-1; data[-1,:]=-1; data[:,0]=-1; data[:,-1]=-1
    data[3:20,12]=100;data[12,12]=0
    data[5:9,3:8]=100
    return {'stamp':'1','w':n,'h':n,'res':0.1,'ox':-0.8,'oy':-0.8,'yaw':0.0,'data':data.ravel().tolist(),'frame_id':'map'}

for yaw in (0, math.pi/2, -math.pi/3, math.pi/4):
    with tempfile.TemporaryDirectory() as td:
        dest=Path(td)/'my_map.npz'
        raw=make_test_map()
        w,h=save_aligned_npz(dest,raw,(0.1,0.1,yaw))
        with np.load(dest) as arr:
            assert arr['occupancy'].shape==(h,w)
            assert bool(arr['aligned_to_initial_forward'])
            assert abs(float(arr['origin_yaw']) - (yaw-math.pi/2))<1e-7
        m=GridMap(dest, robot_radius=0.05)
        for r,c in [(3,4),(4,3),(m.height//2,m.width//2)]:
            x,y=m.cell_to_world(r,c)
            assert m.world_to_cell(x,y)==(r,c),(yaw,r,c,m.world_to_cell(x,y))
        meta=load_saved_npz(dest)
        assert len(meta['rows'])==h and len(meta['rows'][0])==w
        free=np.argwhere(~m.blocked)
        assert len(free)>50
        startrow,startcol=free[0]; gr,gc=free[-1]
        result=plan(m,m.cell_to_world(startrow,startcol),m.cell_to_world(gr,gc),simplify_path=False)
        if result:
            assert all(m.is_free(*m.world_to_cell(*p)) for p in result)
            assert all(sum(abs(a-b) for a,b in zip(m.world_to_cell(*p),m.world_to_cell(*q)))==1 for p,q in zip(result,result[1:])),result
print('NPZ roundtrip, rotation, grid and A* 4-direction: PASS')

class FakeProc:
    def __init__(self, name, cmd, log_path):
        self.name=name;self.cmd=cmd;self.log_path=log_path;self.p=types.SimpleNamespace(pid=777)
        self.running=False
    def start(self):self.running=True
    def stop(self,*args):self.running=False
    def alive(self):return self.running
class FakeROS:
    def __init__(self):
        self.latest_map=make_test_map()
        self.initial_pose=(0.1,0.1,math.pi/2)
        self.pose=[0.1,0.1,math.pi/2]
    def set_drive(self,*args):pass
    def save_map(self, base, timeout=20.0):
        Path(base+'.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
        Path(base+'.yaml').write_text('image: '+Path(base).name+'.pgm\nresolution: 0.1\norigin: [-0.8, -0.8, 0.0]\n')
        return True
    def serialize(self,base,timeout=20.0):
        Path(base+'.posegraph').write_bytes(b'POSEGRAPH')
        Path(base+'.data').write_bytes(b'DATA')
        return True
with tempfile.TemporaryDirectory() as td:
    fake=FakeROS()
    c=Core(fake,'test.launch.py',td,'fake_nav.py',proc_factory=FakeProc)
    c.mode='mapping'
    # avoid command writes to /tmp? nav start removes state file; safe fake.
    info=c.finish_mapping('demo', radius=0.05, speed=0.08)
    assert all(Path(td,'demo.'+ext).is_file() for ext in ('npz','pgm','yaml','posegraph','data'))
    assert c.mode=='nav'
    assert c.frozen['yaw']==0
    assert len(c.get_map()['rows'])>0
    goal_xy=c.plan_grid.cell_to_world(18,18)
    assert c.plan_grid.is_free(*c.plan_grid.world_to_cell(*goal_xy))
    path=c.preview_goal(list(goal_xy))
    assert path['ok'] and len(path['path'])>0
    assert len(path['path'])>=2
    assert all(sum(abs(a-b) for a,b in zip(c.plan_grid.world_to_cell(*p),c.plan_grid.world_to_cell(*q)))==1 for p,q in zip(path['path'],path['path'][1:])),path
    try:
        c.preview_goal([-9999.0,-9999.0])
    except Conflict:
        pass
    else:raise AssertionError('outside accepted')
    commands = []
    c._write_cmd = lambda kind, points=None: commands.append((kind, points))
    c.send_cmd('go', points=[goal_xy])
    assert commands[-1][0] == 'go' and len(commands[-1][1]) == 1
    try:
        c.send_cmd('go',points=[goal_xy,goal_xy])
    except Conflict:pass
    else:raise AssertionError('multigoal accepted')
print('Core save, 5 files, map, preview and invalid goal rejection: PASS')

# 이미 저장된 파일을 교체하는 동안 새 파일이 비어 있으면 기존 파일 유지
with tempfile.TemporaryDirectory() as td:
    td=Path(td); src=td/'tmp'/'test';src.parent.mkdir();dst=td/'test'
    for ext in ('npz','pgm','yaml','posegraph','data'):
        Path(str(dst)+'.'+ext).write_bytes(b'OLD-'+ext.encode())
        Path(str(src)+'.'+ext).write_bytes(b'NEW-'+ext.encode())
    install_map_bundle(src,dst)
    assert all(Path(str(dst)+'.'+e).read_bytes()==b'NEW-'+e.encode() for e in ('npz','pgm','yaml','posegraph','data'))
    for ext in ('npz','pgm','yaml','posegraph','data'):
        Path(str(src)+'.'+ext).write_bytes(b'NEXT-'+ext.encode())
    Path(str(src)+'.data').unlink()
    try:install_map_bundle(src,dst)
    except FileNotFoundError:pass
    else:raise AssertionError('partial map should not install')
    assert all(Path(str(dst)+'.'+e).read_bytes()==b'NEW-'+e.encode() for e in ('npz','pgm','yaml','posegraph','data'))
print('5-file replace and missing-file preservation: PASS')
