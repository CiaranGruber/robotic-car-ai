#!/usr/bin/env python3
"""Desktop camera-heading experiment. Run with Python; no ROS connection."""
import csv
import math
import sys
import tkinter as tk
from pathlib import Path
from tkinter import ttk, filedialog, messagebox

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from policy.pan_simulation import PanSimulation


class Game:
    def __init__(self, root):
        self.root = root
        root.title('AI4R · 摄像头朝向实验室')
        root.geometry('1100x850')
        root.minsize(850, 720)
        root.configure(bg='#101923')
        self.sim = PanSimulation()
        self.sim.mode = 'ideal'
        self.paused = False
        self.demo = False
        self.keys = set()
        self.vars = {}
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#101923')
        style.configure('TLabel', background='#101923', foreground='#e5eef6', font=('Microsoft YaHei UI', 10))
        style.configure('TButton', font=('Microsoft YaHei UI', 10), padding=7)
        style.configure('TCheckbutton', background='#101923', foreground='#e5eef6')
        outer = ttk.Frame(root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='摄像头朝向实验室', font=('Microsoft YaHei UI', 22, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='选择场地目标方向，再转动车身。左转为正；0° 指向画面上方。纯仿真，不连接小车。').pack(anchor='w', pady=(3,10))
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill='x')
        self.mode = tk.StringVar(value='理想模式')
        box = ttk.Combobox(toolbar, textvariable=self.mode, state='readonly', width=15,
                           values=['理想模式', '项目模式', '无补偿对照'])
        box.pack(side='left', padx=(0,8))
        box.bind('<<ComboboxSelected>>', lambda _: self.restart())
        self.pause_button = ttk.Button(toolbar, text='暂停', command=self.pause)
        self.pause_button.pack(side='left', padx=4)
        ttk.Button(toolbar, text='重置 / 解锁', command=self.restart).pack(side='left', padx=4)
        ttk.Button(toolbar, text='固定轨迹 20 秒', command=self.start_demo).pack(side='left', padx=4)
        ttk.Button(toolbar, text='导出 CSV', command=self.export).pack(side='left', padx=4)
        self.sensor = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text='IMU 有数据', variable=self.sensor).pack(side='left', padx=8)
        main = ttk.Frame(outer)
        main.pack(fill='both', expand=True, pady=10)
        self.canvas = tk.Canvas(main, background='#142330', highlightthickness=0, width=580, height=390)
        self.canvas.pack(side='left', fill='both', expand=True)
        panel = ttk.Frame(main, padding=(18,0,0,0))
        panel.pack(side='right', fill='y')
        self.slider(panel, 'target', '镜头目标朝向（°）', -180, 180, 0)
        self.slider(panel, 'body', '车身朝向（°）', -180, 180, 0)
        ttk.Label(panel, text='点击场地图也能选择目标方向\n左右方向键：缓慢转动车身', justify='left').pack(anchor='w', pady=5)
        self.slider(panel, 'limit', '补偿指令范围 ±（项目默认 0.15）', .02, .3, .15)
        self.slider(panel, 'rate', '指令限速 / 秒', .02, .5, .1)
        self.slider(panel, 'lag', '舵机时间常数（假设，秒）', 0, 1, .15)
        self.slider(panel, 'delay', 'IMU 延迟（假设，秒）', 0, .8, 0)
        ttk.Label(panel, text='标定：中心 0.32；增益 −0.43 / rad\n项目默认补偿范围约 ±20°\n更改模型参数后请重置，便于公平比较。', justify='left').pack(anchor='w', pady=8)
        self.status = ttk.Label(outer, text='', font=('Microsoft YaHei UI', 11, 'bold'))
        self.status.pack(anchor='w')
        self.metrics = ttk.Label(outer, text='')
        self.metrics.pack(anchor='w', pady=4)
        ttk.Label(outer, text='最近 20 秒 · 朝向误差（°）；导出包含当前记录和模型参数，最多保留 10 分钟。').pack(anchor='w')
        self.chart = tk.Canvas(outer, background='#142330', height=140, highlightthickness=0)
        self.chart.pack(fill='x', pady=5)
        ttk.Label(outer, text='青色：车身方向   金色虚线：目标方向   粉色：实际镜头方向\n项目模式直接调用 pan_stabilisation.step；舵机模型与延迟可调，结果不是实车性能证明。', justify='left').pack(anchor='w')
        self.canvas.bind('<Button-1>', self.select_target)
        root.bind('<KeyPress-Left>', lambda _: self.keys.add('left'))
        root.bind('<KeyPress-Right>', lambda _: self.keys.add('right'))
        root.bind('<KeyRelease-Left>', lambda _: self.keys.discard('left'))
        root.bind('<KeyRelease-Right>', lambda _: self.keys.discard('right'))
        root.bind('<FocusOut>', lambda _: self.keys.clear())
        self.tick()

    def slider(self, parent, key, title, low, high, value):
        var = tk.DoubleVar(value=value)
        self.vars[key] = var
        label = ttk.Label(parent, text=f'{title}: {value:.2f}')
        label.pack(anchor='w', pady=(6,0))
        ttk.Scale(parent, variable=var, from_=low, to=high, length=290,
                  command=lambda v: label.configure(text=f'{title}: {float(v):.2f}')).pack(fill='x')
        var.trace_add('write', lambda *_: label.configure(text=f'{title}: {var.get():.2f}'))

    def restart(self):
        self.demo = False
        self.sim.mode = {'理想模式':'ideal','项目模式':'project','无补偿对照':'off'}[self.mode.get()]
        self.sim.reset()
        self.paused = False
        self.pause_button.configure(text='暂停')

    def start_demo(self):
        self.vars['target'].set(0)
        self.vars['body'].set(0)
        self.restart()
        self.demo = True

    def pause(self):
        self.paused = not self.paused
        self.pause_button.configure(text='继续' if self.paused else '暂停')

    def select_target(self, event):
        x, y = self.canvas.winfo_width()/2, self.canvas.winfo_height()/2
        self.vars['target'].set(math.degrees(math.atan2(x-event.x, y-event.y)))

    def export(self):
        if not self.sim.records:
            return
        path = filedialog.asksaveasfilename(defaultextension='.csv', initialfile='pan-experiment.csv',
                                          filetypes=[('CSV','*.csv')])
        if path:
            try:
                with open(path, 'w', newline='', encoding='utf-8-sig') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(self.sim.records[0]))
                    writer.writeheader()
                    writer.writerows(self.sim.records)
            except OSError as exc:
                messagebox.showerror('导出失败', str(exc))

    def tick(self):
        if not self.paused:
            if self.demo:
                t = self.sim.time
                self.vars['body'].set(15*math.sin(2*math.pi*t/10))
                if t >= 20-1e-9:
                    self.demo = False
                    self.pause()
            elif self.keys:
                change = (int('left' in self.keys)-int('right' in self.keys))*.4
                self.vars['body'].set(max(-180,min(180,self.vars['body'].get()+change)))
            if not self.paused:
                for key, value in self.vars.items():
                    setattr(self.sim, key, value.get())
                self.sim.sensor_on = self.sensor.get()
                self.sim.step()
        self.draw()
        self.root.after(20, self.tick)

    def draw(self):
        c = self.canvas
        c.delete('all')
        w,h = c.winfo_width(),c.winfo_height()
        x,y = w/2,h/2
        r = max(40,min(w,h)*.36)
        for deg in range(0,360,30):
            a = math.radians(deg)
            dx,dy = -math.sin(a),-math.cos(a)
            c.create_line(x+dx*r*.93,y+dy*r*.93,x+dx*r,y+dy*r,fill='#566f83')
        c.create_oval(x-r,y-r,x+r,y+r,outline='#566f83')
        c.create_text(x, y-r-18,text='0° / 场地前方',fill='#e5eef6')
        def point(px,py,deg):
            a=math.radians(deg)
            return x+px*math.cos(a)+py*math.sin(a),y-px*math.sin(a)+py*math.cos(a)
        corners=[point(a,b,self.sim.body) for a,b in [(-26,-48),(26,-48),(26,48),(-26,48)]]
        c.create_polygon(*[v for pair in corners for v in pair],fill='#244858',outline='#56d9dd',width=2)
        for deg,color,length,dash in [(self.sim.body,'#56d9dd',r*.7,None),
                                       (self.sim.target,'#ffd477',r,(7,5)),
                                       (self.sim.body+self.sim.pan,'#f29dc7',r*.9,None)]:
            a=math.radians(deg)
            kw=dict(fill=color,width=3,arrow='last')
            if dash:kw['dash']=dash
            c.create_line(x,y,x-math.sin(a)*length,y-math.cos(a)*length,**kw)
        c.create_oval(x-8,y-8,x+8,y+8,fill='#f29dc7',outline='')
        rows=self.sim.records
        if not rows:return
        last=rows[-1]
        errors=[abs(z['error_deg']) for z in rows]
        self.status.configure(text=('已暂停 · ' if self.paused else '')+self.sim.status)
        self.metrics.configure(text=f"时间 {self.sim.time:.1f}s  |  当前误差 {last['error_deg']:+.2f}°  |  最大误差 {max(errors):.2f}°  |  平均绝对误差 {sum(errors)/len(errors):.2f}°  |  云台指令 {self.sim.command:.3f}")
        p=self.chart;p.delete('all');pw,ph=p.winfo_width(),p.winfo_height()
        recent=[z for z in rows if z['time_s']>=self.sim.time-20]
        bound=max(5,max(abs(z['error_deg']) for z in recent))
        p.create_line(45,ph/2,pw-10,ph/2,fill='#566f83')
        for value in (-bound,0,bound):
            p.create_text(23,ph/2-value/bound*(ph/2-15),text=f'{value:.0f}',fill='#e5eef6')
        pts=[]
        start=max(0,self.sim.time-20)
        for row in recent:
            pts.extend([45+(row['time_s']-start)/20*(pw-55),ph/2-row['error_deg']/bound*(ph/2-15)])
        if len(pts)>=4:p.create_line(*pts,fill='#f29dc7',width=2)
        p.create_text(48,ph-8,text=f'{start:.0f}s',anchor='w',fill='#e5eef6')
        p.create_text(pw-12,ph-8,text=f'{start+20:.0f}s',anchor='e',fill='#e5eef6')


if __name__ == '__main__':
    root=tk.Tk()
    Game(root)
    root.mainloop()
