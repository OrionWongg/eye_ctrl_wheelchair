"""Centered Chinese guidance with large targets and explicit capture progress."""
import tkinter as tk
import math
from fusion_control import TURN_THRESHOLD

class GuideWindow:
    def __init__(self, ipad_controls=False, center_only=False):
        self.ipad_panel=None
        self.bottom_height=160 if ipad_controls else 65
        self.root=tk.Tk();self.root.title('眼动与 iPad 控制' if ipad_controls else '眼动控制 · 自动引导')
        sw,sh=self.root.winfo_screenwidth(),self.root.winfo_screenheight()
        self.width=min(1280,sw-80);self.height=min(800,sh-100)
        self.root.geometry(f'{self.width}x{self.height}+{(sw-self.width)//2}+{(sh-self.height)//2}')
        self.placement=((sw-self.width)//2,(sh-self.height)//2)
        self.root.resizable(False,False);self.root.configure(bg='#101827')
        self.root.attributes('-topmost',True)
        self.closed=False;self.action=None;self.was_ready=False
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.root.bind('<Escape>',lambda e:self.close())
        self.root.bind('c',lambda e:self.request('center'))
        self.root.bind('r',lambda e:self.request('full'))
        self.canvas=tk.Canvas(self.root,width=self.width,height=self.height-self.bottom_height,bg='#101827',highlightthickness=0)
        self.canvas.pack()
        if ipad_controls:
            from ipad_panel import IpadPanel
            self.ipad_panel=IpadPanel(self.root)
        bar=tk.Frame(self.root,bg='#101827');bar.pack(fill='x',padx=24,pady=8)
        actions=[('重新确认正中','center'),('退出','exit')] if ipad_controls or center_only else [('重新确认正中','center'),('重新标定左右','full'),('重试保存','save'),('退出','exit')]
        for text,action in actions:
            tk.Button(bar,text=text,font=('Noto Sans CJK SC',13),command=lambda a=action:self.request(a),
                      bg='#263449',fg='white',activebackground='#40506a',relief='flat',padx=14,pady=6).pack(side='left',padx=8)
        self.render('loading',0,'正在启动摄像头和检测器','NO_CONTROLLER',False,0,[],0)
        self.root.after(150,self.center_window)

    def center_window(self):
        if self.closed:return
        # GNOME adds title-bar offsets after mapping; correct the content center.
        self.root.update_idletasks()
        x=(self.root.winfo_screenwidth()-self.root.winfo_width())//2
        y=(self.root.winfo_screenheight()-self.root.winfo_height())//2
        gx=self.placement[0]+x-self.root.winfo_rootx()
        gy=self.placement[1]+y-self.root.winfo_rooty()
        self.root.geometry(f'+{gx}+{gy}')
        self.placement=(gx,gy)

    def request(self,action):
        if action=='exit':self.close()
        else:self.action=action
    def close(self):
        if self.ipad_panel:self.ipad_panel.command('stop')
        self.closed=True
    def poll(self):
        if self.ipad_panel:self.ipad_panel.update()
        if self.closed:return 'exit'
        try:self.root.update_idletasks();self.root.update()
        except tk.TclError:self.closed=True;return 'exit'
        action=self.action;self.action=None;return action
    def render(self,phase,progress,message,controller,valid,h,eyes,step=0,blink_count=0,target='turtle',blockers=None):
        c=self.canvas;c.delete('all');w=self.width;ch=self.height-self.bottom_height
        ready=phase=='ready'
        if ready!=self.was_ready:
            self.root.attributes('-topmost',not ready);self.was_ready=ready
        def text(x,y,value,size=22,color='#eaf0fa',bold=False):
            c.create_text(x,y,text=value,font=('Noto Sans CJK SC',size,'bold' if bold else 'normal'),fill=color)
        titles={'loading':'正在启动','center':'请看正中间的圆点','confirm':'请回到正中间，看圆点',
                'left':'请看左边的圆点','right':'请看右边的圆点','failed':'需要重新采集左右方向'}
        if ready:
            title=('双眨眼，启动小乌龟' if controller=='DISARMED' else
                   '已暂停 · 睁眼后短眨眼一次恢复' if controller=='PAUSED' else
                   '控制中 · 长闭眼可暂停' if controller=='ACTIVE' else '检测已就绪 · 当前为预览')
            if controller=='DISARMED' and blink_count==1:title='已识别一次眨眼 · 请再眨眼一次'
            if target=='wheelchair':title=title.replace('小乌龟','轮椅')
            if not valid:title='暂时未看清眼睛 · 保持睁眼和头部稳定'
            if blockers:
                title='控制器未就绪 · 眨眼暂不驱动轮椅'
        else:title=titles.get(phase,'请跟随引导')
        text(w/2,42,'眼动控制  /  自动引导',18,'#8fa7c8')
        text(w/2,108,title,30,'#ffffff',True)
        if not ready:text(w/2,162,'头部保持不动，只移动眼睛；不用按键',19,'#bac9df')
        x={'left':w*.14,'right':w*.86}.get(phase,w/2);y=self.height/2
        if ready:
            x=w/2+max(-1,min(1,h))*w*.34
            label='左转输入' if h<-TURN_THRESHOLD else '右转输入' if h>TURN_THRESHOLD else '正中输入'
            text(w/2,y-92,label,25,'#6ee7b7' if valid else '#fbbf24',True)
        color='#56d9ff' if not ready else '#6ee7b7' if valid else '#fbbf24'
        c.create_oval(x-46,y-46,x+46,y+46,outline=color,width=3)
        c.create_oval(x-16,y-16,x+16,y+16,fill=color,outline='')
        if phase in ('left','right'):
            end=x+65 if phase=='left' else x-65
            start=w/2-80 if phase=='left' else w/2+80
            c.create_line(start,y,end,y,fill=color,width=10,arrow=tk.LAST,arrowshape=(22,26,12))
        if not ready:
            text(w/2,ch-188,message,20,'#fbbf24' if progress==0 else '#eaf0fa')
            c.create_rectangle(w*.22,ch-145,w*.78,ch-123,fill='#29384f',outline='')
            if progress>0:c.create_rectangle(w*.22,ch-145,w*.22+w*.56*progress,ch-123,fill='#56d9ff',outline='')
            countdown=f'  ·  继续保持约 {(1-progress)*1.5:.1f} 秒' if 0<progress<1 else '  ·  等待画面稳定' if progress==1 else ''
            text(w/2,ch-92,f'稳定采集 {round(progress*100)}%'+countdown,18,'#bac9df')
        else:text(w/2,ch-145,
            '运行中看左右直接转向；回中后可再触发，暂停后眨眼恢复' if target=='wheelchair' else
            '看左／右选择方向；输入异常时小乌龟保持停止',18,'#bac9df')
        if ready and blockers:
            descriptions={
                'IPAD_NOT_ARMED':'请点击下方“启用眼动”' if self.ipad_panel else '请在 iPad 控制窗口点击 Arm Eye Control',
                'IPAD_BLE_NOT_CONNECTED':'等待 iPad 蓝牙连接 nimble',
                'IPAD_HOMING_NOT_READY':'等待 iPad 指针归位完成',
                'IPAD_APP_MODE_REQUIRED':'控制器需要处于 APP 模式',
            }
            text(w/2,ch-92,'；'.join(descriptions.get(b,b) for b in blockers),17,'#fbbf24')
        names={'open':'睁眼','closed':'闭眼','unknown':'未看清'}
        labels=['左眼','右眼']
        text(w/2,ch-39,'    '.join(f'{labels[i]}：{names.get(e,e)}' for i,e in enumerate(eyes)),17,'#8fa7c8')
        self.root.update_idletasks()
    def destroy(self):
        try:self.root.destroy()
        except tk.TclError:pass
