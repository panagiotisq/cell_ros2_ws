import numpy as np, json, os
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

PLY  = "/home/panosk/panos_recon/colmap/dense/fused.ply"          # UNCLEANED (has all points)
PAIRS_OUT = "/home/panosk/panos_recon/align/pairs.json"
TF_OUT    = "/home/panosk/panos_recon/align/sim3_colmap_to_mocap.json"
MAXP = 12
PALETTE = [[1,0.2,0.2],[1,0.6,0],[1,1,0.1],[0.2,1,0.3],[0.3,0.7,1],[0.7,0.3,1],
           [1,0.3,1],[0.2,1,1],[1,1,1],[0.6,0.4,0.2],[0.5,0.5,0.5],[0.9,0.5,0.7]]

def umeyama(A, B):
    n=len(A); muA=A.mean(0); muB=B.mean(0); A0=A-muA; B0=B-muB
    H=(A0.T@B0)/n; U,D,Vt=np.linalg.svd(H); S=np.eye(3)
    if np.linalg.det(U)*np.linalg.det(Vt)<0: S[2,2]=-1
    R=(U@S@Vt).T
    varA=(A0**2).sum()/n; s=(D*np.diag(S)).sum()/varA; t=muB-s*(R@muA)
    return s,R,t

class Picker:
    def __init__(self):
        self.pcd=o3d.io.read_point_cloud(PLY)
        self.pts=np.asarray(self.pcd.points)
        self.kd=o3d.geometry.KDTreeFlann(self.pcd)
        diag=np.linalg.norm(self.pcd.get_max_bound()-self.pcd.get_min_bound())
        self.r=diag*0.004/3.0
        self.picks=[]                      # dict(xyz,name)

        gui.Application.instance.initialize()
        self.w=gui.Application.instance.create_window("Control-point picker",1700,1000)
        em=self.w.theme.font_size

        self.scene=gui.SceneWidget(); self.scene.scene=rendering.Open3DScene(self.w.renderer)
        self.scene.scene.set_background([0.05,0.05,0.07,1])
        mat=rendering.MaterialRecord(); mat.shader="defaultUnlit"; mat.point_size=3.0
        self.scene.scene.add_geometry("pcd",self.pcd,mat)
        bb=self.pcd.get_axis_aligned_bounding_box()
        self.scene.setup_camera(60,bb,bb.get_center())
        self.scene.set_on_mouse(self._on_mouse)

        self.panel=gui.Vert(0.4*em,gui.Margins(0.5*em,0.5*em,0.5*em,0.5*em))
        self.panel.add_child(gui.Label("CTRL + LEFT-CLICK a point to pick it."))
        self.panel.add_child(gui.Label("Sphere color = its row color below."))
        self.panel.add_child(gui.Label("Fill each point's MOCAP  X  Y  Z :"))

        # pre-create fixed rows (show/hide, never add/remove)
        self.rows=[]
        rowbox=gui.Vert(0.2*em)
        for i in range(MAXP):
            row=gui.Horiz(0.2*em)
            tag=gui.Label(f"P{i+1}"); tag.text_color=gui.Color(*PALETTE[i%len(PALETTE)])
            fx=gui.TextEdit(); fy=gui.TextEdit(); fz=gui.TextEdit()
            fx.placeholder_text="X"; fy.placeholder_text="Y"; fz.placeholder_text="Z"
            row.add_child(tag); row.add_child(fx); row.add_child(fy); row.add_child(fz)
            row.visible=False
            rowbox.add_child(row)
            self.rows.append(dict(row=row,fx=fx,fy=fy,fz=fz))
        self.panel.add_child(rowbox)

        self.panel.add_fixed(0.3*em)
        hb=gui.Horiz(0.4*em)
        for txt,cb in (("Delete last",self._delete_last),("Save pairs",self._save),("RUN ALIGN",self._run)):
            b=gui.Button(txt); b.set_on_clicked(cb); hb.add_child(b)
        self.panel.add_child(hb)
        self.status=gui.Label("picked: 0\n"); self.panel.add_child(self.status)

        self.w.add_child(self.scene); self.w.add_child(self.panel)
        self.w.set_on_layout(self._layout)

    def _layout(self,ctx):
        r=self.w.content_rect; pw=27*self.w.theme.font_size
        self.scene.frame=gui.Rect(r.x,r.y,r.width-pw,r.height)
        self.panel.frame=gui.Rect(r.get_right()-pw,r.y,pw,r.height)

    def _on_mouse(self,event):
        if event.type==gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(gui.KeyModifier.CTRL):
            def depth_cb(depth_image):
                x=int(event.x-self.scene.frame.x); y=int(event.y-self.scene.frame.y)
                d=np.asarray(depth_image)
                if y<0 or x<0 or y>=d.shape[0] or x>=d.shape[1]: return
                z=d[y,x]
                if z==1.0: return
                world=self.scene.scene.camera.unproject(x,y,z,self.scene.frame.width,self.scene.frame.height)
                wv=np.array([world[0],world[1],world[2]])
                _,idx,_=self.kd.search_knn_vector_3d(wv,1)
                p=self.pts[idx[0]]
                gui.Application.instance.post_to_main_thread(self.w,lambda:self._add_pick(p))
            self.scene.scene.scene.render_to_depth_image(depth_cb)
            return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED

    def _add_pick(self,p):
        i=len(self.picks)
        if i>=MAXP: self.status.text=f"max {MAXP} points\n"; return
        name=f"P{i+1}"; col=PALETTE[i%len(PALETTE)]
        s=o3d.geometry.TriangleMesh.create_sphere(self.r); s.translate(p); s.paint_uniform_color(col)
        sm=rendering.MaterialRecord(); sm.shader="defaultUnlit"
        self.scene.scene.add_geometry(name,s,sm)
        self.rows[i]["row"].visible=True
        for f in ("fx","fy","fz"): self.rows[i][f].text_value=""
        self.picks.append(dict(xyz=[float(p[0]),float(p[1]),float(p[2])],name=name))
        self.status.text=f"picked: {len(self.picks)}\n"; self.w.set_needs_layout(); self._autosave()
        print(f"[pick] {name} colmap=({p[0]:.5f},{p[1]:.5f},{p[2]:.5f})",flush=True)

    def _delete_last(self):
        if not self.picks: return
        i=len(self.picks)-1; pk=self.picks.pop()
        self.scene.scene.remove_geometry(pk["name"])
        self.rows[i]["row"].visible=False
        for f in ("fx","fy","fz"): self.rows[i][f].text_value=""
        self.status.text=f"picked: {len(self.picks)}\n"; self.w.set_needs_layout(); self._autosave()
        print(f"[delete] now {len(self.picks)}",flush=True)

    def _collect(self):
        A=[]; B=[]; nm=[]
        for i,pk in enumerate(self.picks):
            A.append(pk["xyz"])
            try: b=[float(self.rows[i]["fx"].text_value),float(self.rows[i]["fy"].text_value),float(self.rows[i]["fz"].text_value)]
            except ValueError: b=None
            B.append(b); nm.append(pk["name"])
        return A,B,nm

    def _autosave(self):
        A,B,nm=self._collect()
        json.dump([{"name":nm[i],"colmap_xyz":A[i],"mocap_xyz":B[i]} for i in range(len(A))],
                  open(PAIRS_OUT,"w"),indent=2)

    def _save(self):
        self._autosave(); self.status.text=f"saved {len(self.picks)} picks ->\n{PAIRS_OUT}\n"; print("[save]",flush=True)

    def _run(self):
        self._autosave(); A,B,nm=self._collect()
        A2=[];B2=[];n2=[]
        for a,b,n in zip(A,B,nm):
            if b is not None: A2.append(a);B2.append(b);n2.append(n)
        if len(A2)<3: self.status.text=f"need >=3 pts w/ mocap coords (have {len(A2)})\n"; return
        A2=np.array(A2); B2=np.array(B2); s,R,t=umeyama(A2,B2)
        pred=(s*(R@A2.T)).T+t; err=np.linalg.norm(pred-B2,axis=1); rms=float(np.sqrt((err**2).mean()))
        T=np.eye(4); T[:3,:3]=s*R; T[:3,3]=t
        json.dump({"scale":float(s),"R":R.tolist(),"t":t.tolist(),"T_colmap_to_mocap":T.tolist(),
                   "rms":rms,"n":len(A2),"names":n2,"per_point_err":err.tolist()},open(TF_OUT,"w"),indent=2)
        lines=[f"n={len(A2)}  scale={s:.5f}  RMS={rms*1000:.1f} mm"]+[f"  {n2[k]}: {err[k]*1000:6.1f} mm" for k in range(len(A2))]
        lines.append(f"saved -> {TF_OUT}")
        self.status.text="\n".join(lines)+"\n"; print("[align] "+" | ".join(lines),flush=True)

Picker()
gui.Application.instance.run()
