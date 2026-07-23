/**
 * Franka FR3 — 7-DOF IK Engine
 * Numerical Jacobian (damped least squares) + quintic trajectory
 * Joint data from official franka_description kinematics.yaml
 */
(function(){
'use strict';
const DEG=Math.PI/180, RAD=180/Math.PI;

// Exact translations from kinematics.yaml (metres→mm)
// Each entry: [tx,ty,tz, roll] — roll applied as parent X-rotation before joint Z-rotation
const CHAIN=[
  {tx:0,    ty:0,    tz:333, roll:0       }, // joint1
  {tx:0,    ty:0,    tz:0,   roll:-1.5708 }, // joint2
  {tx:0,    ty:-316, tz:0,   roll:1.5708  }, // joint3
  {tx:82.5, ty:0,    tz:0,   roll:1.5708  }, // joint4
  {tx:-82.5,ty:384,  tz:0,   roll:-1.5708 }, // joint5
  {tx:0,    ty:0,    tz:0,   roll:1.5708  }, // joint6
  {tx:88,   ty:0,    tz:0,   roll:1.5708  }, // joint7
];
const EE_Z=107; // mm beyond joint7
const N=7;

const HOME=[0,-45,0,-135,0,90,45].map(d=>d*DEG);

const LIMITS=[
  [-166,166],[-105,105],[-166,166],[-176,-7],[-165,165],[25,265],[-175,175]
].map(([a,b])=>[a*DEG,b*DEG]);

// 4×4 matrix helpers
const I4=()=>[1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
function mul4(A,B){const C=new Array(16).fill(0);for(let r=0;r<4;r++)for(let c=0;c<4;c++)for(let k=0;k<4;k++)C[r*4+c]+=A[r*4+k]*B[k*4+c];return C;}
function Tx(r){const c=Math.cos(r),s=Math.sin(r);return[1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1];}
function Tz(r){const c=Math.cos(r),s=Math.sin(r);return[c,-s,0,0, s,c,0,0, 0,0,1,0, 0,0,0,1];}
function Tr(x,y,z){return[1,0,0,x, 0,1,0,y, 0,0,1,z, 0,0,0,1];}
function pos(T){return[T[3],T[7],T[11]];}
function zax(T){return[T[2],T[6],T[10]];}
function sub3(a,b){return[a[0]-b[0],a[1]-b[1],a[2]-b[2]];}
function cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function norm3(a){return Math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);}

function fk(q){
  const frames=[];
  let T=I4();
  for(let i=0;i<N;i++){
    const {tx,ty,tz,roll}=CHAIN[i];
    T=mul4(T,Tr(tx,ty,tz));
    if(Math.abs(roll)>1e-6) T=mul4(T,Tx(roll));
    T=mul4(T,Tz(q[i]));
    frames.push(T.slice());
  }
  frames.push(mul4(T,Tr(0,0,EE_Z)));
  return frames;
}

function getTCP(q){return pos(fk(q)[N]);}

function jacobian(q){
  const fr=fk(q);
  const pEE=pos(fr[N]);
  return fr.slice(0,N).map(f=>cross(zax(f),sub3(pEE,pos(f))));
}

function inv3(m){
  const[[a,b,c],[d,e,f],[g,h,k]]=m;
  const det=a*(e*k-f*h)-b*(d*k-f*g)+c*(d*h-e*g);
  if(Math.abs(det)<1e-12)return null;
  const r=1/det;
  return[[(e*k-f*h)*r,(c*h-b*k)*r,(b*f-c*e)*r],
         [(f*g-d*k)*r,(a*k-c*g)*r,(c*d-a*f)*r],
         [(d*h-e*g)*r,(b*g-a*h)*r,(a*e-b*d)*r]];
}

function dlsStep(J,dx,lam){
  const JJT=[[0,0,0],[0,0,0],[0,0,0]];
  J.forEach(col=>{for(let r=0;r<3;r++)for(let c=0;c<3;c++)JJT[r][c]+=col[r]*col[c];});
  const l2=lam*lam;
  JJT[0][0]+=l2;JJT[1][1]+=l2;JJT[2][2]+=l2;
  const inv=inv3(JJT); if(!inv)return new Array(N).fill(0);
  const v=[0,0,0];
  for(let r=0;r<3;r++)for(let c=0;c<3;c++)v[r]+=inv[r][c]*dx[c];
  return J.map(col=>col.reduce((s,x,r)=>s+x*v[r],0));
}

function clamp(q){return q.map((v,i)=>Math.max(LIMITS[i][0],Math.min(LIMITS[i][1],v)));}

function solveIK(target,q0){
  let q=clamp(q0?q0.slice():HOME.slice());
  let best=q.slice(),bestErr=Infinity;
  for(let it=0;it<150;it++){
    const tcp=getTCP(q);
    const dx=[target[0]-tcp[0],target[1]-tcp[1],target[2]-tcp[2]];
    const err=norm3(dx);
    if(err<bestErr){bestErr=err;best=q.slice();}
    if(err<2)break;
    const dq=dlsStep(jacobian(q),dx,0.5);
    q=clamp(q.map((v,i)=>v+0.8*dq[i]));
  }
  const tcp=getTCP(best);
  const traj=quintic(q0||HOME,best,50);
  return{angles:best,error:bestErr,tcp,trajectory:traj,success:bestErr<20};
}

function quintic(q0,q1,n){
  const pts=[];
  for(let i=0;i<=n;i++){
    const t=i/n,s=t*t*t*(6*t*t-15*t+10);
    pts.push(q0.map((v,j)=>v+s*(q1[j]-v)));
  }
  return pts;
}

function checkWorkspace(x,y,z){
  const r=Math.sqrt(x*x+y*y+z*z);
  const errs=[];
  if(r<100)errs.push('Too close to base');
  if(r>855)errs.push('Out of reach (max 855mm)');
  if(z<-360)errs.push('Below base limit');
  return{valid:!errs.length,errors:errs};
}

// ── IKController ─────────────────────────────────────────────
// Robot-agnostic router: picks FR3's own math (above, window.PandaIK) or
// KukaIK.js's KukaIK module based on whichever robot PandaSliderControl
// currently has active, so the "Solve IK" button, live TCP readout, and
// LIN motion all work identically for either robot. Each robot module
// exposes the same shape (getTCP, solveIK, checkWorkspace, getLinkPoints,
// HOME, DEG/RAD), which is what makes this router possible without
// per-robot branching anywhere else in the app.
const IKController={
  _q:HOME.slice(),
  _raf:null,

  _activeModule(){
    const robot = (window.PandaSliderControl && window.PandaSliderControl.getActiveRobot) ?
      window.PandaSliderControl.getActiveRobot() : 'fr3';
    return robot === 'kuka' ? window.KukaIK : window.PandaIK;
  },

  init(){
    this._q=this._activeModule().HOME.slice();
    document.getElementById('moveToPose')?.addEventListener('click',()=>this.solve());
    document.getElementById('getCurrentTCP')?.addEventListener('click',()=>this.captureTCP());
    setInterval(()=>this._liveTCP(),150);
    console.log('[IK] ready');
  },

  setSliderInstances(){},

  // Called whenever the active robot changes (see the header switcher UI in
  // index.html) so _q is re-seeded to the NEW robot's HOME instead of
  // staying stale at the old robot's angle vector (which is the wrong
  // length: 7 for FR3, 6 for KUKA).
  onRobotChanged(){
    this._q = this._activeModule().HOME.slice();
  },

  addSliderListener(){
    window._pandaIKSliderCallback=(idx,deg)=>{
      const mod = this._activeModule();
      this._q[idx]=deg*mod.DEG;
    };
  },

  solve(){
    const mod=this._activeModule();
    const robot=(window.PandaSliderControl && window.PandaSliderControl.getActiveRobot) ? window.PandaSliderControl.getActiveRobot() : 'fr3';
    const x=parseFloat(document.getElementById('X')?.value||0);
    const y=parseFloat(document.getElementById('Y')?.value||0);
    const z=parseFloat(document.getElementById('Z')?.value||400);
    const ws=mod.checkWorkspace(x,y,z);
    if(!ws.valid){this._status('⚠ '+ws.errors.join(' | '),'warn');return{success:false,blocked:'workspace'};}
    this._status('Solving…','');
    const res=mod.solveIK([x,y,z],this._q);

    if (typeof window.PandaSafety !== 'undefined') {
      const safety = window.PandaSafety.checkSafety(res.angles, robot);
      if (!safety.safe) {
        const reason = safety.groundViolation ? 'would go below ground' : 'self-collision risk';
        const detail = safety.details[0] ? ' — '+safety.details[0] : '';
        this._status(`⛔ Blocked: ${reason}${detail}`, 'err');
        if (typeof window.termLog === 'function') window.termLog(`LIN blocked: ${reason}${detail}`, 'err');
        return { success:false, blocked:'safety', safety };
      }
    }

    this._status(res.success?`✓ err ${res.error.toFixed(1)}mm`:`⚠ best ${res.error.toFixed(1)}mm`,res.success?'ok':'warn');
    this._run(res.trajectory,res.angles);
    return { success: res.success, blocked: null };
  },

  _run(traj,final){
    if(this._raf)cancelAnimationFrame(this._raf);
    const mod=this._activeModule();
    const cfg=window.PandaSliderControl.JOINT_CONFIG; // ordered names for the ACTIVE robot
    let i=0;
    const step=()=>{
      if(i>=traj.length){this._q=final.slice();return;}
      const map={};
      traj[i++].forEach((v,j)=>{ if(cfg[j]) map[cfg[j].name]=v*mod.RAD; });
      PandaSliderControl.loadAllAngles(map,false);
      this._raf=requestAnimationFrame(step);
    };
    this._raf=requestAnimationFrame(step);
  },

  captureTCP(){
    const mod=this._activeModule();
    const tcp=mod.getTCP(this._q);
    document.getElementById('X').value=tcp[0].toFixed(1);
    document.getElementById('Y').value=tcp[1].toFixed(1);
    document.getElementById('Z').value=tcp[2].toFixed(1);
    this._status(`TCP [${tcp[0].toFixed(0)}, ${tcp[1].toFixed(0)}, ${tcp[2].toFixed(0)}]`,'ok');
  },

  _liveTCP(){
    const mod=this._activeModule();
    const tcp=mod.getTCP(this._q);
    const xEl=document.getElementById('tcpX');
    const yEl=document.getElementById('tcpY');
    const zEl=document.getElementById('tcpZ');
    if(xEl)xEl.textContent=tcp[0].toFixed(0);
    if(yEl)yEl.textContent=tcp[1].toFixed(0);
    if(zEl)zEl.textContent=tcp[2].toFixed(0);
  },

  _status(msg,type){
    const el=document.getElementById('status');
    if(el){el.textContent=msg;el.className='status-msg '+(type||'');}
  },
};

// All link-chain points in world space (mm): base origin, joint1..joint7
// frame origins, then the end-effector. Used by pandaSafety.js for
// ground-plane and self-collision checks — the same FK math as getTCP,
// just returning every intermediate point instead of only the last one.
function getLinkPoints(q){
  const fr = fk(q);
  const pts = [[0,0,0]];
  for (let i = 0; i < fr.length; i++) pts.push(pos(fr[i]));
  return pts;
}

window.PandaIK={getTCP,solveIK,checkWorkspace,getLinkPoints,HOME,LIMITS,DEG,RAD,N};
window.IKController=IKController;
})();
