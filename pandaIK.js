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

// ── Orientation (RPY) support for the IK panel's "Apply" button ─────────
// Builds a target rotation matrix from roll/pitch/yaw (radians, applied
// as R = Rz(yaw)·Ry(pitch)·Rx(roll) — the standard aircraft convention)
// and extracts the 3x3 rotation part of a 4x4 transform.
function rot3(T){return[[T[0],T[1],T[2]],[T[4],T[5],T[6]],[T[8],T[9],T[10]]];}
function matMul3(A,B){const C=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)C[i][j]+=A[i][k]*B[k][j];return C;}
function transpose3(A){return[[A[0][0],A[1][0],A[2][0]],[A[0][1],A[1][1],A[2][1]],[A[0][2],A[1][2],A[2][2]]];}
function Rx3(a){const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]];}
function Ry3(a){const c=Math.cos(a),s=Math.sin(a);return[[c,0,s],[0,1,0],[-s,0,c]];}
function Rz3(a){const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]];}
function buildR(roll,pitch,yaw){return matMul3(matMul3(Rz3(yaw),Ry3(pitch)),Rx3(roll));}

// Orientation error as a rotation vector (axis * angle, radians) — this is
// the mathematically correct error signal to pair with the angular-
// velocity Jacobian below (jacobian6's rows 4-6), unlike a naive
// Euler-angle subtraction, which is a DIFFERENT representation from what
// the Jacobian's rows actually differentiate and converges poorly or not
// at all (verified against a naive Euler-difference version, which failed
// to converge even for modest targets — this axis-angle version converges
// cleanly, including for large reorientations).
function rotVecError(Rtarget,Rcur){
  const Rerr=matMul3(Rtarget,transpose3(Rcur));
  const tr=Rerr[0][0]+Rerr[1][1]+Rerr[2][2];
  const ang=Math.acos(Math.max(-1,Math.min(1,(tr-1)/2)));
  if(ang<1e-8) return [0,0,0];
  const s=2*Math.sin(ang);
  return [(Rerr[2][1]-Rerr[1][2])/s*ang,(Rerr[0][2]-Rerr[2][0])/s*ang,(Rerr[1][0]-Rerr[0][1])/s*ang];
}

// Generic square-matrix inverse (Gauss-Jordan). The original 3D solver
// below uses a closed-form 3x3 inverse (inv3); this generalized version
// is only used for the 6D (position+orientation) solve so it doesn't
// touch the already-verified position-only path at all.
function invN(A){
  const n=A.length;
  const M=A.map((row,i)=>{const I=new Array(n).fill(0);I[i]=1;return row.concat(I);});
  for(let i=0;i<n;i++){
    let piv=i;
    for(let k=i+1;k<n;k++) if(Math.abs(M[k][i])>Math.abs(M[piv][i])) piv=k;
    [M[i],M[piv]]=[M[piv],M[i]];
    const d=M[i][i];
    if(Math.abs(d)<1e-10) return null;
    for(let j=0;j<2*n;j++) M[i][j]/=d;
    for(let k=0;k<n;k++){
      if(k===i)continue;
      const f=M[k][i];
      for(let j=0;j<2*n;j++) M[k][j]-=f*M[i][j];
    }
  }
  return M.map(row=>row.slice(n));
}

// 6-row Jacobian (position + orientation). Column i's angular part is the
// joint's own rotation axis (zax(f)) — valid because a Z-rotation doesn't
// change its own axis direction, so it's the same vector already used for
// the linear part below, just reused as the angular row too.
function jacobian6(q){
  const fr=fk(q);
  const pEE=pos(fr[N]);
  return fr.slice(0,N).map(f=>{
    const z=zax(f);
    const lin=cross(z,sub3(pEE,pos(f)));
    return [lin[0],lin[1],lin[2],z[0],z[1],z[2]];
  });
}

function dlsStep6(J,dx,lam){
  const D=6;
  const JJT=Array.from({length:D},()=>new Array(D).fill(0));
  J.forEach(col=>{for(let r=0;r<D;r++)for(let c=0;c<D;c++)JJT[r][c]+=col[r]*col[c];});
  const l2=lam*lam;
  for(let i=0;i<D;i++) JJT[i][i]+=l2;
  const inv=invN(JJT); if(!inv) return new Array(N).fill(0);
  const v=new Array(D).fill(0);
  for(let r=0;r<D;r++)for(let c=0;c<D;c++) v[r]+=inv[r][c]*dx[c];
  return J.map(col=>col.reduce((s,x,r)=>s+x*v[r],0));
}

// solveIK(target, q0, targetRPY?) — targetRPY is optional [roll,pitch,yaw]
// in RADIANS. When omitted, this is byte-for-byte the original
// position-only solver (used by "Solve IK"). When provided, it runs a
// separate 6D position+orientation solve (used by the orientation
// "Apply" button) without altering the position-only path at all.
function solveIK(target,q0,targetRPY){
  let q=clamp(q0?q0.slice():HOME.slice());
  let best=q.slice(),bestErr=Infinity;

  if(!targetRPY){
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

  // Position error is in millimetres (~10s-100s), orientation error is a
  // rotation-vector in radians (~0-3.14) — combining them raw in one
  // damped-least-squares step lets whichever has the bigger numbers
  // dominate unpredictably. POS_SCALE brings position into the same
  // numeric range (metres) as the orientation error before they're
  // combined; the resulting joint deltas are unaffected by the scale
  // choice itself, only the relative weighting between the two error
  // types during the solve.
  const POS_SCALE=0.001;
  const Rtarget=buildR(targetRPY[0],targetRPY[1],targetRPY[2]);
  let bestOriErr=Infinity;
  for(let it=0;it<300;it++){
    const T=fk(q)[N];
    const tcp=pos(T);
    const dxp=[target[0]-tcp[0],target[1]-tcp[1],target[2]-tcp[2]];
    const dxo=rotVecError(Rtarget,rot3(T));
    const errP=norm3(dxp);
    const errO=norm3(dxo);
    if(errP<bestErr){bestErr=errP;bestOriErr=errO;best=q.slice();}
    if(errP<2 && errO<0.02) break; // ~1° orientation tolerance
    const J6=jacobian6(q).map(col=>[col[0]*POS_SCALE,col[1]*POS_SCALE,col[2]*POS_SCALE,col[3],col[4],col[5]]);
    const dxScaled=[dxp[0]*POS_SCALE,dxp[1]*POS_SCALE,dxp[2]*POS_SCALE,dxo[0],dxo[1],dxo[2]];
    const dq=dlsStep6(J6,dxScaled,0.3);
    q=clamp(q.map((v,i)=>v+0.6*dq[i]));
  }
  const tcp=getTCP(best);
  const traj=quintic(q0||HOME,best,50);
  return{angles:best,error:bestErr,orientationError:bestOriErr,tcp,trajectory:traj,success:bestErr<20 && bestOriErr<0.09};
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
    document.getElementById('setOrientation')?.addEventListener('click',()=>this.applyOrientation());
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

  // Wired to the IK panel's orientation "Apply" button. Reads X/Y/Z plus
  // the O1/O2/O3 (roll/pitch/yaw, degrees) fields and runs a combined
  // position+orientation solve — same safety-check and trajectory
  // playback as solve(), just asking the active robot's module for a 6D
  // solve instead of position-only.
  applyOrientation(){
    const mod=this._activeModule();
    const robot=(window.PandaSliderControl && window.PandaSliderControl.getActiveRobot) ? window.PandaSliderControl.getActiveRobot() : 'fr3';
    const x=parseFloat(document.getElementById('X')?.value||0);
    const y=parseFloat(document.getElementById('Y')?.value||0);
    const z=parseFloat(document.getElementById('Z')?.value||400);
    const r=parseFloat(document.getElementById('O1')?.value||0);
    const p=parseFloat(document.getElementById('O2')?.value||0);
    const yaw=parseFloat(document.getElementById('O3')?.value||0);
    const ws=mod.checkWorkspace(x,y,z);
    if(!ws.valid){this._status('⚠ '+ws.errors.join(' | '),'warn');return{success:false,blocked:'workspace'};}
    this._status('Solving orientation…','');
    const rpyRad=[r*mod.DEG,p*mod.DEG,yaw*mod.DEG];
    const res=mod.solveIK([x,y,z],this._q,rpyRad);

    if (typeof window.PandaSafety !== 'undefined') {
      const safety = window.PandaSafety.checkSafety(res.angles, robot);
      if (!safety.safe) {
        const reason = safety.groundViolation ? 'would go below ground' : 'self-collision risk';
        const detail = safety.details[0] ? ' — '+safety.details[0] : '';
        this._status(`⛔ Blocked: ${reason}${detail}`, 'err');
        if (typeof window.termLog === 'function') window.termLog(`Orientation apply blocked: ${reason}${detail}`, 'err');
        return { success:false, blocked:'safety', safety };
      }
    }

    const oriDeg = res.orientationError!=null ? (res.orientationError*mod.RAD).toFixed(1) : '?';
    this._status(res.success?`✓ pos ${res.error.toFixed(1)}mm / ori ${oriDeg}°`:`⚠ best pos ${res.error.toFixed(1)}mm / ori ${oriDeg}°`,res.success?'ok':'warn');
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
