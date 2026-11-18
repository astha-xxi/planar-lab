export type Point = { x: number; y: number };
export type FourBarParameters = { ground:number; crank:number; coupler:number; rocker:number; inputAngle:number; branch:1|-1 };
export type FourBarSolution = { A:Point; B:Point; O2:Point; O4:Point; valid:boolean; reason?:string; transmissionAngle:number|null; outputAngle:number|null; couplerAngle:number|null; closureError:number|null; inputAngle:number; };
export type KinematicState = { omega3:number|null; omega4:number|null; alpha3:number|null; alpha4:number|null; };
const rad=(d:number)=>d*Math.PI/180, deg=(r:number)=>r*180/Math.PI;
const dist=(p:Point,q:Point)=>Math.hypot(q.x-p.x,q.y-p.y);
export function solveFourBar(p:FourBarParameters):FourBarSolution {
 const {ground:d,crank:a,coupler:b,rocker:c,inputAngle,branch}=p; const O2={x:0,y:0},O4={x:d,y:0};
 const A={x:a*Math.cos(rad(inputAngle)),y:a*Math.sin(rad(inputAngle))}; const dx=d-A.x,dy=-A.y,L=Math.hypot(dx,dy);
 const invalid=(reason:string):FourBarSolution=>({A,B:{x:NaN,y:NaN},O2,O4,valid:false,reason,transmissionAngle:null,outputAngle:null,couplerAngle:null,closureError:null,inputAngle});
 if (![d,a,b,c,inputAngle].every(Number.isFinite)||Math.min(d,a,b,c)<=0) return invalid('All dimensions must be positive finite values.');
 if(L<1e-10) return invalid('The crank pin coincides with the fixed rocker pivot.');
 if(L>b+c+1e-9||L<Math.abs(b-c)-1e-9) return invalid('No closed configuration exists at this input angle.');
 const along=(b*b-c*c+L*L)/(2*L),h2=b*b-along*along; if(h2 < -1e-7) return invalid('The linkage is outside its closure boundary.');
 const h=Math.sqrt(Math.max(0,h2)),ux=dx/L,uy=dy/L,px=A.x+along*ux,py=A.y+along*uy;
 const B={x:px+branch*h*-uy,y:py+branch*h*ux};
 const t4=deg(Math.atan2(B.y,B.x-d)),t3=deg(Math.atan2(B.y-A.y,B.x-A.x));
 const raw=Math.abs(((t3-t4+540)%360)-180),mu=Math.min(raw,180-raw);
 return {A,B,O2,O4,valid:true,transmissionAngle:mu,outputAngle:t4,couplerAngle:t3,closureError:Math.abs(dist(A,B)-b)+Math.abs(dist(O4,B)-c),inputAngle};
}
export function solveKinematics(p:FourBarParameters,omega2:number,alpha2=0):KinematicState {
 const s=solveFourBar(p); if(!s.valid||s.couplerAngle===null||s.outputAngle===null) return {omega3:null,omega4:null,alpha3:null,alpha4:null};
 const t2=rad(p.inputAngle),t3=rad(s.couplerAngle),t4=rad(s.outputAngle),a=p.crank,b=p.coupler,c=p.rocker;
 const A=-b*Math.sin(t3),B=c*Math.sin(t4),C=-b*Math.cos(t3),D=c*Math.cos(t4);
 const det=A*D-B*C; if(Math.abs(det)<1e-10) return {omega3:null,omega4:null,alpha3:null,alpha4:null};
 const r1=a*omega2*Math.sin(t2),r2=a*omega2*Math.cos(t2);
 const omega3=(r1*D-B*r2)/det,omega4=(A*r2-r1*C)/det;
 const q1=a*alpha2*Math.sin(t2)+a*omega2*omega2*Math.cos(t2)+b*omega3*omega3*Math.cos(t3)-c*omega4*omega4*Math.cos(t4);
 const q2=a*alpha2*Math.cos(t2)-a*omega2*omega2*Math.sin(t2)-b*omega3*omega3*Math.sin(t3)+c*omega4*omega4*Math.sin(t4);
 const alpha3=(q1*D-B*q2)/det,alpha4=(A*q2-q1*C)/det;
 return {omega3,omega4,alpha3,alpha4};
}
export function grashofClass(lengths:number[]):string { if(lengths.length!==4||lengths.some(v=>!Number.isFinite(v)||v<=0))return 'Invalid geometry'; const [s,p,q,l]=[...lengths].sort((a,b)=>a-b); const delta=s+l-p-q; return Math.abs(delta)<1e-8?'Change-point (Grashof equality)':delta<0?'Grashof (at least one link can fully rotate)':'Non-Grashof (no link can fully rotate)'; }
export function sampleMotion(p:FourBarParameters,step=2,omega2=1){ const rows=[]; for(let angle=0;angle<=360+1e-8;angle+=step){const s=solveFourBar({...p,inputAngle:angle});const k=solveKinematics({...p,inputAngle:angle},omega2);rows.push({inputAngle:angle,valid:s.valid,x:s.valid?s.B.x:null,y:s.valid?s.B.y:null,outputAngle:s.outputAngle,transmissionAngle:s.transmissionAngle,omega4:k.omega4,alpha4:k.alpha4,closureError:s.closureError});} return rows; }
export function optimizeFourBar(base:FourBarParameters,target:Point[],iterations=900){
 // Deterministic bounded coordinate search; objective combines RMS path error and invalid-configuration penalty.
 let best={...base},bestScore=Infinity; const score=(p:FourBarParameters)=>{let e=0,n=0;for(let i=0;i<target.length;i++){const s=solveFourBar({...p,inputAngle:i*360/Math.max(1,target.length-1)});if(!s.valid)return 1e8;if(s.B){e+=(s.B.x-target[i].x)**2+(s.B.y-target[i].y)**2;n++;}}return Math.sqrt(e/Math.max(1,n));};
 bestScore=score(best);let span=35;for(let k=0;k<iterations;k++){const key=(['crank','coupler','rocker'] as const)[k%3];const direction=((Math.floor(k/3)%2)===0?1:-1);const candidate={...best,[key]:Math.max(20,Math.min(240,best[key]+direction*span))};const v=score(candidate);if(v<bestScore){best=candidate;bestScore=v;}if(k%30===29)span*=.82;}
 return {parameters:best,score:bestScore};
}
