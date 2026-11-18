import {describe,it,expect} from 'vitest';
import {grashofClass,solveFourBar,solveKinematics,sampleMotion} from './fourBar';
const base={ground:160,crank:55,coupler:145,rocker:120,inputAngle:42,branch:1 as const};
describe('four-bar position solver',()=>{
 it('solves a reachable configuration with negligible closure error',()=>{const s=solveFourBar(base);expect(s.valid).toBe(true);expect(s.closureError).toBeLessThan(1e-8);});
 it('rejects unreachable configurations',()=>{expect(solveFourBar({...base,inputAngle:200,crank:230,coupler:25,rocker:25}).valid).toBe(false);});
 it('returns mirrored branches',()=>{const a=solveFourBar(base),b=solveFourBar({...base,branch:-1});expect(a.valid&&b.valid).toBe(true);expect(a.B.y).not.toBeCloseTo(b.B.y,4);});
 it('classifies Grashof geometry',()=>{expect(grashofClass([160,55,145,120])).toContain('Grashof');});
});
describe('kinematic analysis',()=>{
 it('returns finite angular velocity and acceleration away from singularity',()=>{const k=solveKinematics(base,1);expect(Number.isFinite(k.omega4)).toBe(true);expect(Number.isFinite(k.alpha4)).toBe(true);});
 it('samples a complete revolution',()=>{const r=sampleMotion(base,2,1);expect(r).toHaveLength(181);expect(r[0].inputAngle).toBe(0);expect(r[180].inputAngle).toBe(360);});
 it('matches output angular velocity with finite-difference position data',()=>{const s0=solveFourBar({...base,inputAngle:42}),s1=solveFourBar({...base,inputAngle:42.001});const finite=(s1.outputAngle!-s0.outputAngle!)*Math.PI/180/(.001*Math.PI/180);const analytic=solveKinematics(base,1).omega4!;expect(Math.abs(finite-analytic)).toBeLessThan(.002);});
});
