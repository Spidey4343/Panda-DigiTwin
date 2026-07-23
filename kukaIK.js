// ============================================================================
// KUKA KR4 R600 Inverse Kinematics Solver
// ============================================================================
// This module implements inverse kinematics for the KUKA KR4 R600 robot
// using numerical methods with 20 discretization points per axis.
//
// Features:
// - Numerical IK solver with Jacobian pseudo-inverse
// - Smooth trajectory planning with configurable points
// - Singularity avoidance
// - Joint limit checking
// - TCP position/orientation control
//
// Author: Sanath (AR_2026 DigiTwin cohort) — reused with credit as the
// robot-specific IK backend for the FR3 app's dual-robot (FR3/KR4R600)
// switcher, per Prof. Avgustinov's suggestion that the app be adaptable
// across robot geometries.
// Created: January 2026
//
// Additions below the original solver class (marked "FR3 app adapter
// layer") wrap it to match this project's PandaIK-style call conventions
// (getTCP(qRad), solveIK(target, q0) -> {angles, error, tcp, trajectory,
// success}, getLinkPoints(qRad) for pandaSafety.js) so the KRL backend and
// safety module can treat FR3 and KUKA interchangeably.
// ============================================================================

(function() {
    'use strict';

    // Robot DH Parameters for KUKA KR4 R600
    // Based on actual WRL file transforms
    const ROBOT_LINKS = {
        // Each link: [tx, ty, tz] translation relative to parent
        BASE: [0, 0, -400],      // Base offset from WRL root
        A1: [0, 0, 187.2],       // A1 translation
        A2: [0, 0, 140.9],       // A2 translation
        A3: [0, 0, 289.6],       // A3 translation
        A4: [173, 0, 20],        // A4 translation
        A5: [135.5, 0, 0],       // A5 translation
        A6: [57.06, 0, 0]        // A6 translation (TCP)
    };

    // Joint limits (in radians)
    const JOINT_LIMITS = {
        A1: { min: -2.880, max: 2.880 },      // ±165°
        A2: { min: -1.745, max: 2.182 },      // -100° to +125° (in WRL coords)
        A3: { min: -3.491, max: 0.960 },      // -200° to +55° (in WRL coords)
        A4: { min: -3.142, max: 3.142 },      // ±180°
        A5: { min: -2.007, max: 2.007 },      // ±115°
        A6: { min: -6.021, max: 6.021 }       // ±345°
    };

    // IK Solver parameters
    const IK_CONFIG = {
        maxIterations: 50,       // Reduced since we use analytical initial guess
        tolerance: 2.0,          // Position tolerance in mm (relaxed for reliability)
        dampingFactor: 0.5,      // Damped least squares factor
        stepSize: 0.4,           // Step size for gradient descent
        pointsPerAxis: 50        // Increased for smoother motion (was 20)
    };

    // Workspace limits for KUKA KR4 R600
    const WORKSPACE_LIMITS = {
        x: { min: -750, max: 750 },      // mm
        y: { min: -750, max: 750 },      // mm
        z: { min: -350, max: 800 },      // mm (accounting for base offset)
        reach: { min: 150, max: 750 }    // Radial distance from base
    };

    class InverseKinematicsSolver {
        constructor() {
            this.currentJointAngles = {
                A1: 0, A2: 0, A3: 0, A4: 0, A5: 0, A6: 0
            };
            this.tcpPosition = { x: 0, y: 0, z: 0 };
            this.tcpOrientation = { roll: 0, pitch: 0, yaw: 0 };
        }

        // ====================================================================
        // Forward Kinematics
        // ====================================================================

        dhTransform(a, alpha, d, theta) {
            const ct = Math.cos(theta);
            const st = Math.sin(theta);
            const ca = Math.cos(alpha);
            const sa = Math.sin(alpha);

            return [
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d   ],
                [0,   0,      0,     1   ]
            ];
        }

        matrixMultiply(A, B) {
            const result = Array(4).fill(0).map(() => Array(4).fill(0));
            for (let i = 0; i < 4; i++) {
                for (let j = 0; j < 4; j++) {
                    for (let k = 0; k < 4; k++) {
                        result[i][j] += A[i][k] * B[k][j];
                    }
                }
            }
            return result;
        }

        extractPosition(T) {
            return {
                x: T[0][3],
                y: T[1][3],
                z: T[2][3]
            };
        }

        extractEulerAngles(T) {
            const r11 = T[0][0], r12 = T[0][1], r13 = T[0][2];
            const r21 = T[1][0], r22 = T[1][1], r23 = T[1][2];
            const r31 = T[2][0], r32 = T[2][1], r33 = T[2][2];

            const pitch = Math.atan2(-r31, Math.sqrt(r11*r11 + r21*r21));
            const roll = Math.atan2(r21/Math.cos(pitch), r11/Math.cos(pitch));
            const yaw = Math.atan2(r32/Math.cos(pitch), r33/Math.cos(pitch));

            return { roll, pitch, yaw };
        }

        forwardKinematics(angles) {
            if (Array.isArray(angles)) {
                angles = {
                    A1: angles[0], A2: angles[1], A3: angles[2],
                    A4: angles[3], A5: angles[4], A6: angles[5]
                };
            }

            let theta1 = angles.A1;
            let theta2 = angles.A2;
            let theta3 = angles.A3;
            let theta4 = angles.A4;
            let theta5 = angles.A5;
            let theta6 = angles.A6;

            if (Math.abs(theta1) > 10 || Math.abs(theta2) > 10) {
                theta1 = theta1 * Math.PI / 180;
                theta2 = theta2 * Math.PI / 180;
                theta3 = theta3 * Math.PI / 180;
                theta4 = theta4 * Math.PI / 180;
                theta5 = theta5 * Math.PI / 180;
                theta6 = theta6 * Math.PI / 180;
            }

            const T1 = this.rotZ(theta1);
            T1[0][3] = ROBOT_LINKS.A1[0];
            T1[1][3] = ROBOT_LINKS.A1[1];
            T1[2][3] = ROBOT_LINKS.A1[2];

            const T2 = this.rotY(theta2);
            T2[0][3] = ROBOT_LINKS.A2[0];
            T2[1][3] = ROBOT_LINKS.A2[1];
            T2[2][3] = ROBOT_LINKS.A2[2];

            const T3 = this.rotY(theta3);
            T3[0][3] = ROBOT_LINKS.A3[0];
            T3[1][3] = ROBOT_LINKS.A3[1];
            T3[2][3] = ROBOT_LINKS.A3[2];

            const T4 = this.rotX(theta4);
            T4[0][3] = ROBOT_LINKS.A4[0];
            T4[1][3] = ROBOT_LINKS.A4[1];
            T4[2][3] = ROBOT_LINKS.A4[2];

            const T5 = this.rotY(theta5);
            T5[0][3] = ROBOT_LINKS.A5[0];
            T5[1][3] = ROBOT_LINKS.A5[1];
            T5[2][3] = ROBOT_LINKS.A5[2];

            const T6 = this.rotX(theta6);
            T6[0][3] = ROBOT_LINKS.A6[0];
            T6[1][3] = ROBOT_LINKS.A6[1];
            T6[2][3] = ROBOT_LINKS.A6[2];

            let T = T1;
            T = this.matrixMultiply(T, T2);
            T = this.matrixMultiply(T, T3);
            T = this.matrixMultiply(T, T4);
            T = this.matrixMultiply(T, T5);
            T = this.matrixMultiply(T, T6);

            T[0][3] += ROBOT_LINKS.BASE[0];
            T[1][3] += ROBOT_LINKS.BASE[1];
            T[2][3] += ROBOT_LINKS.BASE[2];

            return {
                position: this.extractPosition(T),
                orientation: this.extractEulerAngles(T),
                transform: T
            };
        }

        rotX(angle) {
            const c = Math.cos(angle);
            const s = Math.sin(angle);
            return [
                [1, 0,  0, 0],
                [0, c, -s, 0],
                [0, s,  c, 0],
                [0, 0,  0, 1]
            ];
        }

        rotY(angle) {
            const c = Math.cos(angle);
            const s = Math.sin(angle);
            return [
                [ c, 0, s, 0],
                [ 0, 1, 0, 0],
                [-s, 0, c, 0],
                [ 0, 0, 0, 1]
            ];
        }

        rotZ(angle) {
            const c = Math.cos(angle);
            const s = Math.sin(angle);
            return [
                [c, -s, 0, 0],
                [s,  c, 0, 0],
                [0,  0, 1, 0],
                [0,  0, 0, 1]
            ];
        }

        // ====================================================================
        // Jacobian Calculation
        // ====================================================================

        calculateJacobian(angles) {
            const epsilon = 0.0001;
            const J = Array(6).fill(0).map(() => Array(6).fill(0));

            const currentFK = this.forwardKinematics(angles);
            const currentPos = currentFK.position;
            const currentOri = currentFK.orientation;

            for (let i = 0; i < 6; i++) {
                const perturbedAngles = {...angles};
                const jointNames = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6'];
                perturbedAngles[jointNames[i]] += epsilon;

                const perturbedFK = this.forwardKinematics(perturbedAngles);
                const perturbedPos = perturbedFK.position;
                const perturbedOri = perturbedFK.orientation;

                J[0][i] = (perturbedPos.x - currentPos.x) / epsilon;
                J[1][i] = (perturbedPos.y - currentPos.y) / epsilon;
                J[2][i] = (perturbedPos.z - currentPos.z) / epsilon;
                J[3][i] = (perturbedOri.roll - currentOri.roll) / epsilon;
                J[4][i] = (perturbedOri.pitch - currentOri.pitch) / epsilon;
                J[5][i] = (perturbedOri.yaw - currentOri.yaw) / epsilon;
            }

            return J;
        }

        pseudoInverse(J) {
            const lambda = IK_CONFIG.dampingFactor;
            const JT = this.transpose(J);
            const JJT = this.matrixMultiplyGeneral(J, JT);

            for (let i = 0; i < 6; i++) {
                JJT[i][i] += lambda * lambda;
            }

            const JJT_inv = this.matrixInverse(JJT);
            return this.matrixMultiplyGeneral(JT, JJT_inv);
        }

        transpose(A) {
            const rows = A.length;
            const cols = A[0].length;
            const result = Array(cols).fill(0).map(() => Array(rows).fill(0));
            for (let i = 0; i < rows; i++) {
                for (let j = 0; j < cols; j++) {
                    result[j][i] = A[i][j];
                }
            }
            return result;
        }

        matrixMultiplyGeneral(A, B) {
            const rowsA = A.length;
            const colsA = A[0].length;
            const rowsB = B.length;
            const colsB = B[0].length;

            if (colsA !== rowsB) {
                throw new Error('Matrix dimensions incompatible for multiplication');
            }

            const result = Array(rowsA).fill(0).map(() => Array(colsB).fill(0));
            for (let i = 0; i < rowsA; i++) {
                for (let j = 0; j < colsB; j++) {
                    for (let k = 0; k < colsA; k++) {
                        result[i][j] += A[i][k] * B[k][j];
                    }
                }
            }
            return result;
        }

        matrixInverse(A) {
            const n = A.length;
            const augmented = A.map((row, i) => {
                const identity = Array(n).fill(0);
                identity[i] = 1;
                return [...row, ...identity];
            });

            for (let i = 0; i < n; i++) {
                let maxRow = i;
                for (let k = i + 1; k < n; k++) {
                    if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                        maxRow = k;
                    }
                }
                [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];

                const pivot = augmented[i][i];
                if (Math.abs(pivot) < 1e-10) {
                    console.warn('Matrix is singular or nearly singular');
                    return this.identityMatrix(n);
                }
                for (let j = 0; j < 2 * n; j++) {
                    augmented[i][j] /= pivot;
                }

                for (let k = 0; k < n; k++) {
                    if (k !== i) {
                        const factor = augmented[k][i];
                        for (let j = 0; j < 2 * n; j++) {
                            augmented[k][j] -= factor * augmented[i][j];
                        }
                    }
                }
            }

            return augmented.map(row => row.slice(n));
        }

        identityMatrix(n) {
            return Array(n).fill(0).map((_, i) =>
                Array(n).fill(0).map((_, j) => i === j ? 1 : 0)
            );
        }

        // ====================================================================
        // Inverse Kinematics Solver
        // ====================================================================

        isInWorkspace(targetPosition) {
            if (targetPosition.x < WORKSPACE_LIMITS.x.min || targetPosition.x > WORKSPACE_LIMITS.x.max) {
                return { valid: false, reason: `X=${targetPosition.x} out of range [${WORKSPACE_LIMITS.x.min}, ${WORKSPACE_LIMITS.x.max}]` };
            }
            if (targetPosition.y < WORKSPACE_LIMITS.y.min || targetPosition.y > WORKSPACE_LIMITS.y.max) {
                return { valid: false, reason: `Y=${targetPosition.y} out of range [${WORKSPACE_LIMITS.y.min}, ${WORKSPACE_LIMITS.y.max}]` };
            }
            if (targetPosition.z < WORKSPACE_LIMITS.z.min || targetPosition.z > WORKSPACE_LIMITS.z.max) {
                return { valid: false, reason: `Z=${targetPosition.z} out of range [${WORKSPACE_LIMITS.z.min}, ${WORKSPACE_LIMITS.z.max}]` };
            }

            const radialDist = Math.sqrt(targetPosition.x * targetPosition.x + targetPosition.y * targetPosition.y);
            if (radialDist < WORKSPACE_LIMITS.reach.min || radialDist > WORKSPACE_LIMITS.reach.max) {
                return { valid: false, reason: `Radial distance ${radialDist.toFixed(1)}mm out of range [${WORKSPACE_LIMITS.reach.min}, ${WORKSPACE_LIMITS.reach.max}]` };
            }

            return { valid: true };
        }

        static getWorkspaceLimits() {
            return WORKSPACE_LIMITS;
        }

        checkJointLimits(angles) {
            for (const joint in angles) {
                const limits = JOINT_LIMITS[joint];
                if (angles[joint] < limits.min || angles[joint] > limits.max) {
                    return false;
                }
            }
            return true;
        }

        clampJointAngles(angles) {
            const clamped = {...angles};
            for (const joint in clamped) {
                const limits = JOINT_LIMITS[joint];
                clamped[joint] = Math.max(limits.min, Math.min(limits.max, clamped[joint]));
            }
            return clamped;
        }

        solveAnalyticalIK3DOF(targetPosition) {
            const px = targetPosition.x;
            const py = targetPosition.y;
            const pz = targetPosition.z - ROBOT_LINKS.BASE[2];

            const d1 = ROBOT_LINKS.A1[2];
            const a2 = ROBOT_LINKS.A2[2];
            const a3 = ROBOT_LINKS.A3[2];
            const d4 = ROBOT_LINKS.A4[0];

            const theta1 = Math.atan2(py, px);
            const r = Math.sqrt(px * px + py * py);
            const wx = r - d4;
            const wz = pz - d1;
            const D = Math.sqrt(wx * wx + wz * wz);

            const maxReach = a2 + a3;
            const minReach = Math.abs(a2 - a3);

            if (D > maxReach || D < minReach) {
                console.warn(`Target unreachable: D=${D.toFixed(1)}, range=[${minReach.toFixed(1)}, ${maxReach.toFixed(1)}]`);
                return null;
            }

            const cos_theta3 = (D * D - a2 * a2 - a3 * a3) / (2 * a2 * a3);
            const cos_theta3_clamped = Math.max(-1, Math.min(1, cos_theta3));
            const theta3 = Math.acos(cos_theta3_clamped);

            const alpha = Math.atan2(wz, wx);
            const beta = Math.atan2(a3 * Math.sin(theta3), a2 + a3 * Math.cos(theta3));
            const theta2 = alpha - beta;

            return {
                A1: theta1,
                A2: theta2,
                A3: theta3,
                A4: 0,
                A5: 0,
                A6: 0
            };
        }

        solveIK(targetPosition, targetOrientation = null, initialGuess = null) {
            const workspaceCheck = this.isInWorkspace(targetPosition);
            if (!workspaceCheck.valid) {
                console.error('Target outside workspace:', workspaceCheck.reason);
                return null;
            }

            let currentAngles = this.solveAnalyticalIK3DOF(targetPosition);

            if (!currentAngles) {
                currentAngles = initialGuess || {...this.currentJointAngles};
                currentAngles.A1 = Math.atan2(targetPosition.y, targetPosition.x);
                if (Math.abs(currentAngles.A2) < 0.01) currentAngles.A2 = -0.5;
                if (Math.abs(currentAngles.A3) < 0.01) currentAngles.A3 = 0.8;
            }

            let converged = false;

            for (let iter = 0; iter < IK_CONFIG.maxIterations; iter++) {
                const currentFK = this.forwardKinematics(currentAngles);
                const currentPos = currentFK.position;

                const errorX = targetPosition.x - currentPos.x;
                const errorY = targetPosition.y - currentPos.y;
                const errorZ = targetPosition.z - currentPos.z;

                const positionError = Math.sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

                if (positionError < IK_CONFIG.tolerance) {
                    converged = true;
                    break;
                }

                if (iter > 10 && positionError > 2000) {
                    break;
                }

                let errorVector;
                if (targetOrientation) {
                    const currentOri = currentFK.orientation;
                    errorVector = [
                        errorX, errorY, errorZ,
                        targetOrientation.roll - currentOri.roll,
                        targetOrientation.pitch - currentOri.pitch,
                        targetOrientation.yaw - currentOri.yaw
                    ];
                } else {
                    errorVector = [errorX, errorY, errorZ, 0, 0, 0];
                }

                const J = this.calculateJacobian(currentAngles);
                const J_pinv = this.pseudoInverse(J);

                let deltaTheta = [0, 0, 0, 0, 0, 0];
                for (let i = 0; i < 6; i++) {
                    for (let j = 0; j < 6; j++) {
                        deltaTheta[i] += J_pinv[i][j] * errorVector[j];
                    }
                    deltaTheta[i] *= IK_CONFIG.stepSize;
                }

                const jointNames = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6'];
                for (let i = 0; i < 6; i++) {
                    currentAngles[jointNames[i]] += deltaTheta[i];
                }

                currentAngles = this.clampJointAngles(currentAngles);
            }

            const finalFK = this.forwardKinematics(currentAngles);
            const finalPos = finalFK.position;
            const finalError = Math.sqrt(
                Math.pow(targetPosition.x - finalPos.x, 2) +
                Math.pow(targetPosition.y - finalPos.y, 2) +
                Math.pow(targetPosition.z - finalPos.z, 2)
            );

            const trajectory = this.generateTrajectory(this.currentJointAngles, currentAngles, IK_CONFIG.pointsPerAxis);

            return { finalAngles: currentAngles, finalError, trajectory, tcp: finalPos };
        }

        generateTrajectory(startAngles, endAngles, numPoints) {
            const trajectory = [];
            const jointNames = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6'];

            for (let i = 0; i <= numPoints; i++) {
                const t = i / numPoints;
                const smoothT = t * t * t * (t * (t * 6 - 15) + 10);

                const point = {};
                for (const joint of jointNames) {
                    const delta = endAngles[joint] - startAngles[joint];
                    point[joint] = startAngles[joint] + delta * smoothT;
                }

                trajectory.push(point);
            }

            return trajectory;
        }

        setJointAngles(angles) {
            this.currentJointAngles = {...angles};
            const fk = this.forwardKinematics(angles);
            this.tcpPosition = fk.position;
            this.tcpOrientation = fk.orientation;
        }

        getTCPPosition() {
            return {...this.tcpPosition};
        }

        getTCPOrientation() {
            return {...this.tcpOrientation};
        }

        static degToRad(degrees) {
            return degrees * Math.PI / 180;
        }

        static radToDeg(radians) {
            return radians * 180 / Math.PI;
        }
    }

    // Export to global scope
    window.InverseKinematicsSolver = InverseKinematicsSolver;
    window.IKSolver = new InverseKinematicsSolver(); // Global instance

    // ========================================================================
    // FR3 app adapter layer
    // ------------------------------------------------------------------------
    // Everything above this line is Sanath's original solver, unmodified.
    // Everything below adapts it to the same call shape as pandaIK.js
    // (window.PandaIK) so the KRL backend (pandaKRL.js) and the safety
    // module (pandaSafety.js) can drive either robot through one interface.
    // ========================================================================

    const JOINT_NAMES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6'];
    const DEG = Math.PI / 180;
    const RAD = 180 / Math.PI;

    function arrToAngles(qRad) {
        const a = {};
        JOINT_NAMES.forEach((name, i) => { a[name] = qRad[i] || 0; });
        return a;
    }
    function anglesToArr(angles) {
        return JOINT_NAMES.map(name => angles[name] || 0);
    }

    // getTCP(qRad) — matches PandaIK.getTCP(qRad): radians array in, [x,y,z] out
    function getTCP(qRad) {
        const fk = window.IKSolver.forwardKinematics(arrToAngles(qRad));
        return [fk.position.x, fk.position.y, fk.position.z];
    }

    // getLinkPoints(qRad) — matches PandaIK.getLinkPoints(qRad): returns the
    // base point plus every intermediate joint-frame origin (used by
    // pandaSafety.js for ground-plane and self-collision capsule checks).
    function getLinkPoints(qRad) {
        const solver = window.IKSolver;
        const a = arrToAngles(qRad);

        const mk = (rotFn, theta, link) => {
            const T = rotFn.call(solver, theta);
            T[0][3] = link[0]; T[1][3] = link[1]; T[2][3] = link[2];
            return T;
        };
        const T1 = mk(solver.rotZ, a.A1, ROBOT_LINKS.A1);
        const T2 = mk(solver.rotY, a.A2, ROBOT_LINKS.A2);
        const T3 = mk(solver.rotY, a.A3, ROBOT_LINKS.A3);
        const T4 = mk(solver.rotX, a.A4, ROBOT_LINKS.A4);
        const T5 = mk(solver.rotY, a.A5, ROBOT_LINKS.A5);
        const T6 = mk(solver.rotX, a.A6, ROBOT_LINKS.A6);

        const withBase = p => [p.x + ROBOT_LINKS.BASE[0], p.y + ROBOT_LINKS.BASE[1], p.z + ROBOT_LINKS.BASE[2]];

        const pts = [[0, 0, 0]]; // fixed base/mounting point
        let T = T1; pts.push(withBase(solver.extractPosition(T)));
        T = solver.matrixMultiply(T, T2); pts.push(withBase(solver.extractPosition(T)));
        T = solver.matrixMultiply(T, T3); pts.push(withBase(solver.extractPosition(T)));
        T = solver.matrixMultiply(T, T4); pts.push(withBase(solver.extractPosition(T)));
        T = solver.matrixMultiply(T, T5); pts.push(withBase(solver.extractPosition(T)));
        T = solver.matrixMultiply(T, T6); pts.push(withBase(solver.extractPosition(T)));
        return pts;
    }

    // solveIK(targetXYZ, q0Rad) — matches PandaIK.solveIK's return shape:
    // { angles, error, tcp, trajectory, success }, angles/trajectory in
    // radians ordered [A1..A6], so IKController.solve() in index.html can
    // call whichever robot's solver identically.
    function solveIK(targetXYZ, q0Rad) {
        const target = { x: targetXYZ[0], y: targetXYZ[1], z: targetXYZ[2] };
        const q0 = q0Rad ? arrToAngles(q0Rad) : null;
        if (q0) window.IKSolver.currentJointAngles = q0;

        const ws = window.IKSolver.isInWorkspace(target);
        if (!ws.valid) {
            return { angles: q0Rad || anglesToArr(window.IKSolver.currentJointAngles), error: Infinity, tcp: target, trajectory: [], success: false, workspaceError: ws.reason };
        }

        const res = window.IKSolver.solveIK(target);
        if (!res) {
            return { angles: q0Rad || anglesToArr(window.IKSolver.currentJointAngles), error: Infinity, tcp: target, trajectory: [], success: false };
        }

        window.IKSolver.setJointAngles(res.finalAngles);

        return {
            angles: anglesToArr(res.finalAngles),
            error: res.finalError,
            tcp: [res.tcp.x, res.tcp.y, res.tcp.z],
            trajectory: res.trajectory.map(anglesToArr),
            success: res.finalError < 20,
        };
    }

    function checkWorkspace(x, y, z) {
        const res = window.IKSolver.isInWorkspace({ x, y, z });
        return { valid: res.valid, errors: res.valid ? [] : [res.reason] };
    }

    window.KukaIK = {
        JOINT_NAMES,
        DEG, RAD,
        getTCP,
        getLinkPoints,
        solveIK,
        checkWorkspace,
        HOME: [0, -90 * DEG, 90 * DEG, 0, 0, 0], // A1..A6, radians — matches WRL HOME comment
        LIMITS: JOINT_NAMES.map(n => [JOINT_LIMITS[n].min, JOINT_LIMITS[n].max]),
    };

})();
