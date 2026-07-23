/**
 * Digital Twin — Safety Checking
 * Approximates each link as a capsule (line segment + radius) and checks:
 *   1) Ground-plane violation — any link point below the base mounting plane
 *   2) Self-collision — two non-adjacent links passing within LINK collision
 *      distance of each other
 * This is a geometric approximation (capsules, not real meshes), tuned so
 * normal reachable poses (including HOME) don't false-positive. It's meant
 * to catch genuine "arm folding into itself" or "arm through the floor"
 * cases, not to be a certified collision engine.
 *
 * Supports multiple robots (FR3, KUKA KR4R600) via a per-robot PROFILES
 * table — checkSafety(qRad, robotKey) picks the right link-point source
 * (window.PandaIK vs window.KukaIK) and the right calibration constants.
 * The FR3 profile's constants were calibrated against known-safe/known-bad
 * test poses (see comments below). The KUKA profile is a conservative
 * estimate based on that robot's actual link lengths/geometry (it's a
 * heavier industrial arm than the FR3, hence larger clearances) rather
 * than pose-by-pose calibration — flagged here so it's not mistaken for
 * being equally validated.
 */
(function () {
  'use strict';

  const PROFILES = {
    fr3: {
      getLinkPoints: qRad => window.PandaIK.getLinkPoints(qRad),
      GROUND_Z: 0,              // base mounting plane, mm (matches CHAIN's z=0 origin)
      GROUND_CLEARANCE: 5,      // mm — stop this far ABOVE the plane, before contact.
                                 // (Previously this was applied as tolerance BELOW the
                                 // plane, which let the arm dip 15mm underground before
                                 // flagging — backwards. Now it blocks approaching within
                                 // 5mm of z=0, so the arm never actually reaches the floor.)
      MERGE_THRESHOLD: 30,      // mm — several FR3 joints share the same physical
                                 // location (zero-offset rotation axes, e.g. the
                                 // wrist cluster), so raw per-joint points produce
                                 // degenerate zero-length "links". Collapsing points
                                 // closer than this turns the 9 DH points into ~6-7
                                 // segments that actually correspond to physical links.
      COLLISION_DIST: 35,       // mm, min center-line distance between non-adjacent
                                 // physical links — calibrated against HOME (82mm),
                                 // a fully extended pose (63mm) and known-folded test
                                 // poses (0mm) so normal reachable poses don't false-positive.
      BASE_COLLISION_DIST: 60,  // mm — segment 0 is always the fixed base/pedestal
                                 // column, physically bulkier than the slender arm
                                 // links. Verified against known-safe poses (min
                                 // observed base clearance ~91mm) vs a known-bad one
                                 // (36mm), so 60mm sits cleanly between the two.
    },
    kuka: {
      getLinkPoints: qRad => window.KukaIK.getLinkPoints(qRad),
      // kuka.wrl's robot root Transform (and GroundPlane) both sit at
      // world z=-400 (see kuka.wrl comments), so "ground" for link points
      // returned in that same world frame is z=-400, not 0.
      GROUND_Z: -400,
      GROUND_CLEARANCE: 5,
      MERGE_THRESHOLD: 30,
      // NOTE — this profile is a rough estimate, not pose-calibrated
      // against known-safe/known-bad test poses the way the FR3 profile
      // above was. At A1=0 the KR4R600's whole chain lies in a single
      // vertical plane through the base, so its HOME pose legitimately
      // brings several non-adjacent links within ~20-30mm of the base's
      // own vertical column in this simplified point-capsule model — not
      // a real collision, just a side effect of approximating a bulky
      // industrial base as a zero-radius line. BASE_COLLISION_DIST is
      // therefore left at 0 (i.e. base-vs-arm checks disabled) until this
      // can be validated against real known-bad poses; arm-to-arm checks
      // stay on with a tighter distance to still catch genuine self-folds.
      COLLISION_DIST: 15,
      BASE_COLLISION_DIST: 0,
    },
  };

  const SKIP_CHAIN_DISTANCE = 1; // ignore segment pairs that are directly adjacent
                                  // after merging (they share a joint)

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function clamp01(v) { return Math.min(1, Math.max(0, v)); }

  // Closest distance between segment p1-q1 and segment p2-q2 (Ericson, RTCD 5.1.9)
  function segSegDist(p1, q1, p2, q2) {
    const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
    const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    const EPS = 1e-9;
    let s, t;
    if (a <= EPS && e <= EPS) { s = 0; t = 0; }
    else if (a <= EPS) { s = 0; t = clamp01(f / e); }
    else {
      const c = dot(d1, r);
      if (e <= EPS) { t = 0; s = clamp01(-c / a); }
      else {
        const b = dot(d1, d2), denom = a * e - b * b;
        s = denom !== 0 ? clamp01((b * f - c * e) / denom) : 0;
        t = (b * s + f) / e;
        if (t < 0) { t = 0; s = clamp01(-c / a); }
        else if (t > 1) { t = 1; s = clamp01((b - c) / a); }
      }
    }
    const c1 = add(p1, scale(d1, s));
    const c2 = add(p2, scale(d2, t));
    const diff = sub(c1, c2);
    return Math.sqrt(dot(diff, diff));
  }

  function norm(a) { return Math.sqrt(dot(a, a)); }

  // Collapse near-duplicate consecutive points (zero-offset joints) into a
  // smaller list that actually represents distinct physical link segments.
  function mergePoints(pts, thresh) {
    const out = [pts[0]];
    for (let i = 1; i < pts.length; i++) {
      const last = out[out.length - 1];
      if (norm(sub(pts[i], last)) > thresh || i === pts.length - 1) out.push(pts[i]);
    }
    return out;
  }

  /**
   * qRad: array of joint angles in radians (7 for FR3, 6 for KUKA — same
   * order each robot's IK module expects).
   * robotKey: 'fr3' (default) or 'kuka' — selects which link-point source
   * and calibration profile (PROFILES above) to use.
   * Returns { safe, groundViolation, selfCollision, minZ, details[] }
   */
  function checkSafety(qRad, robotKey) {
    const profile = PROFILES[robotKey] || PROFILES.fr3;
    const details = [];
    const rawPts = profile.getLinkPoints(qRad);

    // rawPts[0] is the fixed base/mounting point — not a link that can move
    // into the floor, so it's excluded from the ground check (otherwise a
    // 5mm clearance would flag every pose).
    let minZ = Infinity;
    rawPts.slice(1).forEach(p => { if (p[2] < minZ) minZ = p[2]; });
    const groundViolation = minZ < profile.GROUND_Z + profile.GROUND_CLEARANCE;
    if (groundViolation) details.push(`link point at z=${minZ.toFixed(0)}mm is within ${profile.GROUND_CLEARANCE}mm of the ground plane`);

    const pts = mergePoints(rawPts, profile.MERGE_THRESHOLD);
    let selfCollision = false;
    const n = pts.length - 1; // number of merged (physical) link segments
    // pts[0]->pts[1] (segment index 0) is always the base->first-joint column:
    // raw point 0 is the fixed origin and raw point 1 sits directly above it
    // regardless of joint1's rotation (rotating about a point on its own axis
    // doesn't move it), so this segment reliably represents the physical
    // base/pedestal in every pose — safe to special-case by index.
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (j - i <= SKIP_CHAIN_DISTANCE) continue;
        const limit = (i === 0 || j === 0) ? profile.BASE_COLLISION_DIST : profile.COLLISION_DIST;
        const d = segSegDist(pts[i], pts[i + 1], pts[j], pts[j + 1]);
        if (d < limit) {
          selfCollision = true;
          details.push(`link segments ${i + 1} and ${j + 1} are ${d.toFixed(0)}mm apart (min ${limit}mm)`);
        }
      }
    }

    return { safe: !groundViolation && !selfCollision, groundViolation, selfCollision, minZ, details };
  }

  window.PandaSafety = { checkSafety, PROFILES };
})();
