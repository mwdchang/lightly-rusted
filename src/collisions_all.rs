use nalgebra::{Vector2, Vector3};

use crate::collisions::{sphere_uv, cube_uv, torus_uv, solve_quartic};


pub struct IntersectInterval {
    pub t_enter: f32,
    pub t_exit:  f32,
    pub normal_enter: Vector3<f32>,
    pub normal_exit:  Vector3<f32>,
    pub uv_enter: Vector2<f32>,
    pub uv_exit:  Vector2<f32>,
}


/**
 * Returns the full [t_enter, t_exit] interval for a unit sphere centred at origin.
 * Unlike intersect_unit_sphere, t_enter may be negative when the ray origin is
 * inside the sphere — this is intentional and required for CSG.
 */
pub fn intersect_all_unit_sphere(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectInterval> {
    let a = dir.dot(&dir);
    let b = 2.0 * origin.dot(&dir);
    let c = origin.dot(&origin) - 1.0;

    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }

    let sqrt_disc = disc.sqrt();
    let t_enter = (-b - sqrt_disc) / (2.0 * a);
    let t_exit  = (-b + sqrt_disc) / (2.0 * a);

    let hit_enter = origin + dir * t_enter;
    let hit_exit  = origin + dir * t_exit;

    let (ue, ve) = sphere_uv(hit_enter);
    let (ux, vx) = sphere_uv(hit_exit);

    Some(IntersectInterval {
        t_enter,
        t_exit,
        normal_enter: hit_enter.normalize(),
        normal_exit:  hit_exit.normalize(),
        uv_enter: Vector2::new(ue, ve),
        uv_exit:  Vector2::new(ux, vx),
    })
}


/**
 * Returns the full [t_enter, t_exit] interval for a unit cube [-1, 1]^3.
 * Uses the slab method; also tracks the outward face normal at both
 * the entry and exit slabs (needed by the CSG normal-selection logic).
 */
pub fn intersect_all_unit_cube(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectInterval> {
    const EPSILON: f32 = 1e-6;
    let bounds_min = Vector3::new(-1.0f32, -1.0, -1.0);
    let bounds_max = Vector3::new( 1.0f32,  1.0,  1.0);

    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;
    let mut normal_enter = Vector3::zeros();
    let mut normal_exit  = Vector3::zeros();

    for axis in 0..3usize {
        let o = origin[axis];
        let d = dir[axis];

        if d.abs() < EPSILON {
            // Ray parallel to this slab — miss if origin is outside
            if o < bounds_min[axis] || o > bounds_max[axis] {
                return None;
            }
            continue;
        }

        let inv_d = 1.0 / d;
        let t_lo = (bounds_min[axis] - o) * inv_d; // intersection with the -axis face
        let t_hi = (bounds_max[axis] - o) * inv_d; // intersection with the +axis face

        // Outward normals for the two slab faces along this axis
        let mut n_lo = Vector3::zeros(); n_lo[axis] = -1.0; // bounds_min face: outward is -axis
        let mut n_hi = Vector3::zeros(); n_hi[axis] =  1.0; // bounds_max face: outward is +axis

        // Order so t0 (entry candidate) <= t1 (exit candidate)
        let (t0, t1, n0, n1) = if t_lo <= t_hi {
            (t_lo, t_hi, n_lo, n_hi)
        } else {
            (t_hi, t_lo, n_hi, n_lo)
        };

        if t0 > tmin { tmin = t0; normal_enter = n0; }
        if t1 < tmax { tmax = t1; normal_exit  = n1; }

        if tmin > tmax {
            return None;
        }
    }

    let hit_enter = origin + dir * tmin;
    let hit_exit  = origin + dir * tmax;

    let (ue, ve) = cube_uv(hit_enter, normal_enter);
    let (ux, vx) = cube_uv(hit_exit,  normal_exit);

    Some(IntersectInterval {
        t_enter: tmin,
        t_exit:  tmax,
        normal_enter,
        normal_exit,
        uv_enter: Vector2::new(ue, ve),
        uv_exit:  Vector2::new(ux, vx),
    })
}


/**
 * Returns the full [t_enter, t_exit] interval for a unit cylinder.
 * x² + z² = 1, y ∈ [-1, 1], capped with disks at y = ±1.
 *
 * All hits (side + caps) are collected without a t > 0 guard,
 * sorted, and the outermost pair is returned as the interval.
 */
pub fn intersect_all_unit_cylinder(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectInterval> {
    // (t, outward_normal, uv)
    let mut hits: Vec<(f32, Vector3<f32>, Vector2<f32>)> = Vec::new();

    // ---- Cylinder side: x² + z² = 1 ----
    let a = dir.x * dir.x + dir.z * dir.z;
    if a.abs() > 1e-6 {
        let b    = 2.0 * (origin.x * dir.x + origin.z * dir.z);
        let c    = origin.x * origin.x + origin.z * origin.z - 1.0;
        let disc = b * b - 4.0 * a * c;

        if disc >= 0.0 {
            let sqrt_disc = disc.sqrt();
            for t in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
                let hit = origin + dir * t;
                if hit.y >= -1.0 && hit.y <= 1.0 {
                    let normal = Vector3::new(hit.x, 0.0, hit.z).normalize();
                    let theta  = hit.z.atan2(hit.x);
                    let u = 0.5 + theta / (2.0 * std::f32::consts::PI);
                    let v = (hit.y + 1.0) * 0.5;
                    hits.push((t, normal, Vector2::new(u, v)));
                }
            }
        }
    }

    // ---- Bottom cap: y = -1 ----
    if dir.y.abs() > 1e-6 {
        let t   = (-1.0 - origin.y) / dir.y;
        let hit = origin + dir * t;
        if hit.x * hit.x + hit.z * hit.z <= 1.0 {
            let u = hit.x * 0.5 + 0.5;
            let v = hit.z * 0.5 + 0.5;
            hits.push((t, Vector3::new(0.0, -1.0, 0.0), Vector2::new(u, v)));
        }
    }

    // ---- Top cap: y = 1 ----
    if dir.y.abs() > 1e-6 {
        let t   = (1.0 - origin.y) / dir.y;
        let hit = origin + dir * t;
        if hit.x * hit.x + hit.z * hit.z <= 1.0 {
            let u = hit.x * 0.5 + 0.5;
            let v = hit.z * 0.5 + 0.5;
            hits.push((t, Vector3::new(0.0, 1.0, 0.0), Vector2::new(u, v)));
        }
    }

    if hits.len() < 2 {
        return None;
    }

    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let (t_enter, normal_enter, uv_enter) = hits.first().cloned().unwrap();
    let (t_exit,  normal_exit,  uv_exit)  = hits.last().cloned().unwrap();

    Some(IntersectInterval {
        t_enter,
        t_exit,
        normal_enter,
        normal_exit,
        uv_enter,
        uv_exit,
    })
}


/**
 * Returns the full [t_enter, t_exit] interval for a unit cone.
 * x² + z² = (1 − y)², y ∈ [0, 1] — apex at y=1, base disk (radius 1) at y=0.
 *
 * All hits (side + base disk) are collected without a t > 0 guard,
 * sorted, and the outermost pair is returned as the interval.
 */
pub fn intersect_all_unit_cone(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectInterval> {
    let mut hits: Vec<(f32, Vector3<f32>, Vector2<f32>)> = Vec::new();

    // ---- Cone side: x² + z² = (1 − y)² ----
    let a    = dir.x * dir.x + dir.z * dir.z - dir.y * dir.y;
    let b    = 2.0 * (origin.x * dir.x + origin.z * dir.z + (1.0 - origin.y) * dir.y);
    let c    = origin.x * origin.x + origin.z * origin.z - (1.0 - origin.y) * (1.0 - origin.y);
    let disc = b * b - 4.0 * a * c;

    if a.abs() > 1e-6 && disc >= 0.0 {
        let sqrt_disc = disc.sqrt();
        for t in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
            let hit = origin + dir * t;
            if hit.y >= 0.0 && hit.y <= 1.0 {
                // Gradient of x² + z² − (1−y)² points outward
                let normal = Vector3::new(
                    2.0 * hit.x,
                    2.0 * (1.0 - hit.y),
                    2.0 * hit.z,
                ).normalize();
                let theta = hit.z.atan2(hit.x);
                let u = 0.5 + theta / (2.0 * std::f32::consts::PI);
                let v = hit.y;
                hits.push((t, normal, Vector2::new(u, v)));
            }
        }
    }

    // ---- Base disk: y = 0, radius = 1 ----
    if dir.y.abs() > 1e-6 {
        let t   = -origin.y / dir.y;
        let hit = origin + dir * t;
        if hit.x * hit.x + hit.z * hit.z <= 1.0 {
            let u = hit.x * 0.5 + 0.5;
            let v = hit.z * 0.5 + 0.5;
            hits.push((t, Vector3::new(0.0, -1.0, 0.0), Vector2::new(u, v)));
        }
    }

    if hits.len() < 2 {
        return None;
    }

    hits.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let (t_enter, normal_enter, uv_enter) = hits.first().cloned().unwrap();
    let (t_exit,  normal_exit,  uv_exit)  = hits.last().cloned().unwrap();

    Some(IntersectInterval {
        t_enter,
        t_exit,
        normal_enter,
        normal_exit,
        uv_enter,
        uv_exit,
    })
}


/**
 * Returns the full [t_enter, t_exit] interval for a unit torus.
 * Major radius = 0.75, minor radius = 0.25, ring lies in the XY plane.
 *
 * The quartic can yield up to 4 real roots; the smallest and largest are
 * used as t_enter / t_exit. Roots are NOT filtered by sign, so t_enter may
 * be negative when the ray origin is inside the torus tube.
 */
#[allow(non_snake_case)]
pub fn intersect_all_unit_torus(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectInterval> {
    const MAJOR_R: f64 = 0.75;
    const MINOR_R: f64 = 0.25;

    let origin64 = origin.map(|x| x as f64);
    let dir64    = dir.map(|x| x as f64);

    let g = dir64.dot(&dir64);
    let h = 2.0 * origin64.dot(&dir64);
    let i = origin64.dot(&origin64);

    let j = dir64.x * dir64.x + dir64.y * dir64.y;
    let k = 2.0 * (origin64.x * dir64.x + origin64.y * dir64.y);
    let l = origin64.x * origin64.x + origin64.y * origin64.y;

    let s = MAJOR_R * MAJOR_R - MINOR_R * MINOR_R;

    let c4 = g * g;
    let c3 = 2.0 * g * h;
    let c2 = h * h + 2.0 * g * (i + s) - 4.0 * MAJOR_R * MAJOR_R * j;
    let c1 = 2.0 * h * (i + s) - 4.0 * MAJOR_R * MAJOR_R * k;
    let c0 = (i + s) * (i + s) - 4.0 * MAJOR_R * MAJOR_R * l;

    // All real roots, no positivity filter
    let mut roots = solve_quartic(c4, c3, c2, c1, c0);
    if roots.len() < 2 {
        return None;
    }

    roots.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let t_enter = *roots.first().unwrap() as f32;
    let t_exit  = *roots.last().unwrap()  as f32;

    // Compute outward normal at a given t
    let torus_normal_at = |t: f32| -> Vector3<f32> {
        let hit64 = origin64 + dir64 * (t as f64);
        let sum   = hit64.dot(&hit64) + MAJOR_R * MAJOR_R - MINOR_R * MINOR_R;
        let n64   = Vector3::new(
            4.0 * hit64.x * (sum - 2.0 * MAJOR_R * MAJOR_R),
            4.0 * hit64.y * (sum - 2.0 * MAJOR_R * MAJOR_R),
            4.0 * hit64.z * sum,
        );
        if n64.norm_squared() < 1e-20 {
            Vector3::zeros()
        } else {
            n64.normalize().map(|x| x as f32)
        }
    };

    let hit_enter = (origin64 + dir64 * (t_enter as f64)).map(|x| x as f32);
    let hit_exit  = (origin64 + dir64 * (t_exit  as f64)).map(|x| x as f32);

    let (ue, ve) = torus_uv(hit_enter, MAJOR_R as f32);
    let (ux, vx) = torus_uv(hit_exit,  MAJOR_R as f32);

    Some(IntersectInterval {
        t_enter,
        t_exit,
        normal_enter: torus_normal_at(t_enter),
        normal_exit:  torus_normal_at(t_exit),
        uv_enter: Vector2::new(ue, ve),
        uv_exit:  Vector2::new(ux, vx),
    })
}
