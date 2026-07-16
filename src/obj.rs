use tobj;
use nalgebra::Vector3;
use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

pub type ModelCache = HashMap<String, ObjModel>;

// pub static MODELS: LazyLock<Mutex<ModelCache>> = LazyLock::new(|| {
//     Mutex::new(HashMap::new())
// });

#[derive(Clone, Debug)]
pub struct Triangle {
    pub p0: Vector3<f32>,
    pub p1: Vector3<f32>,
    pub p2: Vector3<f32>,

    /// Precomputed geometric normal.
    pub face_normal: Vector3<f32>,

    /// Optional smooth normals from the OBJ.
    pub n0: Option<Vector3<f32>>,
    pub n1: Option<Vector3<f32>>,
    pub n2: Option<Vector3<f32>>,
}

#[derive(Clone, Debug)]
pub struct ObjModel {
    pub triangles: Vec<Triangle>,
    pub bounds: BoundingBox,
}

#[derive(Clone, Debug)]
pub struct BoundingBox {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

impl BoundingBox {
    pub fn empty() -> Self {
        Self {
            min: Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    pub fn expand(&mut self, p: &Vector3<f32>) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);

        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }

    pub fn center(&self) -> Vector3<f32> {
        (self.min + self.max) * 0.5
    }

    pub fn size(&self) -> Vector3<f32> {
        self.max - self.min
    }

    pub fn intersect(
        &self,
        origin: Vector3<f32>,
        dir: Vector3<f32>,
    ) -> bool {
        let inv_dir = Vector3::new(
            1.0 / dir.x,
            1.0 / dir.y,
            1.0 / dir.z,
        );

        let mut tmin = (self.min.x - origin.x) * inv_dir.x;
        let mut tmax = (self.max.x - origin.x) * inv_dir.x;

        if tmin > tmax {
            std::mem::swap(&mut tmin, &mut tmax);
        }

        let mut tymin = (self.min.y - origin.y) * inv_dir.y;
        let mut tymax = (self.max.y - origin.y) * inv_dir.y;

        if tymin > tymax {
            std::mem::swap(&mut tymin, &mut tymax);
        }

        if tmin > tymax || tymin > tmax {
            return false;
        }

        tmin = tmin.max(tymin);
        tmax = tmax.min(tymax);

        let mut tzmin = (self.min.z - origin.z) * inv_dir.z;
        let mut tzmax = (self.max.z - origin.z) * inv_dir.z;

        if tzmin > tzmax {
            std::mem::swap(&mut tzmin, &mut tzmax);
        }

        if tmin > tzmax || tzmin > tmax {
            return false;
        }
        true
    }
}


pub fn load_obj_into_cache(
    cache: &mut ModelCache,
    name: impl Into<String>,
    path: impl AsRef<std::path::Path>,
    recenter: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let (models, _) = tobj::load_obj(
        // path,
        path.as_ref(),
        &tobj::LoadOptions {
            triangulate: true,
            single_index: true,
            ..Default::default()
        },
    )?;

    let mut triangles = Vec::new();
    let mut bounds = BoundingBox::empty();

    for model in models {
        let mesh = &model.mesh;

        // Read vertices.
        let mut vertices = Vec::<Vector3<f32>>::new();
        vertices.reserve(mesh.positions.len() / 3);

        for i in 0..mesh.positions.len() / 3 {
            vertices.push(Vector3::new(
                mesh.positions[3 * i],
                mesh.positions[3 * i + 1],
                mesh.positions[3 * i + 2],
            ));
        }

        // Optionally recenter around the AABB midpoint.
        if recenter {
            let mut min = vertices[0];
            let mut max = vertices[0];

            for v in &vertices {
                min.x = min.x.min(v.x);
                min.y = min.y.min(v.y);
                min.z = min.z.min(v.z);

                max.x = max.x.max(v.x);
                max.y = max.y.max(v.y);
                max.z = max.z.max(v.z);
            }

            let center = (min + max) * 0.5;

            for v in &mut vertices {
                *v -= center;
            }
        }

        let read_normal = |idx: usize| -> Option<Vector3<f32>> {
            if mesh.normals.is_empty() {
                return None;
            }

            Some(
                Vector3::new(
                    mesh.normals[3 * idx],
                    mesh.normals[3 * idx + 1],
                    mesh.normals[3 * idx + 2],
                )
                .normalize(),
            )
        };

        // Convert indexed mesh into a triangle list.
        for face in 0..mesh.indices.len() / 3 {
            let i0 = mesh.indices[3 * face] as usize;
            let i1 = mesh.indices[3 * face + 1] as usize;
            let i2 = mesh.indices[3 * face + 2] as usize;

            let p0 = vertices[i0];
            let p1 = vertices[i1];
            let p2 = vertices[i2];

            let face_normal = (p1 - p0).cross(&(p2 - p0)).normalize();

            triangles.push(Triangle {
                p0,
                p1,
                p2,

                face_normal,

                n0: read_normal(i0),
                n1: read_normal(i1),
                n2: read_normal(i2),
            });
        }
        

        for v in &vertices {
            bounds.expand(v);
        }
    }


    cache.insert(
        name.into(),
        ObjModel {
            triangles,
            bounds
        },
    );

    Ok(())
}
