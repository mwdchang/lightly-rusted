use nalgebra::{Matrix4, Vector3};

// JSON parse
use serde_json::Value;
use std::fs::File;


pub struct PointLight {
    pub position: Vector3<f32>,
    pub intensity: Vector3<f32>,
}

pub struct Material {
    pub albedo: Vector3<f32>, // basically diffuse
    pub shine: f32,
}


/**
 * Scene graph structure
**/
pub struct Node {
    transform_world: Matrix4<f32>,
    transform_local: Matrix4<f32>,
    transform_inverse: Matrix4<f32>,
    mesh_id: Option<String>,
    material_id: u32,
    children: Vec<Node>
}



pub struct Scene {
    pub description: String,
    root: Node,
    point_lights: Vec<PointLight>,
    materials: Vec<Material>
}

impl Node {
    pub fn new(mesh_id: Option<String>) -> Self {
        Self {
            transform_world: Matrix4::identity(),
            transform_local: Matrix4::identity(),
            transform_inverse: Matrix4::identity(),
            mesh_id,
            material_id: 0,
            children: vec![],
        }
    }

    pub fn set_mesh_id(&mut self, id: String) {
        self.mesh_id = Some(id)
    }

    pub fn get_mesh_id(&self) -> &Option<String> {
        &self.mesh_id
    }

    pub fn set_material_id(&mut self, id: u32) {
        self.material_id = id
    }

    pub fn get_material_id(&self) -> u32 {
        self.material_id
    }





    pub fn get_transform_local(&self) -> Matrix4<f32> {
        self.transform_local
    }

    pub fn get_transform_world(&self) -> Matrix4<f32> {
        self.transform_world
    }

    pub fn get_transform_inverse(&self) -> Matrix4<f32> {
        self.transform_inverse
    }


    pub fn get_children(&self) -> &Vec<Node> {
        &self.children
    }

    pub fn get_child(&mut self, index: usize) -> Option<&Node> {
        self.children.get(index)
    }


    pub fn add_child(&mut self, child: Node) {
        self.children.push(child);
        if !self.children.is_empty() {
            for child in &mut self.children {
                child.set_world_transform(self.transform_local * self.transform_world);
            }
        }
    }

    pub fn set_transform(&mut self, transform: Matrix4<f32>) {
        self.transform_local = transform; 
        let inv = (
            self.transform_local *
            self.transform_world
        ).try_inverse().unwrap();
        self.transform_inverse = inv;


        if !self.children.is_empty() {
            for child in &mut self.children {
                child.set_world_transform(self.transform_local * self.transform_world);
            }
        }
    }

    pub fn set_world_transform(&mut self, transform: Matrix4<f32>) {
        self.transform_world = transform; 
        let inv = (
            self.transform_local *
            self.transform_world
        ).try_inverse().unwrap();
        self.transform_inverse = inv;

        if !self.children.is_empty() {
            for child in &mut self.children {
                child.set_world_transform(self.transform_local * self.transform_world);
            }
        }
    }

    fn print_tree_recursive(&self, depth: usize) {
        let indent = "  ".repeat(depth);

        println!("{}Node:", indent);

        if let Some(mesh_id) = &self.mesh_id {
            println!("{}  Mesh: {}", indent, mesh_id);
        } else {
            println!("{}  Mesh: None", indent);
        }

        // println!("{}  Local Transform:", indent);
        // println!("{}{:?}", indent, self.transform_local);

        println!("{}  World Transform:", indent);
        // println!("{}{:?}", indent, self.transform_world);
        println!("{}", self.transform_world);

        for (index, child) in self.children.iter().enumerate() {
            println!("{}Child {}:", indent, index);
            child.print_tree_recursive(depth + 1);
        }
    }
}

impl Scene {
    pub fn new(desc: String, root: Node) -> Self {
        Self {
            description: desc,
            root,
            point_lights: vec![],
            materials: vec![]
        }
    }

    pub fn get_root(&self) -> &Node {
        &self.root
    }

    pub fn set_root(&mut self, root: Node) {
        self.root = root
    }


    pub fn get_root_mut(&mut self) -> &mut Node {
        &mut self.root
    }

    pub fn add_point_light(&mut self, pl: PointLight) {
        self.point_lights.push(pl);
    }

    pub fn get_point_lights(&self) -> &Vec<PointLight> {
        &self.point_lights
    }

    pub fn add_material(&mut self, m: Material) {
        self.materials.push(m);
    }

    pub fn get_materials(&self) -> &Vec<Material> {
        &self.materials
    }


    
    pub fn print_tree(&self) {
        println!("Scene: {}", self.description);
        println!("Materials");

        for m in self.get_materials() {
            println!("{} {}", m.shine, m.albedo);
        }

        self.root.print_tree_recursive(0);
    }
}

pub fn read_scene(filename: &str) -> Scene {
    let file = File::open(filename).unwrap();
    let json: Value = serde_json::from_reader(file).unwrap();
    let description = json["description"].as_str().unwrap();

    let mut root = Node::new(None);
    let mut scene = Scene::new(description.to_string(), root);


    println!("Scene json\n {:#?}", json);

    // === Parse lights ===
    json["point_lights"]
        .as_array()
        .unwrap()
        .iter()
        .for_each(|light| {
            let pos = light["position"].as_array().unwrap();
            let intensity = light["intensity"].as_array().unwrap();

            let pl = PointLight {
                position: Vector3::new(
                    pos[0].as_f64().unwrap() as f32,
                    pos[1].as_f64().unwrap() as f32,
                    pos[2].as_f64().unwrap() as f32,
                ),
                intensity: Vector3::new(
                    intensity[0].as_f64().unwrap() as f32,
                    intensity[1].as_f64().unwrap() as f32,
                    intensity[2].as_f64().unwrap() as f32,
                ),
            };
            scene.add_point_light(pl);
        });


    // === Parse materials ===
    json["materials"]
        .as_array()
        .unwrap()
        .iter()
        .for_each(|material| {
            let vals = material["albedo"].as_array().unwrap();
            let shine = material["shine"].as_f64().unwrap() as f32;

            let m = Material {
                albedo: Vector3::new(
                    vals[0].as_f64().unwrap() as f32,
                    vals[1].as_f64().unwrap() as f32,
                    vals[2].as_f64().unwrap() as f32,
                ),
                shine
            };
            scene.add_material(m);
        });
    

    return scene
}

