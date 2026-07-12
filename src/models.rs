use nalgebra::{Matrix4, Vector3};

/**
 * Scene graph structure
**/
pub struct Node {
    transform_world: Matrix4<f32>,
    transform_local: Matrix4<f32>,
    transform_inverse: Matrix4<f32>,
    mesh_id: Option<String>,
    children: Vec<Node>
}


pub struct PointLight {
    pub position: Vector3<f32>,
    pub intensity: Vector3<f32>,
}


pub struct Scene {
    description: String,
    root: Node,
    point_lights: Vec<PointLight>
}

impl Node {
    pub fn new(mesh_id: Option<String>) -> Self {
        Self {
            transform_world: Matrix4::identity(),
            transform_local: Matrix4::identity(),
            transform_inverse: Matrix4::identity(),
            mesh_id,
            children: vec![],
        }
    }

    pub fn get_mesh_id(&self) -> &Option<String> {
        &self.mesh_id
    }

    pub fn get_transform_local(&self) -> Matrix4<f32> {
        self.transform_local
    }

    pub fn get_transform_world(&self) -> Matrix4<f32> {
        self.transform_world
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
        if !self.children.is_empty() {
            for child in &mut self.children {
                child.set_world_transform(self.transform_local * self.transform_world);
            }
        }
    }

    pub fn set_world_transform(&mut self, transform: Matrix4<f32>) {
        self.transform_world = transform; 
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
            point_lights: vec![]
        }
    }

    pub fn get_root(&self) -> &Node {
        &self.root
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
    
    pub fn print_tree(&self) {
        println!("Scene: {}", self.description);
        self.root.print_tree_recursive(0);
    }
}
