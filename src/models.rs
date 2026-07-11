use nalgebra::{Matrix4};

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

    pub fn get_transform_local(&self) -> Matrix4<f32> {
        self.transform_local
    }

    pub fn get_transform_world(&self) -> Matrix4<f32> {
        self.transform_world
    }

    pub fn get_child(&mut self, index: usize) -> Option<&Node> {
        self.children.get(index)
    }



    pub fn add_child(&mut self, child: Node) {
        self.children.push(child);
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
}

pub struct Scene {
    description: String,
    root: Node
}
