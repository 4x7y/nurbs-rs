use crate::robotics::*;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use crate::math::{Matrix4f, Vector3f};
use std::collections::HashMap;
use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::resource::Mesh;
use crate::utils::*;

pub struct Arrow {
    pub radius: f32,
    pub length: f32,
    pub color: Color,
}

pub struct SimScene {
    pub window: Window,
    pub node_name2ptr: HashMap<String, SceneNode>,
}

impl SimScene {
    pub fn new(name: &str) -> Self {
        let mut scene = SimScene {
            window: Window::new(name),
            node_name2ptr: HashMap::new(),
        };
        scene.window.set_light(Light::StickToCamera);
        scene.window.set_background_color(1., 1., 1.);

        return scene;
    }

    pub fn new_with_size(name: &str, width: u32, height: u32) -> Self {
        let mut scene = SimScene {
            window: Window::new_with_size(name, width, height),
            node_name2ptr: HashMap::new(),
        };
        scene.window.set_light(Light::StickToCamera);

        return scene;
    }

    pub fn load(&mut self, model: &SimModel) {
        let objs = model.renderable_objects();
    }

    pub fn render(&mut self, model: &SimModel, data: &SimData) {
        let nodes = model.scene_nodes(data);
    }

    pub fn add_arrow(&mut self, arrow: Arrow, dir: Vector3f) {
        let mut cy = self.window.add_cylinder(arrow.radius, arrow.length);
        cy.set_color(1., 1., 1.);
    }
}

pub struct SimData {
    pub data: i32,
}

pub struct SimObjectState {
    pub name: String,
    pub tform: Matrix4f,
}

pub struct SimModel {
    pub rbtrees: Vec<RigidBodyTree>,
}

pub enum RenderableObject {
    MESH(String, Rc<RefCell<Mesh>>),
}

impl SimModel {

    pub fn new() -> Self {
        SimModel {
            rbtrees: vec![]
        }
    }

    pub fn add_rbtree(&mut self, rbtree: RigidBodyTree) {
        self.rbtrees.push(rbtree);
    }

    pub fn step(&mut self) {
        unimplemented!()
    }

    pub fn scene_nodes(&self, data: &SimData) -> Vec<SimObjectState> {
        unimplemented!()
    }

    pub fn renderable_objects(&self) -> Vec<RenderableObject> {
        unimplemented!()
    }
}