

pub struct RigidBodyTree {
    pub name: String,                           // name
}

trait RigidBodyTreeModel {
    fn import();                                // import rigid body tree model from urdf
    fn load();                                  // load rigid body tree model
    fn new();                                   // create an empty rigid body tree model
    fn add_body();                              // add body to rigid body tree
    fn get_body();                              // get body handle by name
    fn add_rigid_body_subtree();                // Add subtree to rigid body tree model
}