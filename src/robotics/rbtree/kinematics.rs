use crate::robotics::*;
use crate::math::{VectorDf, MatrixDDf, Matrix4f};
use std::cmp::{max, min};

impl RigidBodyTree {

    /// Compute all transformation matrix from body frame `{B}` to world
    /// frame `{W}`.
    pub fn forward_kinematics(&self, qpos: &VectorDf) -> Vec<Matrix4f> {
        let n = self.num_body();
        let mut tforms = vec![Matrix4f::identity(); n];
        let mut k = 0;

        for i in 0..n {
            let body = self.bodies[i].borrow();
            let pnum = body.qpos_dof();
            let qi = body.get_qpos_from_vec(qpos, k);
            tforms[i] = body.tform_body2parent(&qi);

            k = k + pnum;
            if let Some(parent_idx) = body.parent_index {
                tforms[i] = tforms[parent_idx] * tforms[i];
            }
        }

        return tforms;
    }

    /// Compute the shortest kinematic path from body `from` to body
    /// `to` in the rigid body tree.
    pub fn kinematics_tree_path(&self, from: &str, to: &str) -> Vec<usize> {
        let id_from = self.body_index_from_name(from);
        let id_to = self.body_index_from_name(to);

        let mut path = Vec::new();
        let id_larger = max(id_to, id_from);
        let id_smaller = min(id_to, id_from);
        let mut curr = id_larger;
        path.push(curr);
        while curr > id_smaller {
            if let Some(parent) = self.parent_index(curr) {
                curr = parent;
                path.push(curr);
            } else {
                break;
            }
        }

        if curr != id_smaller {
            return Vec::new();
        }

        if id_to > id_from {
            path.reverse();
        }
        return path;
    }

    /// Get the transform T that converts points originally expressed
    /// in `{from}` frame to `{to}` frame
    pub fn get_transform(&self, qpos: &VectorDf, from: &str, to: &str) -> Matrix4f {
        let tforms = self.forward_kinematics(qpos);

        let id_from = self.body_index_from_name(from);
        let tform_from2world = tforms[id_from];
        let id_to = self.body_index_from_name(to);
        let tform_to2world = tforms[id_to];
        let tform_world2to = tform_inv(tform_to2world);
        return tform_world2to * tform_from2world;
    }

    /// Get the transform T that converts points originally expressed
    /// in `{from}` frame to world frame `{W}`
    pub fn get_transform_to_world(&self, qpos: &VectorDf, from: &str) -> Matrix4f {
        let tforms  = self.forward_kinematics(qpos);
        let from_id = self.body_index_from_name(from);
        return tforms[from_id];
    }


    /// Compute the geometric Jacobian.
    ///
    /// Computes the geometric Jacobian for the body end-effector name
    /// in rbtree under the configuration `qpos`. The Jacobian matrix
    /// `jac` is of size 6xN, where `N` is the number of degrees of
    /// freedom. The Jacobian maps joint-space velocity to the Cartesian
    /// space end-effector velocity relative to the base coordinate frame.
    pub fn geometric_jacobian(&self, qpos: &VectorDf, name: &str) -> MatrixDDf {
        unimplemented!()
    }

}