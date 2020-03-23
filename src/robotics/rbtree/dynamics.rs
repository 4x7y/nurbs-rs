use crate::robotics::*;
use crate::math::{VectorDf, MatrixDDf, Vector6f, Matrix6f};

impl RigidBodyTree {

    /// Compute the mass matrix, `M`, of the robot in the configuration `q`
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    pub fn mass_matrix(&self, qpos: &VectorDf) -> MatrixDDf {
        let nb = self.num_body();
        let mut iner_comp = vec![Matrix6f::zeros(); nb];  // composite-rigid-body inertia
        let mut xforms    = vec![Matrix6f::zeros(); nb];  // spatial transform from parent of body i to body i
        let nv = self.num_dof();
        let mut mass_mat = MatrixDDf::zeros(nv, nv);

        // preparation
        let mut k = 0;
        for i in 0..nb {
            let body = self.body_id2ptr[i].borrow();
            iner_comp[i] = body.link.inertial.spatial_inertia.clone();
            let pnum = body.qpos_dof();
            let qi = body.get_qpos_from_vec(qpos, k);
            k += pnum;
            let tform = body.tform_body2parent(qi);
            let tform_inv = tform_inv(tform);
            xforms[i] = tform_to_spatial_xform(tform_inv);
        }

        // main loop
        for i in (0..nb).rev() {
            let body = self.body_id2ptr[i].borrow();
            if let Some(joint) = &body.joint {
                let pid = body.parent_index.unwrap();
                iner_comp[pid] = iner_comp[pid] + xforms[i].transpose() * iner_comp[i] * xforms[i];
            }

            if let Some(joint) = &body.joint {
                let a = body.qvel_dof_map();
                if a.1 > a.0 {
                    let si = &joint.screw_axis;
                    let fi = iner_comp[i] * si;
                    mass_mat.slice_mut((a.0, a.0), (a.1 - a.0, a.1 - a.0))
                        .copy_from(&(si.transpose() * fi));

                    let mut fi = xforms[i].transpose() * fi;
                    if let Some(j_tmp) = body.parent_index {
                        let mut j = j_tmp;
                        let mut body_j = self.body_id2ptr[j].borrow();
                        while let Some(joint_j) = &body_j.joint {
                            let sj = &joint_j.screw_axis;
                            let b = body_j.qvel_dof_map();
                            if b.0 < b.1 {
                                let mass_ji = sj.transpose() * fi;
                                mass_mat.slice_mut((b.0, a.0), (b.1 - b.0, a.1 - a.0))
                                    .copy_from(&mass_ji);
                                mass_mat.slice_mut((a.0, b.0), (a.1 - a.0, b.1 - b.0))
                                    .copy_from(&mass_ji.transpose());
                            }
                            fi = xforms[j].transpose() * fi;

                            j = body_j.parent_index.unwrap();
                            body_j = self.body_id2ptr[j].borrow();
                        }
                    }
                }
            }
        }

        return mass_mat;
    }

    /// Compute resultant joint acceleration given joint torques and states.
    ///
    /// Computes the joint accelerations due to gravity, joint velocities
    /// `qvel`, torques applied to the joints `torq`, and external forces
    /// applied to each body `fext`, when ROBOT is at configuration `qpos`.
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    /// - `qvel`: joint velocity                          (nv x 1)
    /// - `torq`: joint torques                           (nv x 1)
    /// - `fext`: external wrenches (moment, force)       (6 x nb)
    pub fn forward_dynamics(&self, qpos: &VectorDf, qvel: &VectorDf,
                            torq: &VectorDf, fext: &Vec<Vector6f>) -> VectorDf {
        unimplemented!()
    }

    /// Compute the joint torques that cancel velocity dependent term.
    ///
    /// computes joint torques `torq` required for ROBOT to cancel the forces
    /// induced by joint velocities `qvel` at the joint configuration `qpos`.
    /// The output of this function is not affected by the Gravity property
    /// of rigid body tree.
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    /// - `qvel`: joint velocity                          (nv x 1)
    pub fn velocity_product(&self, qpos: &VectorDf, qvel: &VectorDf) -> VectorDf {
        unimplemented!()
    }

    /// Compute required joint torques to compensate gravity.
    ///
    /// Computes the joint torques required to hold ROBOT at configuration
    /// `qpos`.
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    pub fn gravity_torque(&self, qpos: &VectorDf) -> VectorDf {
        unimplemented!()
    }
}