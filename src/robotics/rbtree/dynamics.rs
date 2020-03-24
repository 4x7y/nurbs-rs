use crate::robotics::*;
use crate::math::{VectorDf, MatrixDDf, Vector6f, Matrix6f};

impl RigidBodyTree {

    /// Compute the mass matrix, `M`, of the robot in the configuration `q`
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    fn mass_matrix_internal(&self, qpos: &VectorDf) -> MatrixDDf {
        let nb = self.num_body();
        // composite-rigid-body inertia
        let mut iner_comp = vec![Matrix6f::zeros(); nb];
        // spatial transform from parent of body i to body i
        let mut xforms    = vec![Matrix6f::zeros(); nb];
        let nv = self.num_dof();
        let mut mass_mat = MatrixDDf::zeros(nv, nv);

        // preparation
        let mut k = 0;
        for i in 0..nb {
            let body = self.bodies[i].borrow();
            iner_comp[i] = body.link.inertial.spatial_inertia.clone();
            let pnum = body.qpos_dof();
            let qi = body.get_qpos_from_vec(qpos, k);
            k += pnum;
            let tform = body.tform_body2parent(&qi);
            let tform_inv = tform_inv(tform);
            xforms[i] = tform_to_spatial_xform(tform_inv);
        }

        // main loop
        for i in (0..nb).rev() {
            let body = self.bodies[i].borrow();
            if let Some(pid) = body.parent_index {
                iner_comp[pid] = iner_comp[pid] + xforms[i].transpose() * iner_comp[i] * xforms[i];
            }

            let a = body.qvel_dof_map();
            if a.1 > a.0 {
                let si = &body.joint.screw_axis;
                let mut fi = iner_comp[i] * si;
                mass_mat.slice_mut((a.0, a.0), (a.1 - a.0, a.1 - a.0))
                    .copy_from(&(si.transpose() * &fi));

                fi = xforms[i].transpose() * &fi;
                let mut j_opt = body.parent_index;

                while let Some(j) = j_opt {
                    let body_j = self.bodies[j].borrow();
                    let joint_j = &body_j.joint;

                    let sj = &joint_j.screw_axis;
                    let b = body_j.qvel_dof_map();
                    if b.0 < b.1 {
                        let mass_ji = sj.transpose() * &fi;
                        mass_mat.slice_mut((b.0, a.0), (b.1 - b.0, a.1 - a.0))
                            .copy_from(&mass_ji);
                        mass_mat.slice_mut((a.0, b.0), (a.1 - a.0, b.1 - b.0))
                            .copy_from(&mass_ji.transpose());
                    }
                    fi = xforms[j].transpose() * &fi;

                    j_opt = body_j.parent_index;
                }
            }
        }

        return mass_mat;
    }

    /// Compute the mass matrix, `M`, of the robot in the configuration `q`
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    pub fn mass_matrix(&self, qpos: &VectorDf) -> MatrixDDf {
        return self.mass_matrix_internal(qpos);
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

    pub fn forward_dynamics_crb(&self, qpos: &VectorDf, qvel: &VectorDf,
                                torq: &VectorDf, fext: &Vec<Vector6f>) -> VectorDf {
        unimplemented!()
    }

    /// Inverse dynamics
    ///
    /// Given the current (or desired) joint positions, and velocities, the
    /// desired joint accelerations, and optionally the external forces
    /// applied to each body, compute the required joint torque.
    ///
    /// # Arguments
    ///
    /// - `qpos`: joint configuration                     (np x 1)
    /// - `qvel`: joint velocity                          (nv x 1)
    /// - `qacc`: joint acceleration                      (nv x 1)
    /// - `fext`: external wrenches (moment, force)       (6 x nb)
    pub fn inverse_dynamics(&self,
                            qpos: &VectorDf, qvel: &VectorDf, qacc: &VectorDf,
                            fext: &Vec<Vector6f>) -> VectorDf {
        let sacc_g = Vector6f::new(0., 0., 0., -self.gravity[0], -self.gravity[1], -self.gravity[2]);
        let nb = self.num_body();
        let nv = self.num_dof();

        let mut xforms_body = vec![Matrix6f::zeros(); nb]; // spatial transform from parent(i) to body i
        let mut xforms_base = vec![Matrix6f::zeros(); nb]; // spatial transform from body i to base
        let mut svel_jnt    = vec![Vector6f::zeros(); nb]; // spatial velocity across joint i, in body frame i
        let mut svel_body   = vec![Vector6f::zeros(); nb]; // spatial velocity of body i
        let mut sacc_body   = vec![Vector6f::zeros(); nb]; // spatial acceleration of body i
        let mut sfrc_body   = vec![Vector6f::zeros(); nb]; // spatial force exerted on body i
        let mut torq        = VectorDf::zeros(nv);         // output generalized force

        // forward iterations
        for i in 0..nb {
            let body_i  = self.bodies[i].borrow();
            let joint_i = &body_i.joint;

            let screw_axis = &joint_i.screw_axis;
            let a = body_i.qpos_dof_map();
            let b = body_i.qvel_dof_map();

            let qpos_i = joint_i.get_qpos(&qpos, a.0);
            let qvel_i = joint_i.get_qvel(&qvel, b.0);
            let qacc_i = joint_i.get_qacc(&qacc, b.0);
            let tform_i = joint_i.tform_body2parent(&qpos_i);
            svel_jnt[i] = screw_axis * qvel_i;

            let tform_inv_i = tform_inv(tform_i);
            xforms_body[i] = tform_to_spatial_xform(tform_inv_i);

            let pid = self.bodies[i].borrow().parent_index;
            if let Some(pid) = pid {
                svel_body[i] = svel_jnt[i] + xforms_body[i] * svel_body[pid];
                sacc_body[i] = xforms_body[i] * sacc_body[pid] + screw_axis * qacc_i
                    + cross_motion(svel_body[i], svel_jnt[i]);
                xforms_base[i] = xforms_base[i] * tform_to_spatial_xform(tform_i);
            } else {
                // parent is base
                svel_body[i] = svel_jnt[i];
                sacc_body[i] = xforms_body[i] * sacc_g + screw_axis * qacc_i;
                xforms_base[i] = tform_to_spatial_xform(tform_i);
            }

            let iner = &body_i.link.inertial.spatial_inertia;
            let h    = iner * svel_body[i];
            sfrc_body[i] = iner * sacc_body[i] + cross_force(svel_body[i], h)
                - xforms_base[i].transpose() * fext[i];
        }

        // backward iteration
        for i in (0..nb).rev() {
            let body_i  = self.bodies[i].borrow();
            let joint_i = &body_i.joint;

            if let JointType::Fixed = joint_i.joint_type {
            } else {
                let screw_axis = &joint_i.screw_axis;
                let torq_i = screw_axis.transpose() * sfrc_body[i];
                let b = body_i.qvel_dof_map();
                torq.slice_mut((b.0, 0), (b.1 - b.0, 1)).copy_from(&torq_i);
            }
            if let Some(pid) = body_i.parent_index {
                sfrc_body[pid] = sfrc_body[pid] + xforms_body[i].transpose() * sfrc_body[i];
            }
        }

        return torq;
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