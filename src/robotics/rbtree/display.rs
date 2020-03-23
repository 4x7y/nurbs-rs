use std::fmt;
use prettytable::{Cell, Row, Table, format};
use crate::robotics::RigidBodyTree;

impl fmt::Display for RigidBodyTree {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut table = Table::new();
        table.set_format(*format::consts::FORMAT_NO_LINESEP_WITH_TITLE);
        table.set_titles(row![
        "Idx",
        format!("Body Name ({} + {})", self.num_fixed_body, self.num_non_fixed_body),
        "Joint Name", "Joint Type", "qpos Map", "qvel Map", "Parent Name", "Children Name(s)"]);

        let mut index = 0;
        for body in &self.bodies {
            let body = body.borrow();
            let name = &body.link.name;
            let none = "None".to_string();
            let parent_name = self.parent.get(name).unwrap_or(&none);
            let mut children_names = "".to_string();
            match self.children.get(name) {
                None => {},
                Some(children) => {
                    for child in children {
                        children_names.push_str(&child);
                        children_names.push_str(" ");
                    }
                },
            };
            table.add_row(row![
                Cell::new(format!("{}", index).as_ref()),
                Cell::new(&body.name()),
                Cell::new(&body.joint_name()),
                Cell::new(&body.joint_type_name()),
                Cell::new(format!("({}, {})",
                              body.qpos_dof_map().0,
                              body.qpos_dof_map().1).as_ref()),
                Cell::new(format!("({}, {})",
                              body.qvel_dof_map().0,
                              body.qvel_dof_map().1).as_ref()),
                Cell::new(parent_name),
                Cell::new(&children_names),
            ]);
            index += 1;
        }

        write!(f, "{}", table.to_string())
    }
}