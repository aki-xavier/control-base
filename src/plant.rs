use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use pga::Multivector;

pub fn rotor_of_quat(q: Quat) -> Multivector {
    pga::mv_scalar(q.w).sub(pga::mv_bivector(q.z, -q.y, q.x, 0.0, 0.0, 0.0))
}

pub fn quat_of_rotor(r: Multivector) -> Quat {
    let b = r.bivector_part();
    Quat {
        w: r.scalar_part(),
        x: -b[2],
        y: b[1],
        z: -b[0],
    }
}

pub fn motor_of_pose(p: Vec3, q: Quat) -> Multivector {
    motor_of_rotor(p, rotor_of_quat(q))
}

pub fn motor_of_rotor(p: Vec3, r: Multivector) -> Multivector {
    pga::translator(p.to_array()).gp(r)
}

pub fn motor_position(m: Multivector) -> Vec3 {
    let c = m.apply(pga::point(0.0, 0.0, 0.0)).coords();
    Vec3::new(c[0], c[1], c[2])
}

pub fn rotvec_between(target: Multivector, current: Multivector) -> Vec3 {
    quat_of_rotor(target)
        .to_mat3()
        .mul(&quat_of_rotor(current).to_mat3().transposed())
        .to_rotvec()
}

pub fn motor_rotation(m: Multivector) -> Multivector {
    let b = m.bivector_part();
    m.grade(0)
        .add(pga::mv_bivector(b[0], b[1], b[2], 0.0, 0.0, 0.0))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Footprint {
    pub offset: Vec3,
    pub back: f64,
    pub front: f64,
    pub half_w: f64,
}

impl Footprint {
    pub const POINT: Footprint = Footprint {
        offset: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        back: 0.0,
        front: 0.0,
        half_w: 0.0,
    };

    pub fn clamp(&self, dx: f64, dy: f64) -> (f64, f64, bool) {
        let mut x = dx;
        let mut y = dy;
        let mut moved = false;
        if x > self.front {
            x = self.front;
            moved = true;
        } else if x < -self.back {
            x = -self.back;
            moved = true;
        }
        if y > self.half_w {
            y = self.half_w;
            moved = true;
        } else if y < -self.half_w {
            y = -self.half_w;
            moved = true;
        }
        (x, y, moved)
    }

    pub fn margin(&self, dx: f64, dy: f64) -> f64 {
        let mut m = self.front - dx;
        if self.back + dx < m {
            m = self.back + dx;
        }
        if self.half_w - dy < m {
            m = self.half_w - dy;
        }
        if self.half_w + dy < m {
            m = self.half_w + dy;
        }
        m
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ContactKind {
    Weld,
    Friction { mu: f64, footprint: Footprint },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExternalContact {
    pub frame: String,
    pub kind: ContactKind,
}

#[derive(Clone, Debug, PartialEq)]
pub enum TaskMap {
    Frame(String),
    Point(String),
}

impl TaskMap {
    pub fn name(&self) -> &str {
        match self {
            TaskMap::Frame(n) => n,
            TaskMap::Point(n) => n,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlantStructure {
    pub dof: usize,
    pub actuated: Vec<bool>,
    pub base_dof: usize,
    pub contacts: Vec<ExternalContact>,
    pub bodies: Vec<String>,
    pub points: Vec<String>,
    pub task_map: TaskMap,
}

impl PlantStructure {
    pub fn fixed_base(dof: usize, bodies: Vec<String>, task_map: TaskMap) -> PlantStructure {
        PlantStructure {
            dof,
            actuated: vec![true; dof],
            base_dof: 0,
            contacts: Vec::new(),
            bodies,
            points: Vec::new(),
            task_map,
        }
    }

    pub fn all_driven(&self) -> bool {
        self.actuated.len() == self.dof && self.actuated.iter().all(|a| *a)
    }

    pub fn base_is_a_state(&self) -> bool {
        self.base_dof > 0
    }

    pub fn braced(&self) -> bool {
        !self.base_is_a_state()
            || self
                .contacts
                .iter()
                .any(|c| matches!(c.kind, ContactKind::Weld))
    }

    pub fn has_body(&self, name: &str) -> bool {
        self.bodies.iter().any(|b| b == name)
    }

    pub fn has_point(&self, name: &str) -> bool {
        self.points.iter().any(|p| p == name)
    }

    pub fn task_is_a_point(&self) -> bool {
        matches!(self.task_map, TaskMap::Point(_))
    }
}

pub trait Plant {
    fn structure(&self) -> PlantStructure;

    fn joint_positions(&mut self) -> Vec<f64>;
    fn joint_velocities(&mut self) -> Vec<f64>;
    fn mass_matrix(&mut self) -> Mat;
    fn bias_torques(&mut self) -> Vec<f64>;
    fn gravity_torques(&mut self) -> Vec<f64>;

    fn configuration_stamp(&mut self) -> Vec<f64> {
        self.joint_positions()
    }

    fn frame_motor(&mut self, name: &str) -> Multivector;
    fn frame_jacobian(&mut self, name: &str) -> Mat;
    fn frame_full_jacobian(&mut self, name: &str) -> Mat;

    fn frame_position(&mut self, name: &str) -> Vec3 {
        motor_position(self.frame_motor(name))
    }

    fn frame_rotation(&mut self, name: &str) -> Multivector {
        motor_rotation(self.frame_motor(name))
    }

    fn point_position(&mut self, name: &str) -> Vec3;
    fn point_jacobian(&mut self, name: &str) -> Mat;

    fn body_frame(&mut self, name: &str) -> (Vec3, Mat);
    fn body_jacobian(&mut self, name: &str) -> Mat;

    fn task_position(&mut self) -> Vec3 {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_position(&f),
            TaskMap::Point(p) => self.point_position(&p),
        }
    }

    fn task_rotation(&mut self) -> Multivector {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_rotation(&f),
            TaskMap::Point(_) => pga::rotor_identity(),
        }
    }

    fn task_jacobian(&mut self) -> Mat {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_jacobian(&f),
            TaskMap::Point(p) => self.point_jacobian(&p),
        }
    }

    fn task_motor(&mut self) -> Multivector {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_motor(&f),
            TaskMap::Point(p) => motor_of_rotor(self.point_position(&p), pga::rotor_identity()),
        }
    }

    fn task_full_jacobian(&mut self) -> Mat {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_full_jacobian(&f),
            TaskMap::Point(_) => Mat::zeros(0, 0),
        }
    }

    fn realize_command(&mut self, tau: &[f64]) -> Vec<f64> {
        tau.to_vec()
    }
}
