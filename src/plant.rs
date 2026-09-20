// plant.rs — why the contract lives here: a contract stated beside an implementation is one that
// implementation can reach into, and the whole point of the contract is that the backend can be
// swapped without the interface knowing it.
//
// WHY THE STRUCTURE IS DATA: a shape left unsaid is one nobody can satisfy. "No base in the state"
// silently meant "welded", so a floating machine could not be described at all, and names that were
// only conventions — the body a pose answered for, the chain order a link index meant — are exactly
// the assumptions a contract cannot afford.
//
// WHY THE TASK IS A KIND AND A NAME: a rotation cannot be made up. A point — a centre of mass, a
// centre of pressure, a zero-moment point — is a quantity of the whole configuration on no link, so
// it has no orientation; a contract that could only name a frame forced one to be invented and then
// handed out as if it meant something.

use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use pga::Multivector;

/// rotor_of_quat: the contract's rotation as a PGA rotor (Cl(3,0,1), the even subalgebra). Why this is
/// a conversion and not a translation between two shapes: a rotor IS the quaternion's four numbers in
/// the algebra's own basis, so the same rotation has one representation and this is it — a caller that
/// reads poses through geometric algebra gets its element here rather than reassembling one.
///
/// The rotor is `w - (q_z e12 - q_y e13 + q_x e23)`, the same convention control-model's `PgaFk` and
/// `pga_layer` are written against, and the one `Plant::frame_motor` below levels.
pub fn rotor_of_quat(q: Quat) -> Multivector {
    pga::mv_scalar(q.w).sub(pga::mv_bivector(q.z, -q.y, q.x, 0.0, 0.0, 0.0))
}

/// quat_of_rotor: the INVERSE reading of `rotor_of_quat`, and why the pair lives together — the signs
/// below are exactly the ones that function subtracts, and a second copy of them is where the two would
/// drift apart.
///
/// Why it is the literal inverse rather than a matrix round trip through `Quat::from_mat3`: that route
/// renormalizes the sign of `w`, so it would return a rotation equal to `r` but not the four numbers `r`
/// carries, and `rotor_of_quat(quat_of_rotor(r))` is what the callers above are entitled to.
pub fn quat_of_rotor(r: Multivector) -> Quat {
    let b = r.bivector_part();
    Quat {
        w: r.scalar_part(),
        x: -b[2],
        y: b[1],
        z: -b[0],
    }
}

/// motor_of_pose: a pose as one PGA motor, `T(p) R` — the rotation applied first, then the translation.
/// Why this is worth a type: the (position, rotation) PAIR is affine — composing two poses is a matrix
/// product and a vector addition — while a motor composes by one product, so a caller holding poses as
/// motors never takes them apart. The translation is the ideal part, which the degenerate `e0` makes
/// multiplicative rather than affine.
pub fn motor_of_pose(p: Vec3, q: Quat) -> Multivector {
    pga::translator(p.to_array()).gp(rotor_of_quat(q))
}

/// Footprint is where a friction contact's pressure may act, in that contact frame's own coordinates:
/// the normal is the frame's +z, the patch runs `front` ahead and `back` behind the point the ground
/// acts at (`offset`), and `half_w` across. Why a patch and not a point: the wrench a sole supplies
/// carries a MOMENT, and the constraint such a contact really lives under is that the moment be one a
/// pressure distribution inside the patch produces — a point contact could hold no ankle moment at
/// all, which is the direction a standing machine's balance is held in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Footprint {
    /// where the patch sits in the contact frame [m]: the point the ground acts at
    pub offset: Vec3,
    /// travel along the frame's x, behind and ahead of `offset` [m], both >= 0
    pub back: f64,
    pub front: f64,
    /// half extent across the frame's y [m], >= 0
    pub half_w: f64,
}

impl Footprint {
    /// A patch of no extent at the frame's origin — what a POINT contact supplies, and the declaration
    /// that no moment about that contact can be held.
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

    /// Clamps a pressure point's offset, in the frame's x and y, into the patch. Why it reports the
    /// move: a pressure point at the edge is the state every balance authority is spent at, and a
    /// caller that cannot see it has no anti-windup and no readout.
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

    /// The travel left before the nearest edge [m]. Why the minimum over the four: the authority left is
    /// bounded by whichever edge the pressure point reaches first, and that distance times the normal
    /// load IS the moment the contact can still add.
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

/// ContactKind is what the environment supplies at one frame. The two regimes are not one scaled:
/// a demand outside a friction cone is unsupported rather than saturated, and the contact breaks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ContactKind {
    /// Holds all six directions at any magnitude, so a wrench demand written in this frame is
    /// realizable verbatim.
    Weld,
    /// Unilateral and friction-limited: `f_z >= 0`, `|f_t| <= mu f_z`, acting somewhere in `footprint`.
    /// Why the patch is stated with the cone and not left to the caller: the moment a contact can supply
    /// is the normal force times the distance its pressure point may travel, so a cone without a patch
    /// does not say what the contact holds.
    Friction { mu: f64, footprint: Footprint },
}

/// ExternalContact is one distal source of external wrench: the frame it acts at, and what it can
/// supply there. The base's weld is stated by `base_dof` rather than repeated here, so this list is
/// the distal contacts alone.
///
/// `frame` is a name the rest of this contract answers for — `frame_pose` and `frame_full_jacobian` —
/// because a load distribution has to ask for the mapping the wrench is transmitted through, and a
/// contact named something no Jacobian answers for would be a declaration nobody can act on.
#[derive(Clone, Debug, PartialEq)]
pub struct ExternalContact {
    pub frame: String,
    pub kind: ContactKind,
}

/// TaskMap is what a plant presents for task control, declared as data rather than left to a bare
/// name. The two forms are not interchangeable: a frame answers six rows, a point three. A contract
/// that could only name a frame had no way to say "this has no rotation", and the failure is quiet
/// in both directions — the orientation gets invented, and then acted on.
#[derive(Clone, Debug, PartialEq)]
pub enum TaskMap {
    /// A frame of this machine's own chain, by name.
    Frame(String),
    /// A point that is not a frame, by name: three rows and no rotation.
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

/// PlantStructure is a machine's structure as data: everything the interface must know about a
/// machine before any quantity is asked of it, stated rather than assumed.
#[derive(Clone, Debug, PartialEq)]
pub struct PlantStructure {
    /// Why the naming is deliberate: for a fixed-base machine these are the joints, and for a
    /// floating one the leading `base_dof` of them are the base pose the joints follow.
    pub dof: usize,
    /// One entry per generalized coordinate. Why it is not a detail: an undriven row of the dynamics
    /// is a constraint the environment must satisfy, not a torque to be written.
    pub actuated: Vec<bool>,
    /// How many leading coordinates are the base pose. Why it matters: a floating base's wrench must
    /// then come from `contacts`.
    pub base_dof: usize,
    /// The distal contacts the environment supplies.
    pub contacts: Vec<ExternalContact>,
    /// Every body `body_frame` / `body_jacobian` answer for, by name and in chain order, so the
    /// order is declared rather than kept.
    pub bodies: Vec<String>,
    /// Every point `point_position` / `point_jacobian` answer for, by name. Why they are a separate
    /// list: a point is a function of the whole configuration, so it is not a frame and not in chain
    /// order.
    pub points: Vec<String>,
    /// The task this plant presents for control. Why it is a kind and not just a name: the interface
    /// can no longer decide silently which frame is being controlled, nor assume a task has a
    /// rotation at all.
    pub task_map: TaskMap,
}

impl PlantStructure {
    /// Why a welded chain needs neither a distal contact nor a separate point list: every point of
    /// interest is on a link, where a frame already answers for it.
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

    /// Why this predicate is needed: a realization that maps a wrench demand straight through `J'`
    /// is only valid when every generalized coordinate is written by a torque.
    pub fn all_driven(&self) -> bool {
        self.actuated.len() == self.dof && self.actuated.iter().all(|a| *a)
    }

    pub fn base_is_a_state(&self) -> bool {
        self.base_dof > 0
    }

    /// Why it matters: a wrench demand is realizable verbatim only when something holds the machine
    /// in all six directions.
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

    /// Why it is exposed as data: reading it is how a six-row quantity is avoided instead of acting
    /// on the identity `task_pose` returns by convention, which is a placeholder and not a quantity.
    pub fn task_is_a_point(&self) -> bool {
        matches!(self.task_map, TaskMap::Point(_))
    }
}

/// The narrow contract. Why narrow: swapping the backend must never reach the interface, so every
/// term is stated in the coordinates the structure declares and every name is declared rather than
/// assumed.
pub trait Plant {
    /// Takes `&self` because a declaration is not a state change: it can be read before any mutable
    /// borrow of the numeric core.
    fn structure(&self) -> PlantStructure;

    fn joint_positions(&mut self) -> Vec<f64>;
    fn joint_velocities(&mut self) -> Vec<f64>;
    fn mass_matrix(&mut self) -> Mat;
    fn bias_torques(&mut self) -> Vec<f64>;
    fn gravity_torques(&mut self) -> Vec<f64>;

    /// Why it is separate from `joint_positions`: the two are not the same requirement, and the
    /// weaker one is real. Coordinates can be a velocity-level subspace with no position vector to
    /// hand over, while "has the configuration moved?" can still be answered. The default IS
    /// `joint_positions`, so a plant that has a configuration owes nothing here.
    fn configuration_stamp(&mut self) -> Vec<f64> {
        self.joint_positions()
    }

    /// The frame mapping, addressed by name. Why the pose carries its convention: the quaternion is
    /// wxyz, and the two Jacobians are 3 x dof (linear) and 6 x dof ([linear; angular]).
    fn frame_pose(&mut self, name: &str) -> (Vec3, Quat);
    fn frame_jacobian(&mut self, name: &str) -> Mat;
    fn frame_full_jacobian(&mut self, name: &str) -> Mat;

    /// The SAME frame pose, as one PGA motor (`motor_of_pose`), for the callers that read geometry
    /// through the algebra: without it every one of them reassembles the pair itself, and a pose built
    /// per caller is as many conventions as there are callers. It answers for exactly the names
    /// `frame_pose` answers for, so a caller cannot ask for a motor of something it cannot ask a pose of.
    fn frame_motor(&mut self, name: &str) -> Multivector {
        let (p, q) = self.frame_pose(name);
        motor_of_pose(p, q)
    }

    /// The non-frame points, addressed by name. Why they are not frames: a point is a quantity of
    /// the whole configuration, so it has no orientation and no link of its own, and a bare frame
    /// name could not carry that difference.
    fn point_position(&mut self, name: &str) -> Vec3;
    fn point_jacobian(&mut self, name: &str) -> Mat;

    /// The whole-body enumeration, addressed by name and in the declared order.
    fn body_frame(&mut self, name: &str) -> (Vec3, Mat);
    fn body_jacobian(&mut self, name: &str) -> Mat;

    /// Why these are total and the six-row pair below is not: both forms of task have a position and
    /// a three-row mapping, but only a frame has a rotation.
    fn task_position(&mut self) -> Vec3 {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_pose(&f).0,
            TaskMap::Point(p) => self.point_position(&p),
        }
    }

    fn task_jacobian(&mut self) -> Mat {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_jacobian(&f),
            TaskMap::Point(p) => self.point_jacobian(&p),
        }
    }

    /// Why the degenerate answers are conventions and not quantities: a point has no orientation, so
    /// the identity rotation stands in for one and the empty matrix is the "no such matrix"
    /// sentinel. `structure().task_is_a_point()` is how both are avoided.
    fn task_pose(&mut self) -> (Vec3, Quat) {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_pose(&f),
            TaskMap::Point(p) => (self.point_position(&p), Quat::IDENTITY),
        }
    }

    fn task_full_jacobian(&mut self) -> Mat {
        match self.structure().task_map {
            TaskMap::Frame(f) => self.frame_full_jacobian(&f),
            TaskMap::Point(_) => Mat::zeros(0, 0),
        }
    }

    /// The task pose as one PGA motor, the same reading `task_pose` gives. Why the point case is the
    /// same caveat and not a new one: a point has no orientation, so the rotation carried here is the
    /// identity convention `task_pose` already stands in with. Read it only when
    /// `structure().task_is_a_point()` is false; a caller that wants a motor of a frame and knows which
    /// frame asks `frame_motor` for it by name and never meets the convention at all.
    fn task_motor(&mut self) -> Multivector {
        let (p, q) = self.task_pose();
        motor_of_pose(p, q)
    }

    /// The command the machine's own actuators take, from a generalized force in the coordinates
    /// `structure()` declared. Why the PLANT owes this and a controller does not: forming a force in
    /// those coordinates is all a loop can do, and they are the actuators only for a machine that is
    /// fully actuated in its own configuration — a stance-held reduction is a VELOCITY-LEVEL subspace,
    /// where the same force has to be lifted back onto the joints, and which of the lift's directions
    /// to spend is a policy its own model has to state rather than a caller invent. The default IS the
    /// identity, so a machine whose coordinates are its torques owes nothing here.
    fn realize_command(&mut self, tau: &[f64]) -> Vec<f64> {
        tau.to_vec()
    }
}
