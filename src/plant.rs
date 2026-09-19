// plant.rs — the Plant contract a controller programs against. It is stated here, and not beside an
// implementation, because the contract's whole point is that the dynamics backend can be swapped
// without the control law changing: a contract that lives with an engine is one an engine can reach
// into. The file names no implementor, and needs none — an implementor is whatever the caller has:
// an engine binding, a model-based view, or a test fake.
//
// STRUCTURE IS DATA, NOT AN ASSUMPTION. This contract used to state one machine's shape and leave it
// unsaid: `compute_jacobian` answered for a frame nothing named, `body_pose` for a body nothing
// named, `link_frame(i)` indexed a chain order only the plant knew, and the absence of a base from
// the state silently meant the base was WELDED — so a machine whose base floats could not implement
// the trait at all, and nothing in the interface said why. `PlantStructure` states all of it: the
// generalized-coordinate count, which of those coordinates are DRIVEN, how many leading ones are the
// base pose, what distal contacts the environment supplies, which bodies exist, which non-frame POINTS
// exist, and WHAT KIND of task the plant presents for control.
//
// THE TASK IS A KIND AND A NAME, NOT JUST A NAME. It used to be a frame name, which could not say that
// a task has no rotation: the centre of mass is a POINT — a mass-weighted quantity of the whole
// configuration, on no link — so a plant presenting it had to invent an orientation and a caller could
// act on the invention. `TaskMap` says which it is, and `task_position` / `task_jacobian` are total for
// both while `task_pose` / `task_full_jacobian` belong to the frame form alone.
//
// The rule this follows: an interface may not depend on WHICH machine is behind it, and must be
// explicit about WHICH KIND of machine it addresses. Names, counts and topology are data; they never
// appear as a convention. Everything a controller decides on is here; nothing about a particular
// machine's identity is.

use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;

/// ContactKind is what the ENVIRONMENT can supply at one frame — the two regimes a brace comes in,
/// and the reason two machines with the same law have different realizations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ContactKind {
    /// The world holds this frame in all six directions, at any magnitude: a weld. Nothing is
    /// infeasible at it, so a wrench demand written in its frame is realizable verbatim — which is
    /// what a base welded to the ground is, and why the arm's loop needs no projection.
    Weld,
    /// Unilateral and friction-limited: `f_z >= 0`, `|f_t| <= mu f_z`. A foot is this, and the
    /// regime it names is not "a smaller weld" — a demand outside it is not saturated, it is
    /// unsupported, and the contact breaks.
    Friction { mu: f64 },
}

/// ExternalContact is ONE source of external wrench a machine leans on: the frame it acts at, and
/// what it can supply there. The base's weld is stated by `base_dof` (see `PlantStructure`) rather
/// than repeated here; this list is the DISTAL contacts — feet, hands, a tool against a wall.
#[derive(Clone, Debug, PartialEq)]
pub struct ExternalContact {
    pub frame: String,
    pub kind: ContactKind,
}

/// TaskMap is WHAT a plant presents for task control, declared as data rather than left to a bare
/// name. The two forms are not interchangeable and that is the point of stating which one it is:
///
///   - a FRAME has a position and an orientation, so it answers six rows;
///   - a POINT has a position and NO orientation, so it answers three.
///
/// The centre of mass is a POINT — a mass-weighted quantity of the WHOLE configuration, attached to no
/// link — and so are a centre of pressure and a zero-moment point. A contract that could only name a
/// frame had no way to say so, and the failure is quiet in both directions: a plant asked for a point's
/// orientation must invent one, and a caller handed an invented orientation can act on it.
#[derive(Clone, Debug, PartialEq)]
pub enum TaskMap {
    /// A frame of this machine's own chain, by name: answered by `frame_pose` / `frame_jacobian` /
    /// `frame_full_jacobian`.
    Frame(String),
    /// A point that is NOT a frame, by name: answered by `point_position` / `point_jacobian`, three
    /// rows and no rotation.
    Point(String),
}

impl TaskMap {
    /// name is the declared name, whichever form it is: what a report should print.
    pub fn name(&self) -> &str {
        match self {
            TaskMap::Frame(n) => n,
            TaskMap::Point(n) => n,
        }
    }
}

/// PlantStructure is a machine's structure AS DATA: what a controller must know before it can decide
/// anything, read once, and — unlike the shape it replaces — stated rather than assumed.
#[derive(Clone, Debug, PartialEq)]
pub struct PlantStructure {
    /// Generalized coordinates. `joint_positions` / `joint_velocities` / `mass_matrix` /
    /// `bias_torques` / `gravity_torques` are all stated in exactly this many entries, and the
    /// naming is deliberate: for a fixed-base machine these ARE the joints, and for a floating one
    /// the leading `base_dof` of them are the base pose the joints follow.
    pub dof: usize,
    /// Driven or not, one entry per generalized coordinate. `dof - driven` is the machine's
    /// underactuation, and it is not a detail: an undriven coordinate's row of the dynamics is a
    /// constraint the environment must satisfy, not a torque the controller may write.
    pub actuated: Vec<bool>,
    /// How many LEADING generalized coordinates are the base pose. `0` is a base welded to the
    /// world (the base is not a state at all, and the machine is braced there in all six
    /// directions); `6` is a floating base, whose wrench must then come from `contacts`.
    pub base_dof: usize,
    /// The distal contacts the environment supplies, and what each can supply.
    pub contacts: Vec<ExternalContact>,
    /// Every body `body_frame` / `body_jacobian` answer for, BY NAME and IN CHAIN ORDER — body `i`
    /// is the distal link of generalized coordinate `i`, which is the index `link_frame(i)` used to
    /// mean without saying so.
    pub bodies: Vec<String>,
    /// Every POINT `point_position` / `point_jacobian` answer for, by name. Points are NOT bodies and
    /// not in chain order: a point is a function of the WHOLE configuration, which is exactly why it
    /// cannot be addressed as a frame and why it is a separate list.
    pub points: Vec<String>,
    /// The task this plant presents for control, answered by `task_position` / `task_jacobian` and —
    /// for a frame — `task_pose` / `task_full_jacobian`. It is a KIND and a NAME here, so the
    /// interface no longer decides silently which frame a loop is controlling, and can no longer
    /// assume that a task has a rotation at all.
    pub task_map: TaskMap,
}

impl PlantStructure {
    /// fixed_base is the structure of a chain welded to the world and fully driven, which is what
    /// the arm is: every coordinate driven, no base in the state, the base its brace in all six
    /// directions. `bodies` is the chain's bodies in chain order and `task_map` the task the plant
    /// presents for control. It has no separate points, because a chain's every point of interest is
    /// on a link and a frame already answers for those.
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

    /// all_driven is `dof == driven`: every generalized coordinate is written by a torque. It is
    /// the precondition of a realization that maps a wrench demand straight through `J'`.
    pub fn all_driven(&self) -> bool {
        self.actuated.len() == self.dof && self.actuated.iter().all(|a| *a)
    }

    /// base_is_a_state is the base's answer as data: true for a floating machine, false for a weld.
    pub fn base_is_a_state(&self) -> bool {
        self.base_dof > 0
    }

    /// braced is whether the environment holds the machine somewhere in all six directions — at the
    /// welded base, or at a distal weld. A wrench demand is realizable verbatim only when it is.
    pub fn braced(&self) -> bool {
        !self.base_is_a_state()
            || self
                .contacts
                .iter()
                .any(|c| matches!(c.kind, ContactKind::Weld))
    }

    /// has_body answers whether `name` is one of this machine's bodies.
    pub fn has_body(&self, name: &str) -> bool {
        self.bodies.iter().any(|b| b == name)
    }

    /// has_point answers whether `name` is one of this machine's non-frame points.
    pub fn has_point(&self, name: &str) -> bool {
        self.points.iter().any(|p| p == name)
    }

    /// task_is_a_point is the task's answer as data: true when the task is a point, so it has a
    /// position and a three-row mapping and NO rotation. A caller that needs an orientation can read
    /// this instead of acting on the identity `task_pose` hands over by convention, and a loop whose
    /// reading is six planes can refuse the plant rather than drive a rotation that is not there.
    pub fn task_is_a_point(&self) -> bool {
        matches!(self.task_map, TaskMap::Point(_))
    }
}

/// The narrow contract the controllers consume, so swapping the dynamics backend never touches the
/// control law. Structure first (`structure`), then the numeric core in the generalized coordinates
/// it declares (positions, velocities, mass matrix, bias and gravity), then the mapping a task is
/// written in — the declared task, this machine's bodies, and its non-frame points, all BY NAME.
pub trait Plant {
    /// structure is the machine as data: DOF, actuation, base, contacts, bodies, points and the task
    /// map. It takes `&self` and not `&mut self` because a declaration is not a state change — a
    /// caller may read it before it has a mutable borrow, and a controller reads it once.
    fn structure(&self) -> PlantStructure;

    fn joint_positions(&mut self) -> Vec<f64>;
    fn joint_velocities(&mut self) -> Vec<f64>;
    fn mass_matrix(&mut self) -> Mat;
    fn bias_torques(&mut self) -> Vec<f64>;
    fn gravity_torques(&mut self) -> Vec<f64>;

    /// configuration_stamp is the quantity a loop uses to decide that the CONFIGURATION has moved —
    /// and nothing else. Its one requirement is that it changes when the configuration does, in
    /// `structure().dof` entries; it is never a position, is never differenced, and no controller may
    /// read a coordinate out of it.
    ///
    /// It is separate from `joint_positions` because the two are not the same requirement, and one
    /// machine in this tree needs the weaker one: a stance-held reduction's coordinates are a
    /// velocity-level subspace of the machine's, so it has no position vector to hand over, while what
    /// the loop actually asks for — "has the metric's configuration moved?" — it can answer. The
    /// default IS `joint_positions`, so a plant whose coordinates are a configuration owes nothing here.
    fn configuration_stamp(&mut self) -> Vec<f64> {
        self.joint_positions()
    }

    /// frame_pose / frame_jacobian / frame_full_jacobian are the FRAME mapping, addressed BY NAME:
    /// any of `structure().bodies`, or the task when it is a frame. The pose is (position, wxyz
    /// quaternion), the Jacobians are 3 x dof (linear) and 6 x dof ([linear; angular]).
    fn frame_pose(&mut self, name: &str) -> (Vec3, Quat);
    fn frame_jacobian(&mut self, name: &str) -> Mat;
    fn frame_full_jacobian(&mut self, name: &str) -> Mat;

    /// point_position / point_jacobian are the NON-FRAME points, addressed by name: any of
    /// `structure().points`. A point is a quantity of the whole configuration with no orientation
    /// and no link of its own, so it has a position and a 3 x dof Jacobian and nothing else — which
    /// is the difference a bare frame name could not carry.
    fn point_position(&mut self, name: &str) -> Vec3;
    fn point_jacobian(&mut self, name: &str) -> Mat;

    /// body_frame / body_jacobian are the whole-body enumeration, also addressed by name: the
    /// (position, rotation) of one body's frame, and its 3 x dof Jacobian. A whole-body layer
    /// iterates `structure().bodies` and asks for each one, so the order is the plant's declared
    /// one rather than a convention it kept to itself.
    fn body_frame(&mut self, name: &str) -> (Vec3, Mat);
    fn body_jacobian(&mut self, name: &str) -> Mat;

    /// task_position / task_jacobian read the task the plant DECLARED (`structure().task_map`)
    /// rather than one the interface assumes, and they are TOTAL: both forms of task have a position
    /// and a three-row mapping. They are what `body_pose` / `compute_jacobian` silently were; a
    /// caller that wants a different frame names it and calls `frame_*` directly.
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

    /// task_pose / task_full_jacobian are the SIX-ROW pair, and only a FRAME has them.
    ///
    /// For a POINT, `task_pose` returns the position with the IDENTITY rotation and
    /// `task_full_jacobian` returns an EMPTY matrix. Neither is a quantity: a point has no
    /// orientation, so the identity is a convention and the empty matrix is the "no such matrix"
    /// sentinel this tree already uses (`PlaneTaskLoop::lam` starts as `Mat::zeros(0, 0)`).
    /// `structure().task_is_a_point()` is how a caller avoids both, and a loop whose reading is six
    /// planes can refuse the plant instead of driving a rotation that does not exist.
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
}
