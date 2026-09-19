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
// base pose, what distal contacts the environment supplies, which bodies exist, and which frame the
// plant presents for task control.
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
    /// The frame this plant presents for task control, answered by `frame_pose` /
    /// `frame_jacobian` / `frame_full_jacobian`. It is a NAME here, so the interface no longer
    /// decides silently which frame a loop is controlling.
    pub task_frame: String,
}

impl PlantStructure {
    /// fixed_base is the structure of a chain welded to the world and fully driven, which is what
    /// the arm is: every coordinate driven, no base in the state, the base its brace in all six
    /// directions. `bodies` is the chain's bodies in chain order and `task_frame` the frame the
    /// plant presents for control.
    pub fn fixed_base(dof: usize, bodies: Vec<String>, task_frame: String) -> PlantStructure {
        PlantStructure {
            dof,
            actuated: vec![true; dof],
            base_dof: 0,
            contacts: Vec::new(),
            bodies,
            task_frame,
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
}

/// The narrow contract the controllers consume, so swapping the dynamics backend never touches the
/// control law. Structure first (`structure`), then the numeric core in the generalized coordinates
/// it declares (positions, velocities, mass matrix, bias and gravity), then the mapping a task is
/// written in — task frames and bodies, both addressed BY NAME.
pub trait Plant {
    /// structure is the machine as data: DOF, actuation, base, contacts, bodies and the task frame.
    /// It takes `&self` and not `&mut self` because a declaration is not a state change — a caller
    /// may read it before it has a mutable borrow, and a controller reads it once.
    fn structure(&self) -> PlantStructure;

    fn joint_positions(&mut self) -> Vec<f64>;
    fn joint_velocities(&mut self) -> Vec<f64>;
    fn mass_matrix(&mut self) -> Mat;
    fn bias_torques(&mut self) -> Vec<f64>;
    fn gravity_torques(&mut self) -> Vec<f64>;

    /// frame_pose / frame_jacobian / frame_full_jacobian are the task mapping, addressed BY NAME:
    /// `structure().task_frame` or any of `structure().bodies`. The pose is (position, wxyz
    /// quaternion), the Jacobians are 3 x dof (linear) and 6 x dof ([linear; angular]).
    fn frame_pose(&mut self, name: &str) -> (Vec3, Quat);
    fn frame_jacobian(&mut self, name: &str) -> Mat;
    fn frame_full_jacobian(&mut self, name: &str) -> Mat;

    /// body_frame / body_jacobian are the whole-body enumeration, also addressed by name: the
    /// (position, rotation) of one body's frame, and its 3 x dof Jacobian. A whole-body layer
    /// iterates `structure().bodies` and asks for each one, so the order is the plant's declared
    /// one rather than a convention it kept to itself.
    fn body_frame(&mut self, name: &str) -> (Vec3, Mat);
    fn body_jacobian(&mut self, name: &str) -> Mat;

    /// task_pose / task_jacobian / task_full_jacobian read the frame the plant DECLARED as its task
    /// (`structure().task_frame`) rather than one the interface assumes. They are what `body_pose` /
    /// `compute_jacobian` / `compute_full_jacobian` silently were; a caller that wants a different
    /// frame names it and calls `frame_*` directly.
    fn task_pose(&mut self) -> (Vec3, Quat) {
        let f = self.structure().task_frame;
        self.frame_pose(&f)
    }

    fn task_jacobian(&mut self) -> Mat {
        let f = self.structure().task_frame;
        self.frame_jacobian(&f)
    }

    fn task_full_jacobian(&mut self) -> Mat {
        let f = self.structure().task_frame;
        self.frame_full_jacobian(&f)
    }
}
