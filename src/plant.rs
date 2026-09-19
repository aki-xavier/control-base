// plant.rs — the Plant contract a controller programs against. It is stated here, and not beside an
// implementation, because the contract's whole point is that the dynamics backend can be swapped
// without the control law changing: a contract that lives with an engine is one an engine can reach
// into. Its implementors are simu's plant/c_engine.rs (the C ABI binding, and the only file of that
// crate allowed to be one), the model-based views its benches serve dynamics from, and the fake its
// SimWorld tests carry.

use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;

/// The narrow contract the controllers consume, so swapping the dynamics backend never touches the
/// control law: joint_positions / joint_velocities / mass_matrix / bias_torques / gravity_torques,
/// compute_jacobian (3 x n), compute_full_jacobian (6 x n, [linear; angular]), body_pose (position +
/// wxyz quaternion) and the per-link frames the avoidance layer reads; all take `&mut self`.
pub trait Plant {
    fn joint_positions(&mut self) -> Vec<f64>;
    fn joint_velocities(&mut self) -> Vec<f64>;
    fn mass_matrix(&mut self) -> Mat;
    fn bias_torques(&mut self) -> Vec<f64>;
    fn gravity_torques(&mut self) -> Vec<f64>;
    fn compute_jacobian(&mut self) -> Mat;
    fn compute_full_jacobian(&mut self) -> Mat;
    fn body_pose(&mut self) -> (Vec3, Quat);
    /// per-link body frames (chain order) for the whole-arm avoidance layer
    fn link_frame(&mut self, i: usize) -> (Vec3, Mat);
    fn link_jacobian(&mut self, i: usize) -> Mat;
}
