use control_base::plant::{
    motor_of_pose, motor_of_rotor, quat_of_rotor, rotor_of_quat, rotvec_between, Plant,
    PlantStructure, TaskMap,
};
use control_math::mat::Mat;
use control_math::quat::Quat;
use control_math::vec3::Vec3;
use pga::Multivector;

fn tip_pos() -> Vec3 {
    Vec3::new(0.3, -0.2, 0.5)
}

fn tip_rot() -> Quat {
    let h = std::f64::consts::FRAC_1_SQRT_2;
    Quat {
        w: h,
        x: 0.0,
        y: 0.0,
        z: h,
    }
}

struct Probe {
    map: TaskMap,
}

fn probe(map: TaskMap) -> Probe {
    Probe { map }
}

impl Plant for Probe {
    fn structure(&self) -> PlantStructure {
        let points = match self.map {
            TaskMap::Point(_) => vec!["com".to_string()],
            TaskMap::Frame(_) => Vec::new(),
        };
        PlantStructure {
            dof: 1,
            actuated: vec![true],
            base_dof: 0,
            contacts: Vec::new(),
            bodies: vec!["l1".to_string()],
            points,
            task_map: self.map.clone(),
        }
    }

    fn joint_positions(&mut self) -> Vec<f64> {
        vec![0.0]
    }

    fn joint_velocities(&mut self) -> Vec<f64> {
        vec![0.0]
    }

    fn mass_matrix(&mut self) -> Mat {
        Mat::eye_scaled(1.0, 1)
    }

    fn bias_torques(&mut self) -> Vec<f64> {
        vec![0.0]
    }

    fn gravity_torques(&mut self) -> Vec<f64> {
        vec![0.0]
    }

    fn frame_motor(&mut self, name: &str) -> Multivector {
        assert_eq!(name, "tip", "this machine has one frame");
        motor_of_rotor(tip_pos(), rotor_of_quat(tip_rot()))
    }

    fn frame_position(&mut self, name: &str) -> Vec3 {
        assert_eq!(name, "tip", "this machine has one frame");
        tip_pos()
    }

    fn frame_jacobian(&mut self, _name: &str) -> Mat {
        Mat::zeros(3, 1)
    }

    fn frame_full_jacobian(&mut self, _name: &str) -> Mat {
        Mat::zeros(6, 1)
    }

    fn point_position(&mut self, name: &str) -> Vec3 {
        assert_eq!(name, "com", "this machine has one point");
        Vec3::new(0.1, 0.0, 0.2)
    }

    fn point_jacobian(&mut self, _name: &str) -> Mat {
        Mat::zeros(3, 1)
    }

    fn body_frame(&mut self, _name: &str) -> (Vec3, Mat) {
        (Vec3::ZERO, Mat::eye_scaled(1.0, 3))
    }

    fn body_jacobian(&mut self, _name: &str) -> Mat {
        Mat::zeros(3, 1)
    }
}

fn act(m: Multivector, x: Vec3) -> Vec3 {
    let c = m.gp(pga::point(x.x, x.y, x.z)).gp(m.reverse()).coords();
    Vec3::new(c[0], c[1], c[2])
}

fn close(a: Vec3, b: Vec3) -> bool {
    a.sub(b).norm() < 1e-12
}

#[test]
fn the_rotor_acts_as_the_contracts_quaternion_does() {
    let r = rotor_of_quat(tip_rot());
    assert!(
        (r.norm() - 1.0).abs() < 1e-12,
        "a rotor from a unit quaternion is unit: {}",
        r.norm()
    );
    for v in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.1, -0.7, 0.3),
    ] {
        let by_rotor = act(r, v);
        let by_matrix = tip_rot().to_mat3().mul_vec3(v);
        assert!(
            close(by_rotor, by_matrix),
            "the rotor sent {v:?} to {by_rotor:?} where the quaternion's own matrix says {by_matrix:?}"
        );
    }
}

#[test]
fn the_motor_rotates_then_translates() {
    let m = motor_of_pose(tip_pos(), tip_rot());
    assert!(
        close(act(m, Vec3::ZERO), tip_pos()),
        "the motor did not carry the origin to the pose's position"
    );
    let local = Vec3::new(1.0, 0.0, 0.0);
    let want = tip_pos().add(tip_rot().to_mat3().mul_vec3(local));
    assert!(
        close(act(m, local), want),
        "the motor took a local point to {:?} where T(p) R says {want:?}",
        act(m, local)
    );
    let other = rotor_of_quat(tip_rot()).gp(pga::translator(tip_pos().to_array()));
    assert!(
        !close(act(other, local), want),
        "R T(p) and T(p) R agree here: this check would not notice the swap"
    );
}

#[test]
fn the_task_motor_is_the_frames_own_motor_and_its_halves_read_out_exactly() {
    let mut p = probe(TaskMap::Frame("tip".to_string()));
    let want = motor_of_rotor(tip_pos(), rotor_of_quat(tip_rot()));
    assert_eq!(
        p.frame_motor("tip").to_matrix(),
        want.to_matrix(),
        "the frame's motor is not the pose this machine states"
    );
    assert_eq!(
        p.task_motor().to_matrix(),
        want.to_matrix(),
        "the task motor is not its frame's own motor"
    );
    assert!(
        close(p.task_position(), tip_pos()),
        "the task's position is {:?} where the machine states {:?}",
        p.task_position(),
        tip_pos()
    );
    assert_eq!(
        p.task_rotation().values,
        rotor_of_quat(tip_rot()).values,
        "the task's rotation is not the versor the frame's pose was built from, term for term"
    );
}

#[test]
fn a_point_tasks_motor_carries_the_convention_and_not_a_rotation() {
    let mut p = probe(TaskMap::Point("com".to_string()));
    assert!(p.structure().task_is_a_point());
    let m = p.task_motor();
    let want = motor_of_pose(p.point_position("com"), Quat::IDENTITY);
    assert_eq!(
        m.to_matrix(),
        want.to_matrix(),
        "a point task's motor is its position with no rotation, the convention both accessors carry"
    );
    let local = Vec3::new(1.0, 0.0, 0.0);
    assert!(close(act(m, local), p.point_position("com").add(local),));
}

#[test]
fn two_motors_compose_by_one_product_and_the_order_is_the_action() {
    let (p1, q1) = (
        Vec3::new(0.1, 0.0, 0.2),
        Quat {
            w: std::f64::consts::FRAC_1_SQRT_2,
            x: std::f64::consts::FRAC_1_SQRT_2,
            y: 0.0,
            z: 0.0,
        },
    );
    let (p2, q2) = (tip_pos(), tip_rot());
    let seq = motor_of_pose(p1, q1).gp(motor_of_pose(p2, q2));
    let x = Vec3::new(0.4, 0.3, -0.1);

    let by_seq = act(seq, x);
    let by_steps = act(motor_of_pose(p1, q1), act(motor_of_pose(p2, q2), x));
    assert!(
        close(by_seq, by_steps),
        "composing motors is not applying them in sequence: {by_seq:?} against {by_steps:?}"
    );
    let want = motor_of_pose(p1.add(q1.to_mat3().mul_vec3(p2)), q1.mul(q2));
    assert!(
        close(act(want, x), by_seq),
        "the product is not the motor of (p1 + R1 p2, R1 R2)"
    );
}

#[test]
fn the_quaternion_and_rotor_readings_are_exact_inverses() {
    let mut worst = 0.0f64;
    for (w, x, y, z) in [
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.5, -0.5, 0.5, -0.5),
        (0.6, 0.8, 0.0, 0.0),
        (2.0, -3.0, 0.5, 1.5),
    ] {
        let q = Quat { w, x, y, z };
        let back = quat_of_rotor(rotor_of_quat(q));
        worst = worst
            .max((back.w - q.w).abs())
            .max((back.x - q.x).abs())
            .max((back.y - q.y).abs())
            .max((back.z - q.z).abs());
    }
    assert!(
        worst == 0.0,
        "the rotor reading and the quaternion reading disagree by {worst:.3e}: one of the two has \
         moved, and every pose handed across the contract is converted by them"
    );
}

#[test]
fn the_rotor_rotvec_is_the_number_the_quaternion_reading_gives() {
    let axis = Vec3::new(0.31, -0.52, 0.79).normalized();
    let mut worst = 0.0f64;
    for (target_angle, current_angle) in [(0.2, -0.7), (1.3, 2.9), (-2.0, 0.4), (0.0, 0.0)] {
        let tq = Quat::from_mat3(&Mat::from_axis_angle(axis, target_angle));
        let cq = Quat::from_mat3(&Mat::from_axis_angle(
            Vec3::new(1.0, 0.0, 0.0),
            current_angle,
        ));
        let by_quat = Quat::rotvec_between(tq, cq);
        let by_rotor = rotvec_between(rotor_of_quat(tq), rotor_of_quat(cq));
        worst = worst
            .max((by_quat.x - by_rotor.x).abs())
            .max((by_quat.y - by_rotor.y).abs())
            .max((by_quat.z - by_rotor.z).abs());
    }
    assert!(
        worst == 0.0,
        "the versor reading and the quaternion reading differ by {worst:.3e}"
    );
}
