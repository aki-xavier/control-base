// charter.rs — the crate's own boundary, as a check rather than a promise. A base crate that grows a
// dependency has stopped being a base, and the way that happens is never a decision: it is one
// convenient `use`. Two claims, both read off the sources as text:
//
//   1. every module's imports are `control-math`, `pga` or this crate's own;
//   2. the dependency set in Cargo.toml is the one the charter states.
//
// `pga` is named and not smuggled: it is the zero-dependency crate of the SHARED VALUE TYPE the
// contract hands its poses out in, so it is exactly what the charter's own rule about shared value
// types asks for — one held by more than one party cannot live under any of them. Anything that would
// need the model, a plant or an engine stays out.
//
// The interfaces are anchored separately (`the_interfaces_really_are_stated_here`), so the claims
// above cannot pass by looking at files that hold nothing.

use std::fs;
use std::path::PathBuf;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

// sources collects every `.rs` under src/ as (relative path, text): a walk that missed a
// subdirectory would leave the charter's escape hatch open.
fn sources() -> Vec<(String, String)> {
    let mut out = Vec::new();
    let dir = root().join("src");
    let entries = fs::read_dir(&dir).unwrap_or_else(|_| panic!("cannot read {}", dir.display()));
    let mut paths: Vec<PathBuf> = entries.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    paths.sort();
    for p in paths {
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        if name.ends_with(".rs") {
            let text = fs::read_to_string(&p).unwrap_or_else(|_| panic!("cannot read {name}"));
            out.push((name, text));
        }
    }
    out
}

// code_of drops whole-line comments: claim 1 is about what the sources do, and the comments are prose
// about it rather than code.
fn code_of(text: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for line in text.lines() {
        if !line.trim().starts_with("//") {
            out.push(line);
        }
    }
    out.join("\n")
}

/// The base imports the arithmetic crate, the algebra of the value type it hands its poses out in, and
/// itself, and nothing else: a fourth name here is the thing this crate exists not to be.
#[test]
fn every_import_is_control_math_pga_or_our_own() {
    let files = sources();
    assert_eq!(
        files.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
        vec![
            "adapt.rs",
            "contact_injection.rs",
            "contact_law.rs",
            "efference.rs",
            "lib.rs",
            "plant.rs"
        ],
        "the module list moved: the base is the contract, the efference copy, the contact model \
         (what a force does and where it comes from) and the calibration, and nothing else"
    );
    for (name, text) in &files {
        for line in code_of(text).lines() {
            let t = line.trim();
            if !t.starts_with("use ") {
                continue;
            }
            assert!(
                t.starts_with("use control_math::")
                    || t.starts_with("use pga::")
                    || t.starts_with("use crate::")
                    || t.starts_with("use self::")
                    || t.starts_with("use super::"),
                "{name} imports outside the base: {t}"
            );
        }
    }
}

/// The interfaces really are stated here, so the claims above are not passing by looking at files
/// that hold nothing: the contract, the shared contact model, the laws a contact force comes from,
/// and the calibration.
#[test]
fn the_interfaces_really_are_stated_here() {
    let src = sources();
    let plant = src
        .iter()
        .find(|(n, _)| n == "plant.rs")
        .expect("src/plant.rs is gone: the Plant contract is not stated in this crate");
    assert!(
        plant.1.contains("pub trait Plant")
            && plant.1.contains("fn structure")
            && plant.1.contains("fn frame_full_jacobian")
            && plant.1.contains("fn frame_motor")
            && plant.1.contains("fn task_motor")
            && plant.1.contains("pub fn motor_of_pose"),
        "src/plant.rs no longer states the Plant contract and the motor its poses are handed out in"
    );
    let ci = src
        .iter()
        .find(|(n, _)| n == "contact_injection.rs")
        .expect("src/contact_injection.rs is gone: the shared contact model is not stated here");
    assert!(
        ci.1.contains("pub struct ContactInjection") && ci.1.contains("pub fn smooth_into"),
        "src/contact_injection.rs no longer states the shared contact model"
    );
    let cl = src.iter().find(|(n, _)| n == "contact_law.rs").expect(
        "src/contact_law.rs is gone: the laws a plant's contact force comes from are not here",
    );
    assert!(
        cl.1.contains("pub trait ContactLaw")
            && cl.1.contains("pub struct ReportContact")
            && cl.1.contains("pub struct SolveContact"),
        "src/contact_law.rs no longer states the contact laws"
    );
    let ada = src
        .iter()
        .find(|(n, _)| n == "adapt.rs")
        .expect("src/adapt.rs is gone: the shared calibration is not stated here");
    assert!(
        ada.1.contains("pub struct Adapt") && ada.1.contains("AdaptParam"),
        "src/adapt.rs no longer states the shared calibration"
    );
}

/// The dependency set is the charter's own: the arithmetic, the one shared value type, and no build
/// script. A base that needs an engine at build time is not a base.
#[test]
fn the_dependency_set_is_exactly_the_charters() {
    let manifest = fs::read_to_string(root().join("Cargo.toml")).expect("Cargo.toml");
    let mut in_deps = false;
    let mut deps: Vec<String> = Vec::new();
    for line in manifest.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_deps = t == "[dependencies]";
            continue;
        }
        if in_deps && !t.is_empty() && !t.starts_with('#') {
            deps.push(t.to_string());
        }
    }
    // sorted names, so a third entry fails on the set and not on its order
    let mut names: Vec<&str> = deps
        .iter()
        .map(|d| d.split_whitespace().next().unwrap_or(""))
        .collect();
    names.sort_unstable();
    assert_eq!(
        names,
        vec!["control-math", "pga"],
        "the base's dependency set moved: {deps:?}"
    );
    assert!(
        !manifest.contains("build =") && !root().join("build.rs").exists(),
        "the base has a build script: it would need something to build against"
    );
}
