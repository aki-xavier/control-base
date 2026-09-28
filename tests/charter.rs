use std::fs;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn sources() -> Vec<(String, String)> {
    fn walk(dir: &Path, prefix: &str, out: &mut Vec<(String, String)>) {
        let entries = fs::read_dir(dir).unwrap_or_else(|_| panic!("cannot read {}", dir.display()));
        let mut paths: Vec<PathBuf> = entries.filter_map(|e| e.ok()).map(|e| e.path()).collect();
        paths.sort();
        for p in paths {
            let name = p.file_name().unwrap().to_string_lossy().to_string();
            if p.is_dir() {
                walk(&p, &format!("{prefix}{name}/"), out);
            } else if name.ends_with(".rs") {
                let text =
                    fs::read_to_string(&p).unwrap_or_else(|_| panic!("cannot read {prefix}{name}"));
                out.push((format!("{prefix}{name}"), text));
            }
        }
    }
    let mut out = Vec::new();
    walk(&root().join("src"), "", &mut out);
    out
}

fn code_of(text: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for line in text.lines() {
        if !line.trim().starts_with("//") {
            out.push(line);
        }
    }
    out.join("\n")
}

#[test]
fn every_import_is_control_math_pga_or_our_own() {
    let files = sources();
    assert!(
        files.len() >= 4,
        "the source walk found {} files: a walk that stopped early would make the claim below pass \
         by looking at nothing",
        files.len()
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
