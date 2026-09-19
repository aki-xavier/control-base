// charter.rs — the crate's own boundary, as a check rather than a promise. A base crate that grows a
// dependency has stopped being a base, and the way that happens is never a decision: it is one
// convenient `use`. Three claims, all read off the sources as text:
//
//   1. every module's imports are `control-math` or this crate's own;
//   2. no source names an engine, a model, or a control law in CODE; the prose holds to the same
//      rule, and the four lists below are the only place these words are written down, because a
//      check has to name what it forbids;
//   3. the dependency set in Cargo.toml is the one the charter states.

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

// code_of drops whole-line comments: the check is about what the sources do, and the comments are
// prose about it rather than code.
fn code_of(text: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for line in text.lines() {
        if !line.trim().starts_with("//") {
            out.push(line);
        }
    }
    out.join("\n")
}

/// The base imports the arithmetic crate and itself, and nothing else: a third name here is the
/// layer this crate exists not to be.
#[test]
fn every_import_is_control_math_or_our_own() {
    let files = sources();
    assert_eq!(
        files.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
        vec!["contact_injection.rs", "efference.rs", "lib.rs", "plant.rs"],
        "the module list moved: the base is the contract, the efference copy and the contact model \
         the plants share, and nothing else"
    );
    for (name, text) in &files {
        for line in code_of(text).lines() {
            let t = line.trim();
            if !t.starts_with("use ") {
                continue;
            }
            assert!(
                t.starts_with("use control_math::")
                    || t.starts_with("use crate::")
                    || t.starts_with("use self::")
                    || t.starts_with("use super::"),
                "{name} imports outside the base: {t}"
            );
        }
    }
}

/// No engine, no model, no control law. The lists are the names that would mean a boundary had been
/// crossed: an engine's own vocabulary, a model layer's types, concrete implementors of this crate's
/// own contract, and the laws that sit above it.
#[test]
fn nothing_here_names_an_engine_a_model_or_a_law() {
    let forbidden: [(&str, &[&str]); 4] = [
        // the C ABI's own vocabulary, the SDK's prefixes, and the shim
        (
            "an engine",
            &[
                "extern \"C\"",
                "eng_",
                "eng_shim",
                "mj_",
                "mujoco",
                "MuJoCo",
            ],
        ),
        // the model layer's types
        (
            "a model type",
            &[
                "BodyTree",
                "TreeDynamicsModel",
                "PgaFk",
                "PgaDynamicsModel",
                "UrdfChain",
                "MjcfModel",
            ],
        ),
        // the concrete plants: a contract may not name its own implementors
        (
            "a concrete plant",
            &[
                "CEnginePlant",
                "BipedPlant",
                "NominalView",
                "FakeEnginePlant",
            ],
        ),
        // the laws and layer programs above this crate, which it sits below
        (
            "a control law",
            &[
                "PlaneTaskLoop",
                "PlaneDesign",
                "StandingLoop",
                "GaitRuntime",
                "GaitPlan",
                "SpinalReflex",
                "ReflexLayer",
                "BehaviorArbiter",
                "Predictor",
                "Keepout",
                "Recruit",
                "Adapt",
                "Cpg",
                "Stepper",
            ],
        ),
    ];
    for (name, text) in &sources() {
        let code = code_of(text);
        for (what, names) in forbidden {
            for bad in names {
                assert!(
                    !code.contains(bad),
                    "{name} names {what} ({bad}) in code: this crate holds interfaces and value \
                     types only"
                );
            }
        }
    }
    // and the lists are anchored: the contract and the shared arithmetic really are stated here, so
    // the checks above are not passing by looking at files that hold nothing
    let src = sources();
    let plant = src
        .iter()
        .find(|(n, _)| n == "plant.rs")
        .expect("src/plant.rs is gone: the Plant contract is not stated in this crate");
    assert!(
        plant.1.contains("pub trait Plant") && plant.1.contains("fn compute_full_jacobian"),
        "src/plant.rs no longer states the Plant contract"
    );
    let ci = src
        .iter()
        .find(|(n, _)| n == "contact_injection.rs")
        .expect("src/contact_injection.rs is gone: the shared contact model is not stated here");
    assert!(
        ci.1.contains("pub struct ContactInjection") && ci.1.contains("pub fn smooth_into"),
        "src/contact_injection.rs no longer states the shared contact model"
    );
}

/// The dependency set is the charter's own: one path below, and no build script. A base that needs
/// an engine at build time is not a base.
#[test]
fn the_dependency_set_is_exactly_control_math() {
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
    assert_eq!(
        deps.len(),
        1,
        "the base has more than one dependency: {deps:?}"
    );
    assert!(
        deps[0].starts_with("control-math"),
        "the base's one dependency is not control-math: {}",
        deps[0]
    );
    assert!(
        !manifest.contains("build =") && !root().join("build.rs").exists(),
        "the base has a build script: it would need something to build against"
    );
}
