fn main() {
    let version = format!("Crampon {}", env!("CARGO_PKG_VERSION"));
    anchor_codegen::ConfigBuilder::new()
        .entry("src/main.rs")
        .set_build_versions("")
        .set_version(version)
        .build();
}
