fn main() {
    // Check if ROS2 feature is enabled and ROS_DISTRO is set
    // Note: In build scripts, we must check CARGO_FEATURE_* env vars instead of cfg!()
    let ros2_feature = std::env::var("CARGO_FEATURE_ROS2").is_ok();
    let ros2_enabled = ros2_feature && std::env::var("ROS_DISTRO").is_ok();

    // Build the C++ bridge
    let mut build = cxx_build::bridge("src/ffi.rs");
    build
        .file("cxx/bridge.cpp")
        .std("c++17")
        .flag_if_supported("-Wno-unused-parameter");

    // When ROS2 is enabled, add the macro and include paths
    if ros2_enabled {
        let ros_distro = std::env::var("ROS_DISTRO").unwrap();
        let ros_root = format!("/opt/ros/{ros_distro}");
        let include_root = format!("{ros_root}/include");

        // Define WIZ_ROS2_ENABLED for C++ code
        build.define("WIZ_ROS2_ENABLED", None);

        // Add all ROS2 include subdirectories
        // ROS2 Humble+ puts each package in its own subdirectory
        if let Ok(entries) = std::fs::read_dir(&include_root) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    build.include(&path);
                }
            }
        }

        // Also add the root include path
        build.include(&include_root);
    }

    build.compile("wiz_ros2_bridge");

    // Explicitly add link search path and library
    let out_dir = std::env::var("OUT_DIR").unwrap();
    println!("cargo:rustc-link-search=native={out_dir}");
    println!("cargo:rustc-link-lib=static=wiz_ros2_bridge");

    // Tell cargo to rerun if these files change
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=cxx/bridge.hpp");
    println!("cargo:rerun-if-changed=cxx/bridge.cpp");

    // When ros2 feature is enabled, link against ROS2 libraries
    if ros2_feature {
        // Find ROS2 installation
        if let Ok(ros_distro) = std::env::var("ROS_DISTRO") {
            let ros_root = format!("/opt/ros/{ros_distro}");

            println!("cargo:rustc-link-search=native={ros_root}/lib");

            // Core ROS2 libraries
            println!("cargo:rustc-link-lib=rclcpp");
            println!("cargo:rustc-link-lib=rcl");
            println!("cargo:rustc-link-lib=rmw");
            println!("cargo:rustc-link-lib=rcutils");

            // Message libraries
            println!("cargo:rustc-link-lib=sensor_msgs__rosidl_typesupport_cpp");
            println!("cargo:rustc-link-lib=geometry_msgs__rosidl_typesupport_cpp");
            println!("cargo:rustc-link-lib=nav_msgs__rosidl_typesupport_cpp");
            println!("cargo:rustc-link-lib=tf2_msgs__rosidl_typesupport_cpp");

            // TF2
            println!("cargo:rustc-link-lib=tf2_ros");
            println!("cargo:rustc-link-lib=tf2");

            // Define ROS2 enabled macro for Rust
            println!("cargo:rustc-cfg=ros2");
        } else {
            println!(
                "cargo:warning=ROS_DISTRO not set, building without ROS2 support. \
                Source ROS2 setup.bash and rebuild with --features ros2"
            );
        }
    }
}
