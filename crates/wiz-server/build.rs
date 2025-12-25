fn main() {
    // Build the C++ bridge
    cxx_build::bridge("src/ffi.rs")
        .file("cxx/bridge.cpp")
        .std("c++17")
        .flag_if_supported("-Wno-unused-parameter")
        .compile("wiz_ros2_bridge");

    // Tell cargo to rerun if these files change
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=cxx/bridge.hpp");
    println!("cargo:rerun-if-changed=cxx/bridge.cpp");

    // When ros2 feature is enabled, link against ROS2 libraries
    #[cfg(feature = "ros2")]
    {
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

            // Add include paths
            println!("cargo:rustc-env=CXXFLAGS=-I{ros_root}/include");

            // Define ROS2 enabled macro
            println!("cargo:rustc-cfg=ros2");
        } else {
            println!(
                "cargo:warning=ROS_DISTRO not set, building without ROS2 support. \
                Source ROS2 setup.bash and rebuild with --features ros2"
            );
        }
    }
}
