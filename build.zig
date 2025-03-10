const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const zig_module = b.addModule("main_zig", .{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    const main_exe = b.addExecutable(.{
        .name = "main",
        .root_module = zig_module,
    });

    const main_module = b.addModule("main", .{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    main_module.addCSourceFiles(.{
        .files = &[_][]const u8{
            "src/main.c",
        },
        .flags = &[_][]const u8{
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
        },
    });

    const main_c_exe = b.addExecutable(.{
        .name = "main_c",
        .root_module = main_module,
    });

    const port_reseter = b.addModule("port_reseter", .{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });

    port_reseter.addCSourceFiles(.{
        .files = &[_][]const u8{
            "src/port_reseter/darwin_reset_port_posix.c",
        },
        .flags = &[_][]const u8{
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
        },
    });

    const port_reseter_lib = b.addStaticLibrary(.{
        .name = "portreseter",
        .root_module = port_reseter,
    });

    zig_module.linkLibrary(port_reseter_lib);
    b.installArtifact(main_exe);
    b.installArtifact(port_reseter_lib);
    b.installArtifact(main_c_exe);

    const c_exe_cmd = b.addRunArtifact(main_c_exe);
    c_exe_cmd.step.dependOn(b.getInstallStep());

    const run_c_step = b.step("run_c", "Run this application");
    run_c_step.dependOn(&c_exe_cmd.step);

    const run_cmd = b.addRunArtifact(main_exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run this application");
    run_step.dependOn(&run_cmd.step);

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }
}
