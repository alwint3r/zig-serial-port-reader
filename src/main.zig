const std = @import("std");
const native_os = @import("builtin").os.tag;

fn configure_serial_port(fd: std.posix.fd_t, baud: std.posix.speed_t) !void {
    var tty = try std.posix.tcgetattr(fd);
    tty.ispeed = baud;
    tty.ospeed = baud;

    tty.cflag = .{
        .CSIZE = .CS8,
        .CREAD = true,
        .CLOCAL = true,
        .PARENB = false,
        .CSTOPB = false,
    };

    if (native_os == .macos) {
        tty.cflag.CCTS_OFLOW = false;
        tty.cflag.CRTS_IFLOW = false;
    }

    tty.lflag = .{
        .ICANON = false,
        .ECHO = false,
        .ECHOE = false,
        .ECHONL = false,
        .ISIG = false,
    };

    tty.iflag = .{
        .IXOFF = false,
        .IXON = false,
        .IXANY = false,
        .IGNBRK = false,
        .BRKINT = false,
        .PARMRK = false,
        .INLCR = false,
        .IGNCR = false,
        .ICRNL = false,
    };

    tty.oflag = .{
        .OPOST = false,
        .ONLCR = false,
    };

    tty.cc[@as(usize, @intFromEnum(std.posix.V.MIN))] = 0;
    tty.cc[@as(usize, @intFromEnum(std.posix.V.TIME))] = 10;

    try std.posix.tcsetattr(fd, .NOW, tty);
}

pub fn main() !void {
    const heap_allocator = std.heap.page_allocator;
    var arena = std.heap.ArenaAllocator.init(heap_allocator);
    defer arena.deinit();

    var args_iter = try std.process.argsWithAllocator(arena.allocator());
    if (args_iter.skip() == false) {
        std.log.err("No arguments provided", .{});
        return error.NoArgumentsProvided;
    }

    const port_name = args_iter.next() orelse {
        std.log.err("No serial port is provided", .{});
        return error.NoPortProvided;
    };

    const baud_rate = args_iter.next() orelse {
        std.log.err("No baud rate is provided", .{});
        return error.NoBaudRateProvided;
    };

    std.log.info("Opening port {s} with baud rate {s}", .{ port_name, baud_rate });
    const baud: std.posix.speed_t = @enumFromInt(try std.fmt.parseInt(u64, baud_rate, 10));
    std.log.info("Baud rate: {any}", .{baud});

    const flag: std.posix.O = .{
        .ACCMODE = .RDWR,
        .NONBLOCK = true,
        .NOCTTY = true,
    };
    const fd = try std.posix.open(
        port_name,
        flag,
        0,
    );
    defer std.posix.close(fd);

    std.log.info("Port opened successfully. FD({})", .{fd});
    try configure_serial_port(fd, baud);

    var buf = try arena.allocator().alloc(u8, 1024);
    defer arena.allocator().free(buf);
    while (true) {
        const bytes_read = try std.posix.read(fd, buf);
        if (bytes_read == 0) {
            continue;
        }
        try std.io.getStdOut().writer().writeAll(buf[0..bytes_read]);
    }
}
