const std = @import("std");
const native_os = @import("builtin").os.tag;
// re-implement this here since the std.posix.T.ior is not marked as pub
fn ior(inout: u32, group: usize, num: usize, len: usize) usize {
    return (inout | ((len & 0x1fff) << 16) | ((group) << 8) | (num));
}

const SerialPort = struct {
    path: []const u8,
    baud: std.posix.speed_t,
    fd: ?std.posix.fd_t,
    tty: ?std.posix.termios,

    pub fn init(path: []const u8, baud: std.posix.speed_t) !SerialPort {
        var port = SerialPort{
            .path = path,
            .baud = baud,
            .fd = null,
            .tty = null,
        };
        try port.open(path);
        try port.configure();
        return port;
    }

    pub fn open(self: *SerialPort, path: []const u8) !void {
        const flag: std.posix.O = .{
            .ACCMODE = .RDWR,
            .NONBLOCK = true,
            .NOCTTY = true,
        };

        self.fd = try std.posix.open(path, flag, 0);
    }

    pub fn reset(self: *SerialPort) !void {
        const fd = self.fd orelse {
            return error.InvalidFileDescriptor;
        };
        switch (native_os) {
            .macos => {
                var status: c_int = 0;
                const IOCMGET = ior(
                    0x40000000,
                    't',
                    106,
                    @sizeOf(c_int),
                );
                const IOCMSET = ior(
                    0x80000000,
                    't',
                    109,
                    @sizeOf(c_int),
                );

                var ret: c_int = undefined;

                ret = std.c.ioctl(fd, @intCast(IOCMGET), &status);
                if (ret == -1) {
                    return error.FailedToGetIOCM;
                }

                status &= ~@as(c_int, 2);
                // This won't work on Debug or ReleaseSafe build as the IOCMSET
                // will be truncated and the ioctl will trigger panic.
                // Welcome to the dark side (C).
                ret = std.c.ioctl(fd, @intCast(IOCMSET), &status);
                if (ret == -1) {
                    return error.FailedToSetIOCM;
                }

                std.time.sleep(100_000_000);
                status |= @as(c_int, 2);
                ret = std.c.ioctl(fd, @intCast(IOCMSET), &status);
                if (ret == -1) {
                    return error.FailedToSetIOCM;
                }
            },
            else => unreachable,
        }
    }

    fn configure(self: *SerialPort) !void {
        const fd = self.fd orelse @panic("Invalid file descriptor");
        var tty = try std.posix.tcgetattr(fd);
        self.tty = tty;
        tty.ispeed = self.baud;
        tty.ospeed = self.baud;

        tty.cflag = .{
            .CSIZE = .CS8,
            .CREAD = true,
            .CLOCAL = true,
            .PARENB = false,
            .CSTOPB = false,
            .HUPCL = true,
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

    pub fn deinit(self: *SerialPort) void {
        if (self.fd) |fd| {
            _ = std.posix.close(fd);
        }
    }
};

fn open_serial_port(port_name: []const u8) !std.posix.fd_t {
    const flag: std.posix.O = .{
        .ACCMODE = .RDWR,
        .NONBLOCK = true,
        .NOCTTY = true,
    };
    return std.posix.open(port_name, flag, 0);
}

const Mode = enum {
    Reset,
    Read,
};

pub fn get_mode_from(mode: []const u8) !Mode {
    if (std.mem.eql(u8, mode, "reset")) {
        return .Reset;
    } else if (std.mem.eql(u8, mode, "read")) {
        return .Read;
    } else {
        return error.InvalidModeProvided;
    }
}

pub fn main() !void {
    const heap_allocator = std.heap.page_allocator;
    var arena = std.heap.ArenaAllocator.init(heap_allocator);
    defer arena.deinit();

    var args_iter = try std.process.argsWithAllocator(arena.allocator());
    defer args_iter.deinit();
    _ = args_iter.skip();

    if (args_iter.inner.count != 4) {
        usage();
        return error.InvalidArgumentsProvided;
    }

    const mode_str = args_iter.next() orelse {
        usage();
        return error.NoModeProvided;
    };

    const port_name = args_iter.next() orelse {
        usage();
        return error.NoPortProvided;
    };

    const baud_rate = args_iter.next() orelse {
        usage();
        return error.NoBaudRateProvided;
    };

    std.log.info("Opening port {s} with baud rate {s}", .{ port_name, baud_rate });
    const baud: std.posix.speed_t = @enumFromInt(try std.fmt.parseInt(u64, baud_rate, 10));
    std.log.info("Baud rate: {any}", .{baud});

    var port = try SerialPort.init(port_name, baud);
    defer port.deinit();

    std.log.info("Port opened successfully. FD({any})", .{port.fd});

    const mode = try get_mode_from(mode_str);
    switch (mode) {
        .Read => {
            var buf = try arena.allocator().alloc(u8, 1024);
            defer arena.allocator().free(buf);
            const fd = port.fd orelse {
                return error.InvalidFileDescriptor;
            };
            while (true) {
                const bytes_read = try std.posix.read(fd, buf);
                if (bytes_read == 0) {
                    continue;
                }
                try std.io.getStdOut().writer().writeAll(buf[0..bytes_read]);
            }
        },

        .Reset => {
            try port.reset();
            std.log.info("Port reset successfully.", .{});
        },
    }
}

fn usage() void {
    std.debug.print("Usage: serial_reader <mode> <port> <baud_rate>\n", .{});
}
