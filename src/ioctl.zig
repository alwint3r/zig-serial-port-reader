const std = @import("std");

pub const IOC_IN = 0x80000000;
pub const IOC_OUT = 0x40000000;

/// re-implement this here since the std.posix.T.ior is not marked as pub
pub fn ior(inout: u32, group: usize, num: usize, len: usize) usize {
    return (inout | ((len & std.c.IOCPARM_MASK) << 16) | ((group) << 8) | (num));
}

pub const TIOCM_DTR = @as(c_int, 2);

pub const TIOCM = struct {
    pub const GET = ior(
        IOC_OUT,
        't',
        106,
        @sizeOf(c_int),
    );

    pub const SET = ior(
        IOC_IN,
        't',
        109,
        @sizeOf(c_int),
    );
};

pub fn ioctl(fd: std.posix.fd_t, request: usize, out: *c_int) c_int {
    @setRuntimeSafety(false);
    const ret = std.c.ioctl(fd, @intCast(request), &out);
    return ret;
}
