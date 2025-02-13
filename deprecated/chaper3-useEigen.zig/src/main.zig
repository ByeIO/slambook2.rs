const std = @import("std");
const eigen = @import("deps/eigen.zig"); // 引入eigen包
pub fn main() !void {
    // 打印到标准错误（基于 `std.io.getStdErr()` 的快捷方式）
    std.debug.print("所有你的 {s} 都属于我们。\n", .{"代码库"});
    // 标准输出用于应用程序的实际输出，例如如果你正在实现gzip，那么只有压缩后的字节应该发送到标准输出，而不是任何调试消息。
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();
    try stdout.print("运行 `zig build test` 来运行测试。\n", .{});
    try bw.flush(); // 不要忘记刷新！
}
test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // 尝试注释掉这行，看看zig是否能检测到内存泄漏！
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}

// 测试eigen库的创建矩阵功能
test "eigen-create-matrix" {
    // 创建一个初始化为 0 的 2x3 矩阵
    const zero = try eigen.Matrix.init(2, 3, std.testing.allocator);
    eigen.Matrix.print_array(zero.data); // 使用print_array方法打印矩阵
    // 或者如果你想要一个 3x3 的正方形矩阵
    const zero_square = try eigen.Matrix.init_square(3, std.testing.allocator);
    eigen.Matrix.print_array(zero_square.data); // 使用print_array方法打印矩阵
    // 从一个数组创建一个矩阵
    const m1 = try eigen.Matrix.from_array([2][2]f64{
        .{ 1, 2 },
        .{ 3, 4 },
    }, std.testing.allocator);
    eigen.Matrix.print_array(m1.data); // 使用print_array方法打印矩阵
}
