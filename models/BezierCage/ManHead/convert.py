import sys

def parse_line(line):
    """解析一行，返回点列表（每个点为三元组）和类型"""
    nums = list(map(float, line.strip().split()))
    if len(nums) % 3 != 0:
        raise ValueError("点数不是3的倍数")
    points = [(nums[i], nums[i+1], nums[i+2]) for i in range(0, len(nums), 3)]
    if len(points) == 16:
        return points, "tensor"
    elif len(points) == 10:
        return points, "triangle"
    else:
        raise ValueError(f"未知控制点数量: {len(points)}，应为16(张量积)或10(三角形)")

def extract_tensor_vertices(points):
    """从16个控制点中提取四个角点，顺序为A,B,C,D（行优先网格）"""
    A = points[0]   # (0,0)
    B = points[3]   # (0,3)
    C = points[15]  # (3,3)
    D = points[12]  # (3,0)
    return A, B, C, D

def extract_triangle_vertices(points):
    """从10个控制点中提取三个顶点，索引0,6,9"""
    A = points[0]
    B = points[6]
    C = points[9]
    return A, B, C

def generate_tensor_controls(n, A, B, C, D):
    """
    生成n阶张量积贝塞尔曲面的控制点（(n+1)x(n+1)个点）
    输出顺序：行优先，即第一行从左到右，第二行从左到右...
    """
    controls = []
    for i in range(n+1):
        u = i / n
        for j in range(n+1):
            v = j / n
            # 双线性插值
            x = (1-u)*(1-v)*A[0] + u*(1-v)*B[0] + u*v*C[0] + (1-u)*v*D[0]
            y = (1-u)*(1-v)*A[1] + u*(1-v)*B[1] + u*v*C[1] + (1-u)*v*D[1]
            z = (1-u)*(1-v)*A[2] + u*(1-v)*B[2] + u*v*C[2] + (1-u)*v*D[2]
            controls.append((x, y, z))
    return controls

def generate_triangle_controls(n, A, B, C):
    """
    生成n阶三角形贝塞尔曲面的控制点，共 (n+1)(n+2)/2 个点
    输出顺序与原始3阶完全一致：
        按 i 从 n 递减到 0，对于每个 i，j 从 (n-i) 递减到 0，
        其中控制点坐标为 (i/n)*A + (j/n)*B + ((n-i-j)/n)*C
    """
    controls = []
    for i in range(n, -1, -1):
        # 对于固定的 i，j 从 (n-i) 递减到 0
        for j in range(n - i, -1, -1):
            k = n - i - j
            alpha = i / n
            beta = j / n
            gamma = k / n
            x = alpha * A[0] + beta * B[0] + gamma * C[0]
            y = alpha * A[1] + beta * B[1] + gamma * C[1]
            z = alpha * A[2] + beta * B[2] + gamma * C[2]
            controls.append((x, y, z))
    return controls

def format_controls(controls):
    """将控制点列表格式化为一行字符串，空格分隔"""
    parts = []
    for p in controls:
        parts.extend([f"{p[0]:.15f}", f"{p[1]:.15f}", f"{p[2]:.15f}"])
    return " ".join(parts)

def process_file(input_file, output_2_file, output_4_file):
    with open(input_file, 'r') as fin, \
         open(output_2_file, 'w') as fout2, \
         open(output_4_file, 'w') as fout4:
        for line_num, line in enumerate(fin, 1):
            line = line.strip()
            if not line:
                continue
            try:
                points, typ = parse_line(line)
            except Exception as e:
                print(f"行{line_num}解析错误: {e}")
                continue

            if typ == "tensor":
                A, B, C, D = extract_tensor_vertices(points)
                ctrl2 = generate_tensor_controls(2, A, B, C, D)  # 3x3 = 9个点
                ctrl4 = generate_tensor_controls(4, A, B, C, D)  # 5x5 = 25个点
            else:  # triangle
                A, B, C = extract_triangle_vertices(points)
                ctrl2 = generate_triangle_controls(2, A, B, C)   # 6个点
                ctrl4 = generate_triangle_controls(4, A, B, C)   # 15个点

            fout2.write(format_controls(ctrl2) + "\n")
            fout4.write(format_controls(ctrl4) + "\n")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python covert.py 输入文件.txt")
        sys.exit(1)
    input_filename = sys.argv[1]
    process_file(input_filename, "output_2.txt", "output_4.txt")
    print("完成，结果已写入 output_2.txt 和 output_4.txt")