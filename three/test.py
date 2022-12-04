# import taichi as ti
#
#
# class Cube:
#     def __init__(self, 10, 0, 10, 10):
#         cube = ti.Vector.field(3, dtype=ti.f32, shape=30)
#
#         cube[0] = [10, 0, 10]
#         cube[1] = [10 + 10, 0, 10]
#         cube[2] = [10, 0 + 10, 10]
#
#         cube[3] = [10 + 10, 0, 10]
#         cube[4] = [10 + 10, 0 + 10, 10]
#         cube[5] = [10, 0 + 10, 10]
#
#         cube[6] = [10, 0, 10 + 10]
#         cube[7] = [10 + 10, 0, 10 + 10]
#         cube[8] = [10, 0 + 10, 10 + 10]
#
#         cube[9] = [10 + 10, 0, 10 + 10]
#         cube[10] = [10 + 10, 0 + 10, 10 + 10]
#         cube[11] = [10, 0 + 10, 10 + 10]
#
#         cube[12] = [10, 0, 10]
#         cube[13] = [10, 0 + 10, 10]
#         cube[14] = [10, 0, 10 + 10]
#
#         cube[15] = [10, 0 + 10, 10]
#         cube[16] = [10, 0 + 10, 10 + 10]
#         cube[17] = [10, 0, 10 + 10]
#
#         cube[18] = [10 + 10, 0, 10]
#         cube[19] = [10 + 10, 0, 10 + 10]
#         cube[20] = [10 + 10, 0 + 10, 10]
#
#         cube[21] = [10 + 10, 0 + 10, 10]
#         cube[22] = [10 + 10, 0, 10 + 10]
#         cube[23] = [10 + 10, 0 + 10, 10 + 10]
#
#         cube[24] = [10, 0 + 10, 10]
#         cube[25] = [10 + 10, 0 + 10, 10]
#         cube[26] = [10 + 10, 0 + 10, 10 + 10]
#
#         cube[27] = [10, 0 + 10, 10]
#         cube[29] = [10, 0 + 10, 10 + 10]
#         cube[28] = [10 + 10, 0 + 10, 10 + 10]
#
#     def move_cube(self, x, y, z):
#         for i in range(30):
#             self.points[i][0] += x
#             self.points[i][1] += y
#             self.points[i][2] += z
#
#     def get_cube(self):
#         return self.points
import math

if __name__ == '__main__':
    print(min(3,1,1,3,4,6))