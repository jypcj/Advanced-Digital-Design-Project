import taichi as ti


class Cube:
    def __init__(self, base_x, base_y, base_z, edge_length):
        self.points = ti.Vector.field(3, dtype=ti.f32, shape=30)

        self.points[0] = [base_x, base_y, base_z]
        self.points[1] = [base_x + edge_length, base_y, base_z]
        self.points[2] = [base_x, base_y + edge_length, base_z]

        self.points[3] = [base_x + edge_length, base_y, base_z]
        self.points[4] = [base_x + edge_length, base_y + edge_length, base_z]
        self.points[5] = [base_x, base_y + edge_length, base_z]

        self.points[6] = [base_x, base_y, base_z + edge_length]
        self.points[7] = [base_x + edge_length, base_y, base_z + edge_length]
        self.points[8] = [base_x, base_y + edge_length, base_z + edge_length]

        self.points[9] = [base_x + edge_length, base_y, base_z + edge_length]
        self.points[10] = [base_x + edge_length, base_y + edge_length, base_z + edge_length]
        self.points[11] = [base_x, base_y + edge_length, base_z + edge_length]

        self.points[12] = [base_x, base_y, base_z]
        self.points[13] = [base_x, base_y + edge_length, base_z]
        self.points[14] = [base_x, base_y, base_z + edge_length]

        self.points[15] = [base_x, base_y + edge_length, base_z]
        self.points[16] = [base_x, base_y + edge_length, base_z + edge_length]
        self.points[17] = [base_x, base_y, base_z + edge_length]

        self.points[18] = [base_x + edge_length, base_y, base_z]
        self.points[19] = [base_x + edge_length, base_y, base_z + edge_length]
        self.points[20] = [base_x + edge_length, base_y + edge_length, base_z]

        self.points[21] = [base_x + edge_length, base_y + edge_length, base_z]
        self.points[22] = [base_x + edge_length, base_y, base_z + edge_length]
        self.points[23] = [base_x + edge_length, base_y + edge_length, base_z + edge_length]

        self.points[24] = [base_x, base_y + edge_length, base_z]
        self.points[25] = [base_x + edge_length, base_y + edge_length, base_z]
        self.points[26] = [base_x + edge_length, base_y + edge_length, base_z + edge_length]

        self.points[27] = [base_x, base_y + edge_length, base_z]
        self.points[29] = [base_x, base_y + edge_length, base_z + edge_length]
        self.points[28] = [base_x + edge_length, base_y + edge_length, base_z + edge_length]

    def move_cube(self, x, y, z):
        for i in range(30):
            self.points[i][0] += x
            self.points[i][1] += y
            self.points[i][2] += z

    def get_cube(self):
        return self.points
