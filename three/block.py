import math
import taichi as ti



ti.init(arch=ti.cuda)

screen_res = (800, 800)
screen_to_world_ratio = 20.0
boundary = [screen_res[0] / screen_to_world_ratio,
            screen_res[1] / screen_to_world_ratio,
            screen_res[0] / screen_to_world_ratio,]  # [40, 40, 40]
cell_size = 2.51
cell_recpr = 1.0 / cell_size


def round_up(f, s):
    return (math.floor(f * cell_recpr / s) + 1) * s


grid_size = [round_up(boundary[0], 1), round_up(boundary[1], 1), round_up(boundary[2], 1)]

dim = 3

num_particles_x = 10

num_particles = 15000
max_num_particles_per_cell = 100
max_num_neighbors = 100
delta_t = 1.0 / 20.0
epsilon = 1e-5
particle_radius = 3.0
particle_radius_in_world = particle_radius / screen_to_world_ratio  # 0.15


h = 1.1
mass = 1.0
rho0 = 1.0
lambda_epsilon = 100.0

solver_iterations = 5
corr_deltaQ_coeff = 0.3
corrK = 0.001

neighbor_radius = h * 1.05

poly6_factor = 315.0 / 64.0 / math.pi
spiky_grad_factor = -45.0 / math.pi

old_positions = ti.Vector.field(dim, float)
positions = ti.Vector.field(dim, float)
velocities = ti.Vector.field(dim, float)
grid_num_particles = ti.field(int)

particle_num_neighbors = ti.field(int)
particle_neighbors = ti.field(int)
lambdas = ti.field(float)
delta_p = ti.Vector.field(dim, float)

block = ti.Vector.field(dim + 1, float) # [x, y, z, edge_length]

cube = ti.Vector.field(3, dtype=ti.f32, shape=30)


ti.root.dense(ti.i, num_particles).place(old_positions, positions, velocities)
grid_snode = ti.root.dense(ti.ijk, grid_size)
grid_snode.place(grid_num_particles)

grid2particles = ti.field(int, (grid_size + [max_num_particles_per_cell,]))
nb_node = ti.root.dense(ti.i, num_particles)
nb_node.place(particle_num_neighbors)
nb_node.dense(ti.j, max_num_neighbors).place(particle_neighbors)
ti.root.dense(ti.i, num_particles).place(lambdas, delta_p)


ti.root.place(block)


@ti.func
def w(s, h):
    """
            calculate W()
    """
    result = 0.0
    if 0 < s and s < h:
        x = (h * h - s * s) / (h * h * h)
        result = poly6_factor * x * x * x
    return result


@ti.func
def nabla_w(r, h):
    """
            calculate ∇W
    """
    result = ti.Vector([0.0, 0.0, 0.0])
    r_len = r.norm()
    if 0 < r_len and r_len < h:
        x = (h - r_len) / (h * h * h)
        g_factor = spiky_grad_factor * x * x
        result = r * g_factor / r_len
    return result


@ti.func
def calculate_s_corr(position_i, position_j):
    """
            calculate s_corr
            set n = 4
    """
    res = w((position_i - position_j).norm(), h) / w(corr_deltaQ_coeff * h, h)
    res = (-corrK) * res * res * res * res
    return res


@ti.func
def get_cell(pos):
    return int(pos * cell_recpr)


@ti.func
def is_in_grid(c):
    return 0 <= c[0] and c[0] < grid_size[0] and 0 <= c[1] and c[1] < grid_size[1]


@ti.func
def confine_position_to_boundary(p):
    # p: position[x, y, z]
    bmin = particle_radius_in_world
    bmax = ti.Vector([boundary[0], boundary[1], boundary[2]
                      ]) - particle_radius_in_world
    for i in ti.static(range(dim)):
        if p[i] <= bmin:
            p[i] = bmin + epsilon * ti.random()
        elif bmax[i] <= p[i]:
            p[i] = bmax[i] - epsilon * ti.random()

    # collision with block
    base_x = cube[0][0]
    base_z = cube[0][2]
    edge_length = 10
    if base_x < p[0] < base_x + edge_length and base_z < p[2] < base_z + edge_length:
        if 0 < p[1] < edge_length:
            top = edge_length - p[1]
            x1 = p[0] - base_x
            x2 = base_x + edge_length - p[0]
            z1 = p[2] - base_z
            z2 = base_z + edge_length - p[2]
            min_value = min(top, x1, x2, z1, z2)
            if min_value == top:
                p[1] = edge_length + epsilon * ti.random()
            elif min_value == x1:
                p[0] = base_x - epsilon * ti.random()
            elif min_value == x2:
                p[0] = base_x + edge_length + epsilon * ti.random()
            elif min_value == z1:
                p[2] = base_z - epsilon * ti.random()
            elif min_value == z2:
                p[2] = base_z + edge_length + epsilon * ti.random()

    return p


@ti.kernel
def apply_force():
    """
        apply gravity to particles, update v_i and x_i
        v_i = v_i + gravity * delta_t
        x_i = x_i + v_i * delta_t
    """
    for i in positions:
        old_positions[i] = positions[i]
    for i in positions:
        position = positions[i]
        velocity = velocities[i]
        gravity = ti.Vector([0.0, -0.98, 0.0])
        velocity += delta_t * gravity
        position += delta_t * velocity
        positions[i] = confine_position_to_boundary(position)

@ti.kernel
def find_neighbors():
    """
        find the number of particles' neighbor
        calculate N_i(x_i)

    """
    # clear
    for I in ti.grouped(grid_num_particles):
        grid_num_particles[I] = 0
    for I in ti.grouped(particle_neighbors):
        particle_neighbors[I] = -1

    # update grid
    for p_i in positions:
        cell = get_cell(positions[p_i])
        offs = ti.atomic_add(grid_num_particles[cell], 1)
        grid2particles[cell, offs] = p_i
    # find particle neighbors
    for p_i in positions:
        pos_i = positions[p_i]
        cell = get_cell(pos_i)
        nb_i = 0
        for offs in ti.static(ti.grouped(ti.ndrange((-1, 2), (-1, 2),(-1, 2)))):
            cell_to_check = cell + offs
            if is_in_grid(cell_to_check):
                for j in range(grid_num_particles[cell_to_check]):
                    p_j = grid2particles[cell_to_check, j]
                    if nb_i < max_num_neighbors and p_j != p_i and (
                            pos_i - positions[p_j]).norm() < neighbor_radius:
                        particle_neighbors[p_i, nb_i] = p_j
                        nb_i += 1
        particle_num_neighbors[p_i] = nb_i

@ti.kernel
def calculate_lambda():
    """
        calculate lambda_i of particles in iteration
        lambda_i = - C_i(p_1,..., p_n) / (∑|nabla(p_k) * c_i|^2 + ε)
        if k == i:
            nabla(p_k) * c_i = ∑nabla(p_k) * W(p_i - p_j, h) / rho_0
        if k == j:
            nabla(p_k) * c_i = -nabla(p_k) * W(p_i - p_j, h) / rho_0
    """
    for p_i in positions:
        pos_i = positions[p_i]

        grad_i = ti.Vector([0.0, 0.0, 0.0])
        sum_gradient_sqr = 0.0
        density_constraint = 0.0

        for j in range(particle_num_neighbors[p_i]):
            p_j = particle_neighbors[p_i, j]
            if p_j < 0:
                break
            pos_ji = pos_i - positions[p_j]
            grad_j = nabla_w(pos_ji, h)
            grad_i += grad_j
            sum_gradient_sqr += grad_j.dot(grad_j)
            density_constraint += w(pos_ji.norm(), h)

        density_constraint = (mass * density_constraint / rho0) - 1.0

        sum_gradient_sqr += grad_i.dot(grad_i)
        lambdas[p_i] = (-density_constraint) / (sum_gradient_sqr +
                                                lambda_epsilon)


@ti.kernel
def calculate_delta_p():
    """
        calculate delta_p of particles in iteration
        delta_p = ∑(lambda_i + lambda_j + s_corr) * nabla_W(p_i - p_j, h) / rho_0
    """
    for i in positions:
        delta_p_i = ti.Vector([0.0, 0.0, 0.0])
        lambda_i = lambdas[i]
        for j in range(particle_num_neighbors[i]):
            neighbor = particle_neighbors[i, j]
            if neighbor < 0:
                break
            s_corr = calculate_s_corr(positions[i], positions[neighbor])
            lambda_j = lambdas[neighbor]
            delta_p_i += (lambda_i + lambda_j + s_corr) * nabla_w(positions[i] - positions[neighbor], h=h)
        delta_p[i] = delta_p_i / rho0

@ti.kernel
def update_position_in_iteration():
    """
        update position of particles in iteration
        new_x_i = new_x_i + delta_p
    """
    for i in positions:
        positions[i] += delta_p[i]

@ti.kernel
def update_final_position():
    """
        update final velocity and position of particles
        v_i = (new_x_i - x_i) / delta_t
        x_i = new_x_i
    """
    # update velocity of particle
    for i in positions:
        velocities[i] = (positions[i] - old_positions[i]) / delta_t
    # update position of particle
    for i in positions:
        positions[i] = confine_position_to_boundary(positions[i])


@ti.kernel
def init():
    init_pos = ti.Vector([10.0,10.0,10.0])
    num_per_row = 21
    num_per_floor = 21 * 21
    for i in range(num_particles):
        floor = i // (num_per_floor)
        row = (i % num_per_floor) // num_per_row
        col = (i % num_per_floor) % num_per_row
        positions[i] = ti.Vector([col, floor, row]) + init_pos

    block[None] = ti.Vector([10, 0, 10, 10])
    cube[0] = [10, 0, 10]
    cube[1] = [10 + 10, 0, 10]
    cube[2] = [10, 0 + 10, 10]

    cube[3] = [10 + 10, 0, 10]
    cube[4] = [10 + 10, 0 + 10, 10]
    cube[5] = [10, 0 + 10, 10]

    cube[6] = [10, 0, 10 + 10]
    cube[7] = [10 + 10, 0, 10 + 10]
    cube[8] = [10, 0 + 10, 10 + 10]

    cube[9] = [10 + 10, 0, 10 + 10]
    cube[10] = [10 + 10, 0 + 10, 10 + 10]
    cube[11] = [10, 0 + 10, 10 + 10]

    cube[12] = [10, 0, 10]
    cube[13] = [10, 0 + 10, 10]
    cube[14] = [10, 0, 10 + 10]

    cube[15] = [10, 0 + 10, 10]
    cube[16] = [10, 0 + 10, 10 + 10]
    cube[17] = [10, 0, 10 + 10]

    cube[18] = [10 + 10, 0, 10]
    cube[19] = [10 + 10, 0, 10 + 10]
    cube[20] = [10 + 10, 0 + 10, 10]

    cube[21] = [10 + 10, 0 + 10, 10]
    cube[22] = [10 + 10, 0, 10 + 10]
    cube[23] = [10 + 10, 0 + 10, 10 + 10]

    cube[24] = [10, 0 + 10, 10]
    cube[25] = [10 + 10, 0 + 10, 10]
    cube[26] = [10 + 10, 0 + 10, 10 + 10]

    cube[27] = [10, 0 + 10, 10]
    cube[29] = [10, 0 + 10, 10 + 10]
    cube[28] = [10 + 10, 0 + 10, 10 + 10]


def main3d():
    global cube
    init()

    window = ti.ui.Window("fluid simulation 3D", screen_res)
    canvas = window.get_canvas()
    scene = ti.ui.Scene()
    camera = ti.ui.make_camera()
    camera.position(80, 60, 80)

    # draw borders, consists of three mesh
    borders = ti.Vector.field(3, dtype=ti.f32, shape=6)
    borders[0] = [0, 0, 0]
    borders[1] = [0, 0, 40]
    borders[2] = [40, 0, 40]
    borders[3] = [0, 0, 0]
    borders[4] = [40, 0, 40]
    borders[5] = [40, 0, 0]




    while window.running:
        camera.track_user_inputs(window, movement_speed=1, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        scene.ambient_light((0.8, 0.8, 0.8))
        scene.point_light(pos=(0.5, 1.5, 1.5), color=(1, 1, 1))

        if window.get_event(ti.ui.PRESS):
            if window.event.key == 'i':
                for i in range(30):
                    cube[i] += [-2, 0, 0]
            elif window.event.key == 'j':
                for i in range(30):
                    cube[i] += [0, 0, -2]
            elif window.event.key == 'k':
                for i in range(30):
                    cube[i] += [2, 0, 0]
            elif window.event.key == 'l':
                for i in range(30):
                    cube[i] += [0, 0, 2]

        # begin pbf algorithm
        iter = 0
        apply_force()
        find_neighbors()  # begin
        while iter < solver_iterations:
            calculate_lambda()
            calculate_delta_p()
            update_position_in_iteration()
            iter += 1
        update_final_position()
        # end pbf algorithm

        # draw particles
        scene.particles(positions, color=(0, 0.75, 1), radius=0.1)

        # draw bottom border
        scene.mesh(borders)

        # draw block
        scene.mesh(cube)


        canvas.scene(scene)
        window.show()


if __name__ == '__main__':
    main3d()

