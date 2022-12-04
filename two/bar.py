import math

import taichi as ti

ti.init(arch=ti.cuda)

screen_res = (600, 600)
screen_to_world_ratio = 10.0
boundary = (screen_res[0] / screen_to_world_ratio,
            screen_res[1] / screen_to_world_ratio)
cell_size = 2.51
cell_recpr = 1.0 / cell_size


def round_up(f, s):
    return (math.floor(f * cell_recpr / s) + 1) * s


grid_size = (round_up(boundary[0], 1), round_up(boundary[1], 1))

dim = 2
bg_color = 0xbb6b6c


num_particles_x = 60
num_particles = num_particles_x * 20
max_num_particles_per_cell = 100
max_num_neighbors = 100
delta_t = 1.0 / 20.0
epsilon = 1e-5
particle_radius = 3.0
particle_radius_in_world = particle_radius / screen_to_world_ratio


h_ = 1.1
mass = 1.0
rho0 = 1.0
lambda_epsilon = 100.0
pbf_num_iters = 6
solver_iterations = 6
corr_deltaQ_coeff = 0.3
corrK = 0.001

neighbor_radius = h_ * 1.05



poly6_factor = 315.0 / 64.0 / math.pi
spiky_grad_factor = -45.0 / math.pi

old_positions = ti.Vector.field(dim, float)
positions = ti.Vector.field(dim, float)
velocities = ti.Vector.field(dim, float)
grid_num_particles = ti.field(int)
grid2particles = ti.field(int)
particle_num_neighbors = ti.field(int)
particle_neighbors = ti.field(int)
lambdas = ti.field(float)
delta_p = ti.Vector.field(dim, float)

kk = ti.Vector.field(2, float)


ti.root.dense(ti.i, num_particles).place(old_positions, positions, velocities)
grid_snode = ti.root.dense(ti.ij, grid_size)
grid_snode.place(grid_num_particles)
grid_snode.dense(ti.k, max_num_particles_per_cell).place(grid2particles)
nb_node = ti.root.dense(ti.i, num_particles)
nb_node.place(particle_num_neighbors)
nb_node.dense(ti.j, max_num_neighbors).place(particle_neighbors)
ti.root.dense(ti.i, num_particles).place(lambdas, delta_p)

ti.root.place(kk)

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
    result = ti.Vector([0.0, 0.0])
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
    res = w((position_i - position_j).norm(), h_) / w(corr_deltaQ_coeff * h_, h_)
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
    bmin = particle_radius_in_world
    bmax = ti.Vector([boundary[0], boundary[1]
                      ]) - particle_radius_in_world
    for i in ti.static(range(dim)):
        if p[i] <= bmin:
            p[i] = bmin + epsilon * ti.random()
        elif bmax[i] <= p[i]:
            p[i] = bmax[i] - epsilon * ti.random()

    # collision with block
    k_value = kk[None][0]
    if p[1] < k_value * p[0]:
        x = (k_value * p[1] + p[0]) / (1 + k_value * k_value)
        y = k_value * x
        p[0] = x - epsilon * ti.random()
        p[1] = y + epsilon * ti.random()

    return p


@ti.kernel
def apply_force():
    """
        apply gravity to particles, update v_i and x_i
        v_i = v_i + gravity * delta_t
        x_i = x_i + v_i * delta_t
    """
    # save positions of particles
    for i in positions:
        old_positions[i] = positions[i]
    # update x_i
    for i in positions:
        position = positions[i]
        velocity = velocities[i]
        gravity = ti.Vector([0.0, -9.8])
        velocity += delta_t * gravity
        position += velocity * delta_t
        positions[i] = confine_position_to_boundary(position)


@ti.kernel
def find_neighbors():
    """
        find the number of particles' neighbor
        calculate N_i(x_i)

    """
    # clear
    for i in ti.grouped(grid_num_particles):
        grid_num_particles[i] = 0
    for i in ti.grouped(particle_neighbors):
        particle_neighbors[i] = -1

    for i in positions:
        cell = get_cell(positions[i])
        offs = ti.atomic_add(grid_num_particles[cell], 1)
        grid2particles[cell, offs] = i

    # count the number of neighbors
    for p_i in positions:
        neighbors_number = 0
        cell = get_cell(positions[p_i])
        for offs in ti.static(ti.grouped(ti.ndrange((-1, 2), (-1, 2)))):
            cell_to_check = cell + offs
            if is_in_grid(cell_to_check):
                for j in range(grid_num_particles[cell_to_check]):
                    p_j = grid2particles[cell_to_check, j]
                    if neighbors_number < max_num_neighbors and p_j != p_i and (
                            positions[p_i] - positions[p_j]).norm() < neighbor_radius:
                        particle_neighbors[p_i, neighbors_number] = p_j
                        neighbors_number += 1
        particle_num_neighbors[p_i] = neighbors_number


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

        grad_i = ti.Vector([0.0, 0.0])
        sum_gradient_sqr = 0.0
        density_constraint = 0.0

        for j in range(particle_num_neighbors[p_i]):
            p_j = particle_neighbors[p_i, j]
            if p_j < 0:
                break
            pos_ji = pos_i - positions[p_j]
            grad_j = nabla_w(pos_ji, h_)
            grad_i += grad_j
            sum_gradient_sqr += grad_j.dot(grad_j)
            density_constraint += w(pos_ji.norm(), h_)

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
        delta_p_i = ti.Vector([0.0, 0.0])
        lambda_i = lambdas[i]
        for j in range(particle_num_neighbors[i]):
            neighbor = particle_neighbors[i, j]
            if neighbor < 0:
                break
            s_corr = calculate_s_corr(positions[i], positions[neighbor])
            lambda_j = lambdas[neighbor]
            delta_p_i += (lambda_i + lambda_j + s_corr) * nabla_w(positions[i] - positions[neighbor], h=h_)
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


def draw(gui):
    # draw background
    gui.clear(bg_color)

    # draw particles
    pos_np = positions.to_numpy()
    for j in range(dim):
        pos_np[:, j] *= screen_to_world_ratio / screen_res[j]
    gui.circles(pos_np, radius=particle_radius, color=0x00bfff)

    # draw a board

    gui.line(begin=(0, 0), end=(1, kk[None][0]), color=0x000000, radius=2.5)

    gui.show()


@ti.kernel
def init_particles():
    for i in range(num_particles):
        delta = h_ * 0.8
        offs = ti.Vector([(boundary[0] - delta * num_particles_x) * 0.5,
                          boundary[1] * 0.02])
        positions[i] = ti.Vector([i % num_particles_x, i // num_particles_x + 40
                                  ]) * delta + offs
        for c in ti.static(range(dim)):
            velocities[i][c] = (ti.random() - 0.5) * 4

        kk[None] = ti.Vector([0.5, 0])


def main_bar():
    init_particles()
    gui = ti.GUI('fluid simulation 2D', screen_res)

    while gui.running:
        for e in gui.get_events(gui.PRESS):
            # right board
            if e.key == 'w':
                kk[None][0] += 0.1
            elif e.key == 's':
                kk[None][0] -= 0.1


        # begin pbf algorithm
        apply_force()
        find_neighbors()
        iter = 0
        while iter < solver_iterations:
            calculate_lambda()
            calculate_delta_p()
            update_position_in_iteration()
            iter += 1
        update_final_position()
        # end pbf algorithm

        draw(gui)


if __name__ == '__main__':
    main_bar()