import pygame as pg
from random import uniform
from vehicle import Vehicle, mass_to_size_constant, circle_width

boid_uid = 0

class Boid(Vehicle):

    # CONFIG
    debug = False
    min_speed = .001
    max_speed = .02
    max_force = 1
    max_turn = 5
    perception = 60
    # crowding = 15
    low_crowding = 1
    high_crowding = 15
    can_wrap = True
    edge_distance_pct = 5
    ###############

    def __init__(self):
        global boid_uid
        self.uid = boid_uid
        boid_uid += 1
        Boid.set_boundary(Boid.edge_distance_pct)

        # Randomize starting position and velocity
        start_position = pg.math.Vector2(
            uniform(0, Boid.max_x),
            uniform(0, Boid.max_y))
        start_velocity = pg.math.Vector2(
            uniform(-1, 1) * Boid.max_speed,
            uniform(-1, 1) * Boid.max_speed)

        super().__init__(start_position, start_velocity,
                         Boid.min_speed, Boid.max_speed,
                         Boid.max_force, Boid.can_wrap)

        self.rect = self.image.get_rect(center=self.position)

        self.debug = Boid.debug
        # self.debug = True

    def separation(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            dist = self.position.distance_to(boid.position)
            # Lower crowding based on mass - after the threshold
            if dist < self.high_crowding:
                steering -= boid.position - self.position
        steering = self.clamp_force(steering)
        return steering

    def alignment(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.velocity
        steering /= len(boids)
        steering -= self.velocity
        steering = self.clamp_force(steering)
        return steering / 8

    def cohesion(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.position
        steering /= len(boids)
        steering -= self.position
        steering = self.clamp_force(steering)
        return steering / 100

    def update(self, dt, boids):
        steering = pg.Vector2()

        if not self.can_wrap:
            steering += self.avoid_edge()

        neighbors = self.get_neighbors(boids)
        if neighbors:

            separation = self.separation(neighbors)
            alignment = self.alignment(neighbors)
            cohesion = self.cohesion(neighbors)

            # DEBUG
            # separation *= 0
            # alignment *= 0
            # cohesion *= 0

            steering += separation + alignment + cohesion

        # steering = self.clamp_force(steering)

        super().update(dt, steering)

    def get_neighbors(self, boids):
        neighbors = []
        for boid in boids:
            if boid != self:
                dist = self.position.distance_to(boid.position)
                # We see in a circle
                if dist < self.perception:
                    neighbors.append(boid)
                if dist < self.radius and self.mass > boid.mass:
                    # print(f"boid {self.uid} eating {boid.uid}")
                    self.eat_boid(boid)
                    # Eat!
                    # pass
                    # print(f"We would eat boid {boid.uid}!")
        return neighbors

    def eat_boid(self, target):
        print(f"Boid {self.uid} mass {self.mass} eating {target.uid} mass {target.mass}")
        self.mass += target.mass
        target.mass = 0.
        target.kill()
        # target.delete = True
        # del target
        old_radius = self.radius
        self.radius = pow(self.mass, 0.5) * mass_to_size_constant
        center_offset = (self.radius - old_radius) / 2
        self.position += pg.Vector2(center_offset, center_offset)
        del self.image
        self.image = pg.Surface((self.radius * 2, self.radius * 2), pg.SRCALPHA)
        pg.draw.circle(
            surface = self.image,
            color = pg.Color("White"),
            center = (int(self.radius), int(self.radius)),
            radius = self.radius,
            width = circle_width)
