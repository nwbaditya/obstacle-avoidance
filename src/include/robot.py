class Robot:
    def __init__(self, radius):
        self.radius = radius
        self.position = [0,0]
        self.velocity = [0,0]
        self.desired_velocity = [0,0]
        self.max_velocity = [0,0]

    