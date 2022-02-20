class POI():
    def __init__(self, identifier, position = [0, 0, 0], direction = [0, 0, 0]):
        self.identifier = identifier
        self.position = position
        self.direction = direction

        # TODO: Find which gazebo direction corresponds to valve "inspection direction" and store that in self.direction