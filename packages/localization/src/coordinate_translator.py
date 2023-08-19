class TileMapTranslator:
    def __init__(self, area_width, area_height, origin_x_1, origin_x_2, origin_y_1, origin_y_2):
        """
        Assumptions: 
        1. origin 1 must be the bottom right corner of the mat.
        2. origin 2 must be the bottom middle of the mat. 
        3. Either the x values or the y values of the coordinates must be 
        equal to each other so that the mat is level with the optitrack coordinate
        system in some way. 
        """
        self.area_width = area_width
        self.area_height = area_height
        self.origin_x_1 = origin_x_1 #bottom left
        self.origin_y_1 = origin_y_1 
        self.origin_x_2 = origin_x_2 # bottom right
        self.origin_y_2 = origin_y_2 

    def translate_coordinate(self, opti_x, opti_y):
        """
        This function takes in an optitrack coordinate and 
        returns what section of a lane the coordinate is in.

        Arguments: 
        opti_x - the x oordinate as given by optitrack 
        opti_y - the z coordinate as given by optitrack

        Returns:
        area_number - The section number that the coordinate is in.
        relative_y - The y position in relation to the mat and not optitrack.
        """

        relative_y = "Mat not set up correctly"
        area_number = "^^^^^^^^^^^^^^^^^^^^^^^"

        if self.origin_y_1 == self.origin_y_2 and self.origin_x_1 < self.origin_x_2: #mat pointing up
            if opti_x < self.origin_x_1 or opti_x > self.origin_x_2 or opti_y < self.origin_y_1: #makes sure object is on mat
                relative_y = "Not in area"
                area_number = "N/A1"
            else:
                relative_y = abs(opti_y - self.origin_y_1)
                area_number = int(relative_y / self.area_height) + 1  #section on the mat
                print("1")

        elif self.origin_y_1 == self.origin_y_2 and self.origin_x_1 > self.origin_x_2: #mat pointing down
            if opti_x < self.origin_x_1 or opti_x > self.origin_x_2 or opti_y > self.origin_y_1: #makes sure object is on mat
                relative_y = "Not in area"
                area_number = "N/A2"
            else:
                relative_y = abs(opti_y - self.origin_y_1)
                area_number = int(relative_y / self.area_height) + 1  #section on the mat
                print("2")

        elif self.origin_x_1 == self.origin_x_2 and self.origin_y_1 < self.origin_y_2: #mat pointing left
            if opti_x > self.origin_x_1 or opti_y > self.origin_y_2 or opti_y < self.origin_y_1: #makes sure object is on mat
                relative_y = "Not in area"
                area_number = "N/A3"
            else:
                relative_y = abs(opti_x - self.origin_x_1)
                area_number = int(relative_y / self.area_height) + 1 #section on the mat
                print("3")

        elif self.origin_x_1 == self.origin_x_2 and self.origin_y_1 > self.origin_y_2: #mat pointing right
            if opti_x < self.origin_x_1 or opti_y > self.origin_y_2 or opti_y < self.origin_y_1: #makes sure object is on mat
                relative_y = "Not in area"
                area_number = "N/A4"
            else:
                relative_y = abs(opti_x - self.origin_x_1)
                area_number = int(relative_y / self.area_height) + 1 #section on the mat
                print("4")

        return area_number, relative_y

# Example usage
if __name__ == "__main__":
    AREA_WIDTH = 0.146
    AREA_HEIGHT = 0.288931
    origin_x_1 = -1.72  #bottom of mat bottom corner/middle (must be right most in lab)
    origin_y_1 = -1.478
    origin_x_2 = -1.574  #bottom of mat bottom corner/middle (must be left most in lab)
    origin_y_2 = -1.478

    translator = TileMapTranslator(AREA_WIDTH, AREA_HEIGHT, origin_x_1, origin_x_2, origin_y_1, origin_y_2)

    opti_x = -1.665  #actual optitrack reading
    opti_y = 0.044


    area_number, relative_y = translator.translate_coordinate(opti_x, opti_y)

    print(f"Input Coordinate: ({opti_x}, {opti_y})")
    print(f"Relative y: ({str(relative_y)})")
    print("Area Number: " + str(area_number))