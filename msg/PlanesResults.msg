int32 N                 # Number of detected planes.

# In the following arrays, 
# the planes' parameters are concatinated one by one.

float32[] norms         # Nx3. Plane normal (nx, ny, nz).
float32[] center_3d     # Nx3. Plane 3D center (cx, cy, cz).
float32[] center_2d     # Nx2. Plane 2D center on the image (px, py),
                        #   which means {px}th column, and {py}th row.
uint16[] mask_color     # Nx3. Plane mask color (blue, green, red). 
                        # Each color's range is [0, 255]