# realTimePhysics
Collection of assignment projects for real-time physics module

Folders 1 - 6 contain source code directly related to the real-time physics problems indicated by the folder names only.
The folder labelled "Other Souce Code" contains other classes and functions I have written not directly for the purpose of the real-time physics course that provide some basic 3D graphics functionality such as displaying a skybox.
The folder labelled "Third Party Files" contains various files that I have not written but which I have used to implement functionality such as writing text to screen, or vector and matrix mathematics.

Project Descriptions:
1 - Particle System
A basic particle system simulation with 1200 particles. The particles are affected by gravitational and drag forces, as well as collisions with the cube in which they are contained, and friction parallel to any planes with which they are in contact. There is also a "Magic Pear" special effect force, which causes the particles to move into the shape of a pear.
Demo video: https://youtu.be/KgT5Rh_EViU

2 - Rigid Body Unconstrained Motion
A simple system which simulates displacement, rotation, angular and linear momentum, force and torque applied to a rigid body. Gravity, simplified drag, and a spring force are also implemented.
Demo video: https://youtu.be/6y2-9FBjxX4

3 - Distance and Contact
An implementation of intersection tests to determine the closest point on a line to a point, closest point on a plane to a point, closet point on a triangle to a point, closest points on two triangles, and distance checks for point-to-point, point-to-line, point-to-plane, and triangle-to-triangle.
Demo video: https://youtu.be/XjhAis1C7r0

4 - Broad-Phase Collision Detection
Broad-phase collision detection is implemented here using bounding spheres and oriented bounding boxes, for 25 rigid bodies.
Demo video: https://youtu.be/qlZYZJj0FMY

5 - Impulse-Based Collision Response
Implements collsions between a rigid body and an infinite plane. Solution works for arbitrary small polyherdra, and plane at an arbitrary slope.
Demo video: https://youtu.be/pUBAwNIkOj0

6 - Position-Based Deformable Objects
An implementation of the deformable objects approach described in "Meshless deformations based on shape matching" by Matthias Muller, Bruno Heidelberger, Matthias Teschner, and Markus Gross. An adapted particle system is used to simulate the forces and motion of the vertices of the body, and shape-matching is used to determine the deformation of the body.
Demo video: https://youtu.be/Fz596NLAXfs

External Libraries used:
Armadillo: A C++ Linear Algebra Library, used here to calculate the square root of a matrix.
• Antons_maths_funcs: a library for vector, matrix and quaternion mathe-matics. However, I have extended this library myself to handle additional mathematical operations such as calculation of the determinant and in-verse of 3x3 matrices, orthonormalisation of 3x3 matrices, and 3x3 matrix by vector multiplication as the original author was primarily focused on 4x4 matrix mathematics.
• gl_utils: (slightly modiﬁed), a library of common OpenGL functionality.
• text: a library for writing text on-screen in an OpengGL application.
• obj_parser: to provide mesh loading functionality.
• stb_image: an image loading library.
• assimp: for loading 3D meshes.
• glew: the OpenGL extension wrangler.
• glfw: an OpenGL development framework.