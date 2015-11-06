#version 410 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 in_color;

uniform mat4 transformation_matrix;

out vec3 pass_color;

void main() {

    gl_Position = transformation_matrix * vec4(position, 1.0);

    if (gl_Position.z > 100)
        gl_PointSize = 5;
    else
        gl_PointSize = 10.0 / gl_Position.z;

    pass_color = in_color;
}
