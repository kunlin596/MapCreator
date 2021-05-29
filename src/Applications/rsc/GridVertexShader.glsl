#version 410 core

layout (location = 0) in vec3 position;

uniform mat4 transformation_matrix;

void main() {

    gl_Position = transformation_matrix * vec4(position, 1.0);

}