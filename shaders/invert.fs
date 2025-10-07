#version 330

// Input from vertex shader
in vec2 fragTexCoord;

// Uniforms provided by raylib
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Final output color
out vec4 finalColor;

void main() {
    vec4 texColor = texture(texture0, fragTexCoord);
    vec4 invColor = vec4(1.0 - texColor.r, 1.0 - texColor.g, 1.0 - texColor.b, texColor.a);
    finalColor = invColor;
}
