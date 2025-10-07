#version 330

// Input from vertex shader
in vec2 fragTexCoord;

// Uniforms provided by raylib
uniform sampler2D colorTexture;
uniform vec4 colDiffuse;

// Final output color
out vec4 finalColor;

float redOffset = 0.003;
float greenOffset = 0.0001;
float blueOffset = -0.003;

vec2 textSize = textureSize(colorTexture, 0).xy;

void main() {
    finalColor.r = texture(colorTexture, fragTexCoord + vec2(redOffset, 0)).r;
    finalColor.g = texture(colorTexture, fragTexCoord + vec2(greenOffset, 0)).g;
    finalColor.ba = texture(colorTexture, fragTexCoord + vec2(blueOffset, 0)).ba;
}

