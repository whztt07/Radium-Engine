out vec4 fragColor;

in vec2 varTexcoord;

uniform sampler2D screenTexture;
uniform sampler2D color0;
uniform sampler2D color1;

uniform int isDepthTexture;
uniform float zNear;
uniform float zFar;

uniform float totalTime;

void main()
{
    vec4 value;

    vec4 value0 = vec4(texture(screenTexture, varTexcoord));
    vec4 value1 = vec4(texture(color0, varTexcoord));
    vec4 value2 = vec4(texture(color1, varTexcoord));

    value = vec4(texture(screenTexture, varTexcoord));

    if (bool(isDepthTexture))
    {
        float n = 1.0;
        float f = 1000.0;
        float z = value.r;
        float linearDepth = (2.0 * n) / (f + n - z  * (f - n));
        value.rgb = vec3(z);
    }

    fragColor = value0;
//    fragColor = vec4(varTexcoord, 0, 1);
}

