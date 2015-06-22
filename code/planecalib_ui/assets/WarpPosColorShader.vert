uniform mat3 uHomography;
uniform mat4 uMVPMatrix;
attribute vec2 aPosCoord;
attribute vec4 aColor;

varying vec4 vColor;

void main(void)
{
	vec3 pos3 = uHomography*vec3(aPosCoord.x, aPosCoord.y, 1.0);
	gl_Position = uMVPMatrix * vec4(pos3, pos3.z);

	vColor = aColor;
}
