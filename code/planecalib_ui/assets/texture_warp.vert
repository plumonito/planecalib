attribute vec4 aPosCoord;
attribute vec2 aTexCoord;
uniform mat4 uMVPMatrix;
uniform mat3 uHomography;

varying vec3 vTexCoord;

void main(void)
{
	vTexCoord = uHomography * vec3(aTexCoord,1.0);
  	gl_Position = uMVPMatrix * aPosCoord;
}
