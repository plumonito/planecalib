#ifdef GL_ES_VERSION_2_0
precision mediump float;
#endif

uniform sampler2D uTexture;
uniform float uAlpha;
varying vec3 vTexCoord;

void main(void)
{
    gl_FragColor.rgb = texture2DProj(uTexture, vTexCoord).rgb;
	gl_FragColor.a = uAlpha;
}
