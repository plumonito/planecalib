#ifdef GL_ES_VERSION_2_0
precision mediump float;
#endif

uniform sampler2D uTexture;
uniform vec4 uColor;
varying vec3 vTexCoord;

void main(void)
{
	vec4 texColor;
#ifdef GL_ES_VERSION_2_0
    texColor = texture2D(uTexture, vTexCoord);
#else
    texColor = texture2D(uTexture, vTexCoord.xy, vTexCoord.z);
#endif
	gl_FragColor = vec4(uColor.rgb, texColor.r);
}
