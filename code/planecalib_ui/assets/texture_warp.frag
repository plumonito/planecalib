#ifdef GL_ES_VERSION_2_0
precision mediump float;
#endif

uniform sampler2D uTexture;
varying vec3 vTexCoord;

void main(void)
{
#ifdef GL_ES_VERSION_2_0
    gl_FragColor = texture2D(uTexture, vTexCoord);
#else
    gl_FragColor = texture2D(uTexture, vTexCoord.xy, vTexCoord.z);
#endif
}
