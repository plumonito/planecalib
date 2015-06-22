uniform vec2 uPP;
uniform float uLambda;
uniform vec2 uFocal;
uniform mat4 uRt;

attribute vec4 aPosCoord;
attribute vec4 aColor;
uniform mat4 uMVPMatrix;

varying vec4 vColor;

void main(void)
{
	vColor = aColor;
	
	//To camera coordinates
	vec4 xc = uRt*aPosCoord;
	
	//Normalize
	vec2 xn = xc.xy / xc.z;
	
	//Focal
	xn.x *= uFocal.x;
	xn.y *= uFocal.y;
	
	//Distort
	float r2 = dot(xn,xn);
	float r4 = r2*r2;
	float factor = 1.0f/(1.0f + uLambda*r2);
	vec2 xd = factor*xn;
	
	//PP
	xn += uPP;
	
	//Add depth again
	vec3 uvz = xc.z * vec3(xn,1);
	
	//To vec4
	vec4 uv4 = vec4(uvz,1);
	
	//To gl coordinates
	gl_Position = uMVPMatrix * uv4;
}
