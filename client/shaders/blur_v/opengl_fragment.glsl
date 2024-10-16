#define rendered texture0

uniform sampler2D rendered;
uniform vec2 texelSize0;
uniform mediump float bloomRadius = 3.0;

#ifdef GL_ES
varying mediump vec2 varTexCoord;
#else
centroid varying vec2 varTexCoord;
#endif

void main(void)
{
	// kernel distance and linear size
	mediump float n = 2. * bloomRadius + 1.;

	vec2 uv = varTexCoord.st - vec2(0., bloomRadius * texelSize0.y);
	vec4 color = vec4(0.);
	mediump float sum = 0.;
	for (mediump float i = 0.; i < n; i++) {
		mediump float weight = pow(1. - (abs(i / bloomRadius - 1.)), 1.3);
		color += texture2D(rendered, uv).rgba * weight;
		sum += weight;
		uv += vec2(0., texelSize0.y);
	}
	color /= sum;
	gl_FragColor = vec4(color.rgb, 1.0); // force full alpha to avoid holes in the image.
}
