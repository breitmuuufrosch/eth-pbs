#pragma once

static const char* vertBumpMap = {
	"attribute vec3 tangent;\n"
	"attribute vec3 binormal;\n"
	"varying vec3 lightDir;\n"
	"void main()\n"
	"{\n"
	"    vec3 normal = normalize(gl_NormalMatrix * gl_Normal);\n"
	"    mat3 rotation = mat3(tangent, binormal, normal);\n"
	"    vec4 vertexInEye = gl_ModelViewMatrix * gl_Vertex;\n"
	"    lightDir = vec3(gl_LightSource[0].position.xyz - vertexInEye.xyz);\n"
	"    lightDir = normalize(rotation * normalize(lightDir));\n"
	"    gl_Position = ftransform();\n"
	"    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
	"}\n"
};

static const char* fragBumpMap = {
	"uniform sampler2D colorTex;\n"
	"uniform sampler2D normalTex;\n"
	"varying vec3 lightDir;\n"
	"void main (void)\n"
	"{\n"
	"    vec4 base = texture2D(colorTex, gl_TexCoord[0].xy);\n"
	"    vec3 bump = texture2D(normalTex, gl_TexCoord[0].xy).xyz;\n"
	"    bump = normalize(bump * 2.0 - 1.0);\n"

	"    float lambert = max(dot(bump, lightDir), 0.0);\n"
	"    if (lambert > 0.0)\n"
	"    {\n"
	"        gl_FragColor = base * gl_LightSource[0].diffuse * lambert;\n"
	"        gl_FragColor += gl_LightSource[0].specular * pow(lambert, 2.0);\n"
	"    }\n"
	"    gl_FragColor += gl_LightSource[0].ambient;\n"
	"}\n"
};