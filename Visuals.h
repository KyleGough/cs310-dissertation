#ifndef VISUALS_H
#define VISUALS_H

//Light Struct.
struct Light {
	size_t name;
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float position[4];
};

//Material Struct.
struct Material {
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float shininess;
};

//Global Light Source.
const Light globalLight = {
	GL_LIGHT0,
	{0.3f, 0.3f, 0.3f, 1.0f},
	{2.0f, 2.0f, 2.0f, 1.0f},
	{0.5f, 0.5f, 0.5f, 1.0f},
	{120.0f, 90.0f, 50.0f, 1.0f}
};

//Global Material.
const Material globalMaterial = {
	{0.02f, 0.02f, 0.02f, 1.0f},
  {0.01f, 0.01f, 0.01f, 1.0f},
	{0.4f, 0.4f, 0.4f, 1.0f},
	0.078125f
};

//Colours.
float caveFaceColour[4] = {0.2f, 0.1f, 0.0f, 1.0f};
float caveDepthColour[4] = {0.2f, 0.1f, 0.05f, 1.0f};

//Sets the properties of a given material.
void setMaterial(const Material& material) {
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material.ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material.diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material.specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);
}

//Sets the properties of a given light.
void setLight(const Light& light) {
	glLightfv(light.name, GL_AMBIENT, light.ambient);
	glLightfv(light.name, GL_DIFFUSE, light.diffuse);
	glLightfv(light.name, GL_SPECULAR, light.specular);
	glLightfv(light.name, GL_POSITION, light.position);
}

#endif
