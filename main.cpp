//
// template-rt.cpp
//

#define _CRT_SECURE_NO_WARNINGS
#include "matm.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm> 
using namespace std;

// -------------------------------------------------------------------
// Ray struct

struct Ray
{
    vec3 origin;		// origin of the ray
    vec3 dir;			// direction of the ray
};


// -------------------------------------------------------------------
// Sphere struct

struct Sphere
{	
	vec3 center;
	float radius;		
	vec3 ka, kd, ks;	// ambient, diffuse and specular reflecction constant in Phong's reflection model
	vec3 reflectivity;	// control how much light is received from reflection in recursive ray-tracing (e.g. 0.1)
	float alpha;		// control size of specular highlight (e.g. 20)

	// default constructor
	Sphere(const vec3& ic=vec3(0.0f), const float& ir=0.0f, const vec3& ika=vec3(0.0f), const vec3& ikd=vec3(0.0f), const vec3& iks=vec3(0.0f), const float& ireflectivity=0.1f, const float& ialpha=1.0f):
	center(ic), radius(ir), ka(ika), kd(ikd), ks(iks), reflectivity(ireflectivity), alpha(ialpha)
	{}

	bool intersect(const Ray& ray, float& t0, float& t1);
};

static int numOfLoops = 0;

bool solveQuad(const float &a, const float &b, const float &c, float &t0, float &t1)
{
	float discriminant = (b*b) - (4 * (a*c));

	if (discriminant < 0)
	{
		return false;
	}
	else if (discriminant == 0)
	{
		t0 = -0.5*(b / a);
		t1 = t0;
	}
	else
	{
		float q = (b > 0) ?
			-0.5 * (b + sqrtf(discriminant)) :
			-0.5 * (b - sqrtf(discriminant));
		t0 = q / a;
		t1 = c / q;
	}

	if (t0 > t1)
	{
		swap(t0, t1);
	}

	return true;
}

float t = 0;

// TODO:
// return true if the input ray intersects with the sphere; otherwise return false
// return by reference the intersections. t0 refers to closer intersection, t1 refers to farther intersection
bool Sphere::intersect(const Ray& ray, float& t0, float& t1)
{
	vec3 length;
	length.x = center.x - ray.origin.x;
	length.y = center.y - ray.origin.y;
	length.z = center.z - ray.origin.z;

	
	float tCA = dot(length, ray.dir);
	
	if (tCA < 0)
	{
		return false;
	}
	

	float lengthDot = dot(length, length);

	float d = sqrtf(lengthDot - (tCA*tCA));


	//if d is greater than radius than the ray misses the sphere by overshooting it
	if (d < 0 || d > radius)
	{
		return false;
	}

	float tHC = sqrtf((radius*radius) - (d*d));


	t0 = tCA - tHC;
	t1 = tCA + tHC;

	
	float a = (ray.dir.x*ray.dir.x) + (ray.dir.y*ray.dir.y) + (ray.dir.z*ray.dir.z);
	float b = (2 * ray.dir.x*(ray.origin.x - center.x)) + (2 * ray.dir.y*(ray.origin.y - center.y)) + (2 * ray.dir.z*(ray.origin.z - center.z));
	float c = (center.x*center.x) + (center.y*center.y) + (center.z*center.z) + (ray.origin.x*ray.origin.x) + (ray.origin.y*ray.origin.y) + (ray.origin.z*ray.origin.z)
			  - 2*((center.x*ray.origin.x) + (center.y*ray.origin.y) + (center.z*ray.origin.z)) - (radius*radius);

	if (!solveQuad(a, b, c, t0, t1))
	{
		return false;
	}

	if (t0 > t1)
	{
		swap(t0, t1);
	}

	if (t0 < 0)
	{
		t0 = t1;
		if (t0 < 0)
		{
			return false;
		}
	}

	t = ((-b - sqrtf((b*b) - 4 * (a*c))) / (2 * a));
	
	return true;		// this should be replaced by code to determine intersection with a sphere
};


// -------------------------------------------------------------------
// Light Structs

struct AmbientLight
{
	vec3 ia;		// ambient intensity (a vec3 of 0.0 to 1.0, each entry in vector refers to R,G,B channel)

	// default constructor
	AmbientLight(const vec3& iia=vec3(0.0f)):
	ia(iia)
	{}

};


struct PointLight
{
	vec3 location;	// location of point light
	vec3 id, is;	// diffuse intensity, specular intensity (vec3 of 0.0 to 1.0)
	
	// default constructor
	PointLight(const vec3& iloc=vec3(0.0f), const vec3& iid=vec3(0.0f), const vec3& iis=vec3(0.0f)):
	location(iloc), id(iid), is(iis)
	{}

};


// -------------------------------------------------------------------
// Our Scene

// lights and spheres in our scene
AmbientLight my_ambient_light;			// our ambient light
vector<PointLight> my_point_lights;		// a vector containing all our point lights
vector<Sphere> my_spheres;				// a vector containing all our spheres


// this stores the color of each pixel, which will take the ray-tracing results
vector<vec3> g_colors;	

int recursion_lvl_max = 2;

// this defines the screen
int g_width = 640;				//number of pixels
int g_height = 480;				// "g_" refers to coord in 2D image space
float fov = 30;					// field of view (in degree)

float invWidth = 1 / float(g_width);
float invHeight = 1 / float(g_height);
float aspectratio = g_width / float(g_height);
float angle = tan(M_PI * 0.5 * fov / float(180));


// -------------------------------------------------------------------
// Utilities

void setColor(int ix, int iy, const vec3& color)
{
    int iy2 = g_height - iy - 1; // Invert iy coordinate.
    g_colors[iy2 * g_width + ix] = color;
}



// -------------------------------------------------------------------
// Ray tracing


vec3 scalarVector(vec3 vec1, vec3 vec2)
{
	vec3 newVec;

	newVec.x = vec1.x*vec2.x;
	newVec.y = vec1.y*vec2.y;
	newVec.z = vec1.z*vec2.z;

	return newVec;
}

int whiteness = 0;

vec3 trace(const Ray& ray, int recursion_lvl)
{
	float inf = 99999;
	float t_min = inf;
	int near_sphere_idx;
	bool has_intersection = false;

	for (int i=0; i<my_spheres.size(); ++i)
	{
		float t0 = inf;		//some large value
		float t1 = inf;

		// check intersection with sphere
		if (my_spheres[i].intersect(ray, t0, t1))
		{
			has_intersection = true;

			if (t0 < t_min)
			{
				t_min = t0;
				near_sphere_idx = i;
			}
		}	
	}

	numOfLoops++;

	if (has_intersection == false)
	{
		// just return background color (black)
		return vec3(0.0f, 0.0f, 0.0f);
	}

	Sphere my_sphere = my_spheres[near_sphere_idx];

	vec3 ambientVal;

	ambientVal = scalarVector(my_sphere.ka, my_ambient_light.ia);

	vec3 hitPoint;
	
	if (t > 0)
	{
		hitPoint = ray.origin + ray.dir*t;
	}

	vec3 N, L, R, V;

	int itert = 0;

	for(PointLight light : my_point_lights)
	{

		N = ((hitPoint - my_sphere.center) / my_sphere.radius);
		N = normalize(N);
	
		L = light.location - hitPoint;
		L = normalize(L);

		R = L - (2 * dot(L, N)*N);
		R = normalize(R);

		V.x = ray.origin.x - hitPoint.x;
		V.y = ray.origin.y - hitPoint.y;
		V.z = ray.origin.z - hitPoint.z;
		V = normalize(V);

		
		ambientVal += light.id * (my_sphere.kd*(max(0.0f, (dot(L, N)))) + (my_sphere.ks*(max(0.0f, (powf(dot(R, V), my_sphere.alpha))))));

	}


	vec3 color(ambientVal.x, ambientVal.y, ambientVal.z);

	// TODO: implement Phong's relection model
	//vec3 color(1.0,1.0,1.0);	// this code should be replaced by codes of the Phong's reflection model (i.e. color should be determined by Phong's model)

	
	if (recursion_lvl >0)
	{
		// TODO: implement recursive ray-tracing here, to add contribution of light reflected from other objects

		
		Ray newRay;
		newRay.dir.x = ray.dir.x - 2 * dot(N, ray.dir)*N.x;
		newRay.dir.y = ray.dir.y - 2 * dot(N, ray.dir)*N.y;
		newRay.dir.z = ray.dir.z - 2 * dot(N, ray.dir)*N.z;
		newRay.origin = normalize(hitPoint);
		
		vec3 normalHP = normalize(hitPoint);

		recursion_lvl -= 1;
		
		
		Ray recRay;
		recRay.dir = newRay.dir;
		recRay.origin = hitPoint;

		color += 0.2*trace(recRay, recursion_lvl);
		
		return color;		// this should be replaced by codes to do recursive ray-tracing
	}
	else 
	{
		return color;
	}			
}


vec3 getDir(int ix, int iy)
{
    // This should return the direction from the origin
    // to pixel (ix, iy), normalized!

	vec3 dir;
	dir.x = (2 * ((ix + 0.5) * invWidth) - 1) * angle * aspectratio;
	dir.y = (2 * ((iy + 0.5) * invHeight) - 1) * angle;
	dir.z = -1;

	return dir;
}

void renderPixel(int ix, int iy)
{
    Ray ray;
    ray.origin = vec3(0.0f, 0.0f, 0.0f);
    ray.dir = getDir(ix, iy);
    vec3 color = trace(ray, recursion_lvl_max);
    setColor(ix, iy, color);
}

void render()
{
    for (int iy = 0; iy < g_height; iy++)
        for (int ix = 0; ix < g_width; ix++)
            renderPixel(ix, iy);
}


// -------------------------------------------------------------------
// PPM saving

void savePPM(int Width, int Height, char* fname, unsigned char* pixels) 
{
    FILE *fp;
    const int maxVal=255;

    printf("Saving image %s: %d x %d\n", fname, Width, Height);
    fp = fopen(fname,"wb");
    if (!fp) {
        printf("Unable to open file '%s'\n", fname);
        return;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);

    for(int j = 0; j < Height; j++) {
        fwrite(&pixels[j*Width*3], 3, Width, fp);
    }

    fclose(fp);
}

void saveFile()
{
    // Convert color components from floats to unsigned chars.
    // clamp values if out of range.
    unsigned char* buf = new unsigned char[g_width * g_height * 3];
    for (int y = 0; y < g_height; y++)
        for (int x = 0; x < g_width; x++)
            for (int i = 0; i < 3; i++)
                buf[y*g_width*3+x*3+i] = (unsigned char)(((float*)g_colors[y*g_width+x])[i] * 255.9f);
    
    // change file name based on input file name.
    savePPM(g_width, g_height, "output.ppm", buf);
    delete[] buf;
}


// -------------------------------------------------------------------
// Main

int main(int argc, char* argv[])
{
	// setup pixel array
    g_colors.resize(g_width * g_height);

	// setup our scene...

	// setup ambient light
	my_ambient_light = AmbientLight(vec3(0.1));

	// setup point lights
	my_point_lights.push_back(
								PointLight(vec3(3,3,0), vec3(0.5, 0.5, 0.5), vec3(0.5,0.5,0.5))
								);

	my_point_lights.push_back(
								PointLight(vec3(-3,-3,0), vec3(0.1, 0.1, 0.1), vec3(0.1,0.1,0.1))
								);

	// setup spheres
	my_spheres.push_back(
							Sphere(vec3(0,0,-10), 1.0, vec3(0.1,0.1,0.1), vec3(0.5,0.5,0.5), vec3(0.5,0.5,0.5), 0.2, 100.0)
							);

	my_spheres.push_back(
							Sphere(vec3(-1.5,0.5,-8), 0.5, vec3(0.0,1.0,0.0), vec3(0.0,1.0,0.0), vec3(0.5,0.5,0.5), 0.0, 10.0)
							);

    render();
    saveFile();
	return 0;
}

