#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <set>
#include <map>

#include "tinyxml2.h"
#include "Triangle.h"
#include "Helpers.h"
#include "Scene.h"
#include "Vec4WithColor.h"
#include "Vec4.h"

using namespace tinyxml2;
using namespace std;

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *xmlElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *rootNode = xmlDoc.FirstChild();

	// read background color
	xmlElement = rootNode->FirstChildElement("BackgroundColor");
	str = xmlElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	xmlElement = rootNode->FirstChildElement("Culling");
	if (xmlElement != NULL)
	{
		str = xmlElement->GetText();

		if (strcmp(str, "enabled") == 0)
		{
			this->cullingEnabled = true;
		}
		else
		{
			this->cullingEnabled = false;
		}
	}

	// read cameras
	xmlElement = rootNode->FirstChildElement("Cameras");
	XMLElement *camElement = xmlElement->FirstChildElement("Camera");
	XMLElement *camFieldElement;
	while (camElement != NULL)
	{
		Camera *camera = new Camera();

		camElement->QueryIntAttribute("id", &camera->cameraId);

		// read projection type
		str = camElement->Attribute("type");

		if (strcmp(str, "orthographic") == 0)
		{
			camera->projectionType = ORTOGRAPHIC_PROJECTION;
		}
		else
		{
			camera->projectionType = PERSPECTIVE_PROJECTION;
		}

		camFieldElement = camElement->FirstChildElement("Position");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->position.x, &camera->position.y, &camera->position.z);

		camFieldElement = camElement->FirstChildElement("Gaze");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->gaze.x, &camera->gaze.y, &camera->gaze.z);

		camFieldElement = camElement->FirstChildElement("Up");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->v.x, &camera->v.y, &camera->v.z);

		// Normalize gaze direction
		camera->gaze = normalizeVec3(camera->gaze);

		// w = -gaze (camera looks down -z)
		camera->w = inverseVec3(camera->gaze);

		// u = up × w    (right vector)
		camera->u = crossProductVec3(camera->v, camera->w);
		camera->u = normalizeVec3(camera->u);

		// v = w × u    (true up vector)
		camera->v = crossProductVec3(camera->w, camera->u);
		camera->v = normalizeVec3(camera->v);

		camFieldElement = camElement->FirstChildElement("ImagePlane");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &camera->left, &camera->right, &camera->bottom, &camera->top,
			   &camera->near, &camera->far, &camera->horRes, &camera->verRes);

		camFieldElement = camElement->FirstChildElement("OutputName");
		str = camFieldElement->GetText();
		camera->outputFilename = string(str);

		this->cameras.push_back(camera);

		camElement = camElement->NextSiblingElement("Camera");
	}

	// read vertices
	xmlElement = rootNode->FirstChildElement("Vertices");
	XMLElement *vertexElement = xmlElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (vertexElement != NULL)
	{
		Vec3WithColor *vertex = new Vec3WithColor();
		vertex->vertexId = vertexId;

		str = vertexElement->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = vertexElement->Attribute("color");
		sscanf(str, "%lf %lf %lf", &vertex->color.r, &vertex->color.g, &vertex->color.b);

		this->vertices.push_back(vertex);

		vertexElement = vertexElement->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	xmlElement = rootNode->FirstChildElement("Translations");
	XMLElement *translationElement = xmlElement->FirstChildElement("Translation");
	while (translationElement != NULL)
	{
		Translation *translation = new Translation();

		translationElement->QueryIntAttribute("id", &translation->translationId);

		str = translationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		this->translations.push_back(translation);

		translationElement = translationElement->NextSiblingElement("Translation");
	}

	// read scalings
	xmlElement = rootNode->FirstChildElement("Scalings");
	XMLElement *scalingElement = xmlElement->FirstChildElement("Scaling");
	while (scalingElement != NULL)
	{
		Scaling *scaling = new Scaling();

		scalingElement->QueryIntAttribute("id", &scaling->scalingId);
		str = scalingElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		this->scalings.push_back(scaling);

		scalingElement = scalingElement->NextSiblingElement("Scaling");
	}

	// read rotations
	xmlElement = rootNode->FirstChildElement("Rotations");
	XMLElement *rotationElement = xmlElement->FirstChildElement("Rotation");
	while (rotationElement != NULL)
	{
		Rotation *rotation = new Rotation();

		rotationElement->QueryIntAttribute("id", &rotation->rotationId);
		str = rotationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		this->rotations.push_back(rotation);

		rotationElement = rotationElement->NextSiblingElement("Rotation");
	}

	// read meshes
	xmlElement = rootNode->FirstChildElement("Meshes");

	XMLElement *meshElement = xmlElement->FirstChildElement("Mesh");
	while (meshElement != NULL)
	{
		Mesh *mesh = new Mesh();

		meshElement->QueryIntAttribute("id", &mesh->meshId);

		// read mesh faces
		char *row;
		char *cloneStr;
		int vertexId1, vertexId2, vertexId3;
		str = meshElement->GetText();
		cloneStr = strdup(str);

		row = strtok(cloneStr, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &vertexId1, &vertexId2, &vertexId3);

			if (result != EOF)
			{
				Vec3WithColor v1 = *(this->vertices[vertexId1 - 1]);
				Vec3WithColor v2 = *(this->vertices[vertexId2 - 1]);
				Vec3WithColor v3 = *(this->vertices[vertexId3 - 1]);

				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		this->meshes.push_back(mesh);

		meshElement = meshElement->NextSiblingElement("Mesh");
	}

	// read instances
	xmlElement = rootNode->FirstChildElement("Instances");

	XMLElement *instanceElement = xmlElement->FirstChildElement("Instance");
	while (instanceElement != NULL)
	{
		Instance *instance = new Instance();
		int meshId;

		instanceElement->QueryIntAttribute("id", &instance->instanceId);
		instanceElement->QueryIntAttribute("meshId", &meshId);

		instance->mesh = *(this->meshes[meshId - 1]);

		// read projection type
		str = instanceElement->Attribute("type");

		if (strcmp(str, "wireframe") == 0)
		{
			instance->instanceType = WIREFRAME_INSTANCE;
		}
		else
		{
			instance->instanceType = SOLID_INSTANCE;
		}

		// read instance transformations
		XMLElement *instanceTransformationsElement = instanceElement->FirstChildElement("Transformations");
		XMLElement *instanceTransformationElement = instanceTransformationsElement->FirstChildElement("Transformation");

		while (instanceTransformationElement != NULL)
		{
			char transformationType;
			int transformationId;

			str = instanceTransformationElement->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			instance->transformationTypes.push_back(transformationType);
			instance->transformationIds.push_back(transformationId);

			instanceTransformationElement = instanceTransformationElement->NextSiblingElement("Transformation");
		}

		instance->numberOfTransformations = instance->transformationIds.size();
		this->instances.push_back(instance);

		instanceElement = instanceElement->NextSiblingElement("Instance");
	}
}

void Scene::assignColorToPixel(int x, int y, Color c)
{
	image[y][x] = c;
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	image.clear();
	depth.clear();

	// Allocate image as image[y][x]
	image.resize(camera->verRes, vector<Color>(camera->horRes, backgroundColor));
	depth.resize(camera->verRes, vector<double>(camera->horRes, 1.01));
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (vector<vector<Color>>) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout(camera->outputFilename.c_str());

	fout << "P3\n";
	fout << "# " << camera->outputFilename << "\n";
	fout << camera->horRes << " " << camera->verRes << "\n";
	fout << "255\n";

	// Output from top to bottom, y = verRes-1 down to 0
	for (int y = camera->verRes - 1; y >= 0; y--)
	{
		for (int x = 0; x < camera->horRes; x++)
		{
			Color &p = image[y][x];
			fout << makeBetweenZeroAnd255(p.r) << " "
				 << makeBetweenZeroAnd255(p.g) << " "
				 << makeBetweenZeroAnd255(p.b) << " ";
		}
		fout << "\n";
	}
}

/*
	Transformations, clipping, culling, rasterization are done here.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// Initialize image and depth buffer
	initializeImage(camera);

	// Precompute camera-related matrices
	Matrix4 viewportMatrix = getViewportMatrix(camera);
	Matrix4 projectionMatrix = getProjectionMatrix(camera);
	Matrix4 cameraMatrix = getCameraTransformMatrix(camera);

	// For each instance, transform its mesh triangles and rasterize
	for (size_t idx = 0; idx < instances.size(); idx++)
	{
		Instance *current_instance = instances[idx];
		Mesh current_mesh = current_instance->mesh;

		// Model (instance) transformation
		Matrix4 modelMatrix = CreateTransformationMatrix(current_instance);

		for (int t = 0; t < current_mesh.numberOfTriangles; t++)
		{
			Triangle tri = current_mesh.triangles[t];

			// Convert triangle vertices to homogeneous Vec4WithColor (t=1)
			Vec4WithColor v1(tri.v1.x, tri.v1.y, tri.v1.z, 1.0, tri.v1.color);
			Vec4WithColor v2(tri.v2.x, tri.v2.y, tri.v2.z, 1.0, tri.v2.color);
			Vec4WithColor v3(tri.v3.x, tri.v3.y, tri.v3.z, 1.0, tri.v3.color);

			// Model -> Camera space
			v1 = multiplyMatrixWithVec4WithColor(cameraMatrix, multiplyMatrixWithVec4WithColor(modelMatrix, v1));
			v2 = multiplyMatrixWithVec4WithColor(cameraMatrix, multiplyMatrixWithVec4WithColor(modelMatrix, v2));
			v3 = multiplyMatrixWithVec4WithColor(cameraMatrix, multiplyMatrixWithVec4WithColor(modelMatrix, v3));

			// Back-face culling (in camera space)
			if (this->cullingEnabled)
			{
				Vec3 a = subtractVec3(Vec3(v2.x, v2.y, v2.z), Vec3(v1.x, v1.y, v1.z));
				Vec3 b = subtractVec3(Vec3(v3.x, v3.y, v3.z), Vec3(v1.x, v1.y, v1.z));
				Vec3 normal = crossProductVec3(a, b);

				// In camera space Z decreases toward camera (z = -1 is forward).
				// A face is visible if normal.z > 0 (pointing towards camera).
				if (normal.z < 0)
					continue;
			} // Projection
			v1 = multiplyMatrixWithVec4WithColor(projectionMatrix, v1);
			v2 = multiplyMatrixWithVec4WithColor(projectionMatrix, v2);
			v3 = multiplyMatrixWithVec4WithColor(projectionMatrix, v3);

			// Perspective divide (homogeneous normalization)
			if (v1.t != 0.0)
			{
				v1.x /= v1.t;
				v1.y /= v1.t;
				v1.z /= v1.t;
				v1.t = 1.0;
			}
			if (v2.t != 0.0)
			{
				v2.x /= v2.t;
				v2.y /= v2.t;
				v2.z /= v2.t;
				v2.t = 1.0;
			}
			if (v3.t != 0.0)
			{
				v3.x /= v3.t;
				v3.y /= v3.t;
				v3.z /= v3.t;
				v3.t = 1.0;
			}

			// Viewport transform -> screen coordinates
			Vec4WithColor s1 = multiplyMatrixWithVec4WithColor(viewportMatrix, v1);
			Vec4WithColor s2 = multiplyMatrixWithVec4WithColor(viewportMatrix, v2);
			Vec4WithColor s3 = multiplyMatrixWithVec4WithColor(viewportMatrix, v3);

			// Rasterize or draw wireframe depending on instance type
			if (current_instance->instanceType == WIREFRAME_INSTANCE)
			{
				drawLine(s1, s2, this->image, this->depth);
				drawLine(s2, s3, this->image, this->depth);
				drawLine(s3, s1, this->image, this->depth);
			}
			else
			{
				rasterizeTriangle(s1, s2, s3, this->image, this->depth);
			}
		}
	}

	// After processing all instances write image to file
	writeImageToPPMFile(camera);
}

Matrix4 Scene::CreateTransformationMatrix(Instance *instance)
{
	Matrix4 result = getIdentityMatrix();

	// Apply transformations IN GIVEN ORDER but as LEFT MULTIPLICATIONS
	for (int j = 0; j < instance->numberOfTransformations; j++)
	{
		char type = instance->transformationTypes[j];
		int id = instance->transformationIds[j];

		if (type == 't')
		{
			Translation *tr = translations[id - 1];
			Matrix4 T = getIdentityMatrix();
			T.values[0][3] = tr->tx;
			T.values[1][3] = tr->ty;
			T.values[2][3] = tr->tz;

			result = multiplyMatrixWithMatrix(T, result);
		}
		else if (type == 's')
		{
			Scaling *sc = scalings[id - 1];
			Matrix4 S = getIdentityMatrix();
			S.values[0][0] = sc->sx;
			S.values[1][1] = sc->sy;
			S.values[2][2] = sc->sz;

			result = multiplyMatrixWithMatrix(S, result);
		}
		else if (type == 'r')
		{
			Rotation *rot = rotations[id - 1];
			Matrix4 R = getRotationMatrix(rot->angle, rot->ux, rot->uy, rot->uz);

			result = multiplyMatrixWithMatrix(R, result);
		}
	}

	return result;
}

set<int> finduniqueids(Mesh *current_mesh)
{
	set<int> uniqueVertexIds;
	for (int j = 0; j < current_mesh->numberOfTriangles; j++)
	{
		Triangle tri = current_mesh->triangles[j];
		uniqueVertexIds.insert(tri.v1.vertexId);
		uniqueVertexIds.insert(tri.v2.vertexId);
		uniqueVertexIds.insert(tri.v3.vertexId);
	}

	return uniqueVertexIds;
}