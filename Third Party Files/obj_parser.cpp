/******************************************************************************\
| OpenGL 4 Example Code.                                                       |
| Accompanies written series "Anton's OpenGL 4 Tutorials"                      |
| Email: anton at antongerdelan dot net                                        |
| First version 7 Nov 2013                                                     |
| Copyright Dr Anton Gerdelan, Trinity College Dublin, Ireland.                |
| See individual libraries' separate legal notices                             |
|******************************************************************************|
| Anton's lazy Wavefront OBJ parser                                            |
| Anton Gerdelan 7 Nov 2013                                                    |
| Notes:                                                                       |
| I ignore MTL files                                                           |
| Mesh MUST be triangulated - quads not accepted                               |
| Mesh MUST contain vertex points, normals, and texture coordinates            |
| Faces MUST come after all other data in the .obj file                        |
\******************************************************************************/
#include "obj_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assimp/cimport.h> // C importer
#include <assimp/scene.h> // collects data
#include <assimp/postprocess.h> // various extra operations
#include <GL/glew.h>
#include "Antons_maths_funcs.h"


bool load_obj_file  (
	const char* file_name,
	float*& points,
	float*& tex_coords,
	float*& normals,
	int& point_count
) {

	float* unsorted_vp_array = NULL;
	float* unsorted_vt_array = NULL;
	float* unsorted_vn_array = NULL;
	int current_unsorted_vp = 0;
	int current_unsorted_vt = 0;
	int current_unsorted_vn = 0;

	FILE* fp = fopen (file_name, "r");
	if (!fp) {
		fprintf (stderr, "ERROR: could not find file %s\n", file_name);
		return false;
	}
	
	// first count points in file so we know how much mem to allocate
	point_count = 0;
	int unsorted_vp_count = 0;
	int unsorted_vt_count = 0;
	int unsorted_vn_count = 0;
	int face_count = 0;
	char line[1024];
	while (fgets (line, 1024, fp)) {
		if (line[0] == 'v') {
			if (line[1] == ' ') {
				unsorted_vp_count++;
			} else if (line[1] == 't') {
				unsorted_vt_count++;
			} else if (line[1] == 'n') {
				unsorted_vn_count++;
			}
		} else if (line[0] == 'f') {
			face_count++;
		}
	}
	printf (
		"found %i vp %i vt %i vn unique in obj. allocating memory...\n",
		unsorted_vp_count, unsorted_vt_count, unsorted_vn_count
	);
	unsorted_vp_array = (float*)malloc (unsorted_vp_count * 3 * sizeof (float));
	unsorted_vt_array = (float*)malloc (unsorted_vt_count * 2 * sizeof (float));
	unsorted_vn_array = (float*)malloc (unsorted_vn_count * 3 * sizeof (float));
	points = (float*)malloc (3 * face_count * 3 * sizeof (float));
	tex_coords = (float*)malloc (3 * face_count * 2 * sizeof (float));
	normals = (float*)malloc (3 * face_count * 3 * sizeof (float));
	printf (
		"allocated %i bytes for mesh\n",
		(int)(3 * face_count * 8 * sizeof (float))
	);
	
	rewind (fp);
	while (fgets (line, 1024, fp)) {
		// vertex
		if (line[0] == 'v') {
		
			// vertex point
			if (line[1] == ' ') {
				float x, y, z;
				x = y = z = 0.0f;
				sscanf (line, "v %f %f %f", &x, &y, &z);
				unsorted_vp_array[current_unsorted_vp * 3] = x;
				unsorted_vp_array[current_unsorted_vp * 3 + 1] = y;
				unsorted_vp_array[current_unsorted_vp * 3 + 2] = z;
				current_unsorted_vp++;
				
			// vertex texture coordinate
			} else if (line[1] == 't') {
				float s, t;
				s = t = 0.0f;
				sscanf (line, "vt %f %f", &s, &t);
				unsorted_vt_array[current_unsorted_vt * 2] = s;
				unsorted_vt_array[current_unsorted_vt * 2 + 1] = t;
				current_unsorted_vt++;
				
			// vertex normal
			} else if (line[1] == 'n') {
				float x, y, z;
				x = y = z = 0.0f;
				sscanf (line, "vn %f %f %f", &x, &y, &z);
				unsorted_vn_array[current_unsorted_vn * 3] = x;
				unsorted_vn_array[current_unsorted_vn * 3 + 1] = y;
				unsorted_vn_array[current_unsorted_vn * 3 + 2] = z;
				current_unsorted_vn++;
			}
			
		// faces
		} else if (line[0] == 'f') {
			// work out if using quads instead of triangles and print a warning
			int slashCount = 0;
			int len = strlen (line);
			for (int i = 0; i < len; i++) {
				if (line[i] == '/') {
					slashCount++;
				}
			}
			if (slashCount != 6) {
				fprintf (
					stderr,
					"ERROR: file contains quads or does not match v vp/vt/vn layout - \
					make sure exported mesh is triangulated and contains vertex points, \
					texture coordinates, and normals\n"
				);
				return false;
			}

			int vp[3], vt[3], vn[3];
			sscanf (
				line,
				"f %i/%i/%i %i/%i/%i %i/%i/%i",
				&vp[0], &vt[0], &vn[0], &vp[1], &vt[1], &vn[1], &vp[2], &vt[2], &vn[2]
			);

			/* start reading points into a buffer. order is -1 because obj starts from
			   1, not 0 */
			// NB: assuming all indices are valid
			for (int i = 0; i < 3; i++) {
				if ((vp[i] - 1 < 0) || (vp[i] - 1 >= unsorted_vp_count)) {
					fprintf (stderr, "ERROR: invalid vertex position index in face\n");
					return false;
				}
				if ((vt[i] - 1 < 0) || (vt[i] - 1 >= unsorted_vt_count)) {
					fprintf (stderr, "ERROR: invalid texture coord index %i in face.\n", vt[i]);
					return false;
				}
				if ((vn[i] - 1 < 0) || (vn[i] - 1 >= unsorted_vn_count)) {
					printf ("ERROR: invalid vertex normal index in face\n");
					return false;
				}
				points[point_count * 3] = unsorted_vp_array[(vp[i] - 1) * 3];
				points[point_count * 3 + 1] = unsorted_vp_array[(vp[i] - 1) * 3 + 1];
				points[point_count * 3 + 2] = unsorted_vp_array[(vp[i] - 1) * 3 + 2];
				tex_coords[point_count * 2] = unsorted_vt_array[(vt[i] - 1) * 2];
				tex_coords[point_count * 2 + 1] = unsorted_vt_array[(vt[i] - 1) * 2 + 1];
				normals[point_count * 3] = unsorted_vn_array[(vn[i] - 1) * 3];
				normals[point_count * 3 + 1] = unsorted_vn_array[(vn[i] - 1) * 3 + 1];
				normals[point_count * 3 + 2] = unsorted_vn_array[(vn[i] - 1) * 3 + 2];
				point_count++;
			}
		}
	}
	fclose (fp);
	free (unsorted_vp_array);
	free (unsorted_vn_array);
	free (unsorted_vt_array);
	printf (
		"allocated %i points\n",
		point_count
	);
	return true;
}

bool loab_obj_file_with_tans(
	const char* file_name,
	float*& points,
	float*& tex_coords,
	float*& normals,
	float*& tangents,
	int& point_count
) {
	const aiScene* scene = aiImportFile(file_name, aiProcess_Triangulate |
		aiProcess_CalcTangentSpace);
	if (!scene) {
		fprintf(stderr, "ERROR: reading mesh %s\n", file_name);
		return false;
	}
	printf("  %i animations\n", scene->mNumAnimations);
	printf("  %i cameras\n", scene->mNumCameras);
	printf("  %i lights\n", scene->mNumLights);
	printf("  %i materials\n", scene->mNumMaterials);
	printf("  %i meshes\n", scene->mNumMeshes);
	printf("  %i textures\n", scene->mNumTextures);

	// get first mesh only
	const aiMesh* mesh = scene->mMeshes[0];
	printf("    %i vertices in mesh[0]\n", mesh->mNumVertices);
	point_count = mesh->mNumVertices;

	// allocate memory for vertex points
	if (mesh->HasPositions()) {
		printf("mesh has positions\n");
		points = (GLfloat*)malloc(point_count * 3 * sizeof(GLfloat));
	}
	if (mesh->HasNormals()) {
		printf("mesh has normals\n");
		normals = (GLfloat*)malloc(point_count * 3 * sizeof(GLfloat));
	}
	if (mesh->HasTextureCoords(0)) {
		printf("mesh has texture coords\n");
		tex_coords = (GLfloat*)malloc(point_count * 2 * sizeof(GLfloat));
	}
	if (mesh->HasTangentsAndBitangents()) {
		printf("mesh has tangents\n");
		tangents = (GLfloat*)malloc(point_count * 4 * sizeof(GLfloat));
	}

	for (unsigned int v_i = 0; v_i < mesh->mNumVertices; v_i++) {
		if (mesh->HasPositions()) {
			const aiVector3D* vp = &(mesh->mVertices[v_i]);
			points[v_i * 3] = (GLfloat)vp->x;
			points[v_i * 3 + 1] = (GLfloat)vp->y;
			points[v_i * 3 + 2] = (GLfloat)vp->z;
		}
		if (mesh->HasNormals()) {
			const aiVector3D* vn = &(mesh->mNormals[v_i]);
			normals[v_i * 3] = (GLfloat)vn->x;
			normals[v_i * 3 + 1] = (GLfloat)vn->y;
			normals[v_i * 3 + 2] = (GLfloat)vn->z;
		}
		if (mesh->HasTextureCoords(0)) {
			const aiVector3D* vt = &(mesh->mTextureCoords[0][v_i]);
			tex_coords[v_i * 2] = (GLfloat)vt->x;
			tex_coords[v_i * 2 + 1] = (GLfloat)vt->y;
		}
		if (mesh->HasTangentsAndBitangents()) {
			const aiVector3D* tangent = &(mesh->mTangents[v_i]);
			const aiVector3D* bitangent = &(mesh->mBitangents[v_i]);
			const aiVector3D* normal = &(mesh->mNormals[v_i]);

			// put the three vectors into my vec3 struct format for doing maths
			vec3 t(tangent->x, tangent->y, tangent->z);
			vec3 n(normal->x, normal->y, normal->z);
			vec3 b(bitangent->x, bitangent->y, bitangent->z);
			// orthogonalise and normalise the tangent so we can use it in something
			// approximating a T,N,B inverse matrix
			vec3 t_i = normalise(t - n * dot(n, t));

			// get determinant of T,B,N 3x3 matrix by dot*cross method
			float det = (dot(cross(n, t), b));
			if (det < 0.0f) {
				det = -1.0f;
			}
			else {
				det = 1.0f;
			}

			// push back 4d vector for inverse tangent with determinant
			tangents[v_i * 4] = t_i.v[0];
			tangents[v_i * 4 + 1] = t_i.v[1];
			tangents[v_i * 4 + 2] = t_i.v[2];
			tangents[v_i * 4 + 3] = det;
		}
	}

	aiReleaseImport(scene);

	printf("mesh loaded\n");

	return true;
}
