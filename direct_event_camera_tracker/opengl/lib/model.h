/*
 * This file is from the LearnOpenGL tutorial from Joey de Vries
 * See https://learnopengl.com/
 *
 * License: CC BY-NC 4.0
 *
 * This file: https://github.com/JoeyDeVries/LearnOpenGL/blob/master/includes/learnopengl/model.h
 *
 * Some modifications by Henri Rebecq and Samuel Bryner.
 */

#ifndef MODEL_H
#define MODEL_H

#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <opencv2/imgcodecs/imgcodecs.hpp> // for loading textures

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "opengl/lib/mesh.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <memory> // for shared_ptr

unsigned int TextureFromFile(const char *path, const std::string &directory);

class Model : protected QOpenGLFunctions
{
public:
    /*  Model Data */
    std::vector<Texture> textures_loaded;    // stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<std::shared_ptr<Mesh>> meshes;
    std::string directory;

    /*  Functions   */
    // constructor, expects a filepath to a 3D model.
    Model(std::string const &path) : has_normals(true) {
        loadModel(path);
    }

    Model() : has_normals(true) {
    }

    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(std::string const &path);

    // draws the model, and thus all its meshes
    void draw(QOpenGLShaderProgram& shader);

    bool hasNormals() { return has_normals; }

private:
    /*  Functions   */

    // processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    void processNode(aiNode *node, const aiScene *scene);

    void processMesh(aiMesh *mesh, const aiScene *scene);

    // checks all material textures of a given type and loads the textures if they're not loaded yet.
    // the required info is returned as a Texture struct.
    void loadMaterialTextures(std::vector<Texture>& textures, aiMaterial *mat, aiTextureType type, std::string typeName);

    bool has_normals;
};

unsigned int TextureFromFile(const char *path, const std::string &directory);

#endif
