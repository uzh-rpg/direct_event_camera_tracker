/*
 * This file is from the LearnOpenGL tutorial from Joey de Vries
 * See https://learnopengl.com/
 *
 * License: CC BY-NC 4.0
 *
 * This file: https://github.com/JoeyDeVries/LearnOpenGL/blob/master/includes/learnopengl/mesh.h
 *
 * Some modifications by Samuel Bryner.
 */

#ifndef MESH_H
#define MESH_H

#include <vector>
#include <iostream>

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QString>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

////////////////////////////////////////////////////////////////////////////////

struct Vertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
    glm::vec2 TexCoords;
    // tangent
    glm::vec3 Tangent;
    // bitangent
    glm::vec3 Bitangent;
    // color
    glm::vec3 Color;
};

////////////////////////////////////////////////////////////////////////////////

struct Texture
{
    unsigned int id;
    std::string type;
    std::string path;
};

////////////////////////////////////////////////////////////////////////////////

class Mesh : protected QOpenGLFunctions
{
public:
    /*  Mesh Data  */
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;
    QOpenGLVertexArrayObject VAO; // vertex array (must be a pointer as object is non-copyable for unknown reasons or prevent copying & moving a Mesh -.-)

    /*  Functions  */
    // constructor
    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const std::vector<Texture>& textures);

    // destructor
    ~Mesh();


    // render the mesh
    void draw(QOpenGLShaderProgram& shader);

private:

    // disallow copies of this object (VAO cannot be copied)
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    // also disallow moving
    Mesh(Mesh&& m) = delete;

    /*  Functions  */
    // initializes all the buffer objects/arrays
    void setupMesh();

    /*  Render data  */
    QOpenGLBuffer VBO; // vertex buffer
    QOpenGLBuffer EBO; // vertex buffer
};

////////////////////////////////////////////////////////////////////////////////

#endif
