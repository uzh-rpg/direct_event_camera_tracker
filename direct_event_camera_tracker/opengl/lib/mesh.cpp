#include "mesh.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const std::vector<Texture>& textures)
    : vertices(vertices), indices(indices), textures(textures),
    VBO(QOpenGLBuffer::VertexBuffer), EBO(QOpenGLBuffer::IndexBuffer)
{
    // now that we have all the required data, set the vertex buffers and its attribute pointers.
    setupMesh();
}

////////////////////////////////////////////////////////////////////////////////

Mesh::~Mesh()
{
}

////////////////////////////////////////////////////////////////////////////////

// render the mesh
void Mesh::draw(QOpenGLShaderProgram& shader)
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // bind appropriate textures
    unsigned int diffuseNr  = 1;
    unsigned int specularNr = 1;
    unsigned int normalNr   = 1;
    unsigned int heightNr   = 1;
    for(unsigned int i = 0; i < textures.size(); i++)
    {
        f->glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
        // retrieve texture number (the N in diffuse_textureN)
        unsigned int number;
        QString name = QString::fromStdString(textures[i].type);
        if(name == "texture_diffuse")
            number = diffuseNr++;
        else if(name == "texture_specular")
            number = specularNr++;
        else if(name == "texture_normal")
            number = normalNr++;
        else if(name == "texture_height")
            number = heightNr++;
        else
            throw "Invalid texture type.";

        // now set the sampler to the correct texture unit
        shader.setUniformValue(shader.uniformLocation(name + QString::number(number)), i);

        // and finally bind the texture
        f->glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }

    // draw mesh
    VAO.bind();
    f->glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    VAO.release();

    // always good practice to set everything back to defaults once configured.
    f->glActiveTexture(GL_TEXTURE0);
}

////////////////////////////////////////////////////////////////////////////////

// initializes all the buffer objects/arrays
void Mesh::setupMesh()
{
    if (textures.empty()) {
        cerr << "WARNING: Mesh has no textures associated!" << endl;
    }

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // create buffers/arrays
    if (!VAO.create()) {
        cerr << "ERROR: Failed to create vertex array object" << endl;
    }
    QOpenGLVertexArrayObject::Binder vao_binder(&VAO);

    VBO.create(); VBO.bind(); VBO.setUsagePattern(QOpenGLBuffer::StaticDraw);

    // A great thing about structs is that their memory layout is sequential
    // for all its items.  The effect is that we can simply pass a pointer to
    // the struct and it translates perfectly to a glm::vec3/2 array which
    // again translates to 3/2 floats which translates to a byte array.
    VBO.allocate(&vertices[0], vertices.size() * sizeof(Vertex));


    EBO.create(); EBO.bind(); EBO.setUsagePattern(QOpenGLBuffer::StaticDraw);

    EBO.allocate(&indices[0], indices.size() * sizeof(unsigned int));

    // set the vertex attribute pointers
    // vertex Positions
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    // vertex normals
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
    // vertex texture coords
    f->glEnableVertexAttribArray(2);
    f->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
    // vertex tangent
    f->glEnableVertexAttribArray(3);
    f->glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Tangent));
    // vertex bitangent
    f->glEnableVertexAttribArray(4);
    f->glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));
    // vertex color
    f->glEnableVertexAttribArray(5);
    f->glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Color));
}

////////////////////////////////////////////////////////////////////////////////

