#include "model.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

void Model::draw(QOpenGLShaderProgram& shader)
{
    for(unsigned int i = 0; i < meshes.size(); i++)
        meshes[i]->draw(shader);
}

////////////////////////////////////////////////////////////////////////////////

// loads a model with supported ASSIMP extensions from file and stores the
// resulting meshes in the meshes vector.
void Model::loadModel(string const &path)
{
    // read file via ASSIMP
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);
    // check for errors
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
        cerr << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
        return;
    }
    // retrieve the directory path of the filepath
    directory = path.substr(0, path.find_last_of('/'));

    // process ASSIMP's root node recursively
    processNode(scene->mRootNode, scene);
}

////////////////////////////////////////////////////////////////////////////////

// processes a node in a recursive fashion. Processes each individual mesh
// located at the node and repeats this process on its children nodes (if any).
void Model::processNode(aiNode *node, const aiScene *scene)
{
    // process each mesh located at the current node
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        // the node object only contains indices to index the actual objects in the scene.
        // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMesh(mesh, scene);
    }

    // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++)
    {
        processNode(node->mChildren[i], scene);
    }

}

////////////////////////////////////////////////////////////////////////////////

void Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
    // data to fill
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    vector<Texture> textures;

    if (!mesh->HasNormals()) {
        cerr << "WARNING: Mesh does not contain normals. Deactivating shading." << endl;;
    }

    // Walk through each of the mesh's vertices
    for(unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        Vertex vertex;
        glm::vec3 vector; // we declare a placeholder vector since assimp uses
        // its own vector class that doesn't directly convert to glm's vec3 class
        // so we transfer the data to this placeholder glm::vec3 first.

        // positions
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex.Position = vector;

        // normals
        if (mesh->HasNormals()) {
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex.Normal = vector;
        } else {
            vertex.Normal = glm::vec3(0,0,0);
            has_normals = false;
        }

        // texture coordinates
        if (mesh->HasTextureCoords(0))
        {
            glm::vec2 vec;
            // a vertex can contain up to 8 different texture coordinates. We
            // thus make the assumption that we won't use models where a vertex
            // can have multiple texture coordinates so we always take the
            // first set (0).
            vec.x = mesh->mTextureCoords[0][i].x; 
            vec.y = mesh->mTextureCoords[0][i].y;
            vertex.TexCoords = vec;
        } else {
            vertex.TexCoords = glm::vec2(0.0f, 0.0f);
        }

        if (mesh->HasTangentsAndBitangents()) {
            // tangent
            vector.x = mesh->mTangents[i].x;
            vector.y = mesh->mTangents[i].y;
            vector.z = mesh->mTangents[i].z;
            vertex.Tangent = vector;

            // bitangent
            vector.x = mesh->mBitangents[i].x;
            vector.y = mesh->mBitangents[i].y;
            vector.z = mesh->mBitangents[i].z;
            vertex.Bitangent = vector;
        } else {
            vertex.Tangent   = glm::vec3(0,0,0);
            vertex.Bitangent = glm::vec3(0,0,0);
        }

        if (mesh->HasVertexColors(0)) {
            vector.x = mesh->mColors[0][i].r;
            vector.y = mesh->mColors[0][i].g;
            vector.z = mesh->mColors[0][i].b;
            vertex.Color = vector;
        } else {
            vertex.Color = glm::vec3(0,0,0);
        }

        vertices.push_back(vertex);
    }

    // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        // retrieve all indices of the face and store them in the indices vector
        for(unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }

    // process materials
    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
    // we assume a convention for sampler names in the shaders. Each diffuse texture should be named
    // as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER.
    // Same applies to other texture as the following list summarizes:
    // diffuse: texture_diffuseN
    // specular: texture_specularN
    // normal: texture_normalN

    // 1. diffuse maps
    loadMaterialTextures(textures, material, aiTextureType_DIFFUSE, "texture_diffuse");

    // 2. specular maps
    loadMaterialTextures(textures, material, aiTextureType_SPECULAR, "texture_specular");

    // 3. normal maps
    loadMaterialTextures(textures, material, aiTextureType_HEIGHT, "texture_normal");

    // 4. height maps
    loadMaterialTextures(textures, material, aiTextureType_AMBIENT, "texture_height");

    // return a mesh object created from the extracted mesh data
    meshes.push_back(shared_ptr<Mesh>(new Mesh(vertices, indices, textures)));
}

////////////////////////////////////////////////////////////////////////////////

// checks all material textures of a given type and loads the textures if they're not loaded yet.
// the required info is returned as a Texture struct.
void Model::loadMaterialTextures(vector<Texture>& textures, aiMaterial *mat, aiTextureType type, string typeName)
{
    for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);

        // check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
        bool skip = false;
        for(unsigned int j = 0; j < textures_loaded.size(); j++)
        {
            if(std::strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0)
            {
                textures.push_back(textures_loaded[j]);
                skip = true; // a texture with the same filepath has already been loaded, continue to next one. (optimization)
                break;
            }
        }

        if(!skip)
        {   // if texture hasn't been loaded already, load it
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), this->directory);
            texture.type = typeName;
            texture.path = str.C_Str();
            textures.push_back(texture);
            textures_loaded.push_back(texture);  // store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

unsigned int TextureFromFile(const char *path, const string &directory)
{
    string filename = string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    cv::Mat texture_mat = cv::imread(filename);

    int width = texture_mat.cols;
    int height = texture_mat.rows;
    int nrComponents = texture_mat.channels();

    //VLOG(5) << "height: " << height;
    //VLOG(5) << "width: " << width;
    //VLOG(5) << "num components: " << nrComponents;

    unsigned char *data = texture_mat.data;

    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;
        else
            throw "Invalid number of color components for texture";

        //use fast 4-byte alignment (default anyway) if possible
        glPixelStorei(GL_UNPACK_ALIGNMENT, (texture_mat.step & 3) ? 1 : 4);
        //set length of one complete row in data (doesn't need to equal image.cols)
        glPixelStorei(GL_UNPACK_ROW_LENGTH, texture_mat.step/texture_mat.elemSize());

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);


        // glGenerateMipmap is only available in OpenGL 3.0 and higher
        QOpenGLFunctions_3_3_Core* f = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();
        if (!f) {
            cerr << "Cannot create OpenGL 3.3 context! No mipmaps for textures." << endl;
        } else {
            f->glGenerateMipmap(GL_TEXTURE_2D);
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    else
    {
        cout << "WARNING: Texture failed to load at path: " << path << std::endl;
    }

    return textureID;
}

////////////////////////////////////////////////////////////////////////////////

