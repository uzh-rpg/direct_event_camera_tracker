#version 330 core

layout (location = 0) in vec3 pos;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 col;

// model-view-projection matrix
uniform mat4 MVP;

uniform vec3 camera_pos;

out vec3 vert_color;

void main()
{
    gl_Position = MVP * vec4(pos, 1.0);

    // render points smaller the farther away they are
    // z/w is in range [-1, 1], where large numbers mean farther away
    // however, very few points are <0 and most are >0.5
    //
    // [1, 0]: 1=far, 0=close
    float dist = (gl_Position.z / gl_Position.w);

    /*
    if (dist > 0.92) {
        gl_PointSize = 4;
    } else {
        gl_PointSize = 1;
    }
    */

    // manually chosen constants so that pointcloud looks dense
    // i.e. points should overlap slightly, but not too much
    // this is quite dependent on the actual dataset!
    // it also strongly depends on the window size
    //
    // WARNING: These parameters are important!
    // Tracking can fail if these are not correct. Play around with these
    // parameters until your pointcloud is rendered sharp but without holes
    // between the points.

    const float min_size = 0.1;
    const float max_size = 50;

    const float z = 0.92;
    const float size_at_z = 2; //1.8;
    const float scaling = 0.05;

    float s = (2-dist/z)*scaling + size_at_z-scaling;

    gl_PointSize = clamp(s, min_size, max_size);


    /*
    gl_PointSize = (3/(dist+1)-1) * (max_size-min_size)/2 + min_size;
    */

    // linear scaling
    //gl_PointSize = dist/4*(max_size-min_size) + min_size;

    //vec3 normal_cam = normalize( (MV * vec4(normal, 0)).xyz );
    //vec3 pos_cam = (MV * vec4(pos, 1.0)).xyz;



    float cosT = dot(normalize(normal), normalize(pos-camera_pos));

    if (cosT > 0) {
        //vert_color = vec3(1,0,0);
        gl_Position = vec4(2.0, 0.0, 0.0, 1.0); // hide vertex by moving it outside of view
    }

    vert_color = col.bgr;
}
