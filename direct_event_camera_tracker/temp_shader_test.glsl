
mat3 rotateZ(float rad) {
    float c = cos(rad);
    float s = sin(rad);
    return mat3(
        c, s, 0.0,
        -s, c, 0.0,
        0.0, 0.0, 1.0
    );
}

mat3 rotateX(float rad) {
    float c = cos(rad);
    float s = sin(rad);
    return mat3(
        1.0, 0.0, 0.0,
        0.0, c, s,
        0.0, -s, c
    );
}

mat3 rotateY(float rad) {
    float c = cos(rad);
    float s = sin(rad);
    return mat3(
        c, 0.0, -s,
        0.0, 1.0, 0.0,
        s, 0.0, c
    );
}


void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    
    mat3 R = rotateY(-iMouse.x / iResolution.x + 0.5) * rotateX(iMouse.y/iResolution.y-0.5);
    vec3 t = vec3(0.01, 0, 0);
    
    // Normalized pixel coordinates (from 0 to 1)
    
    
    
    vec3 uv = vec3(fragCoord/iResolution.xy-vec2(0.5, 0.5), 1);
    
    float Z = texture(iChannel1, uv.xy).r ;
    
    vec3 A_p = Z * uv;
    
    vec3 B_p = R * A_p + t;
    
    vec3 B_uv = B_p / B_p.z;
    

    if (B_uv.x >= 0.0f && B_uv.x <= 1.0f && B_uv.y >= 0.0f && B_uv.y <= 1.0f) {
    
    	fragColor = texture(iChannel0, B_uv.xy + vec2(0.5, 0.5));
    } else {
        fragColor = vec4(1,0,0,1);
    }
    
    //vec2 test = uv.xy + vec2(Z/10.0f,0);
    
    
    //fragColor = texture(iChannel0, test);

}
