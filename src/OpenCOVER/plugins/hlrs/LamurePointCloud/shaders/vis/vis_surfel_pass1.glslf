#version 420 core
in GS_OUT { noperspective vec2 uv; } fin;

void main() {
    if (dot(fin.uv, fin.uv) > 1.0) discard;
    // tiefe nur für valide Pixel schreiben (Late-Z)
    gl_FragDepth = gl_FragCoord.z;
}
