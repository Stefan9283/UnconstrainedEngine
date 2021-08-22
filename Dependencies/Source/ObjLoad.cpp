#include "ObjLoad.h"


Mesh* readObj(const char* filepath) {
    FILE *f = fopen(filepath, "r");

    if (!f) return nullptr;
    Mesh *m = new Mesh;
    m->name = filepath;

    char *line = (char *) calloc(100, sizeof(char));
    std::vector<glm::vec3> posArr, normArr;
    std::vector<objVert> objvert;
    while (fgets(line, 100, f)) {


        if (line[0] == 'v' && line[1] == ' ') {
            glm::vec3 pos;
            sscanf(line, "v %f %f %f", &pos.x, &pos.y, &pos.z);
            posArr.push_back(pos);
        } else if (line[1] == 'n') {
            glm::vec3 norm;
            sscanf(line, "vn %f %f %f", &norm.x, &norm.y, &norm.z);
            normArr.push_back(norm);
            //std::cout << glm::to_string(norm) << "\n";
        } else if (line[0] == 'f') {
            objVert i[3];
            sscanf(
                    line,
                    "f %d/%d/%d %d/%d/%d %d/%d/%d",
                    &i[0].pos, &i[0].tex, &i[0].norm,
                    &i[1].pos, &i[1].tex, &i[1].norm,
                    &i[2].pos, &i[2].tex, &i[2].norm);

            objvert.push_back(i[0]);
            objvert.push_back(i[1]);
            objvert.push_back(i[2]);
        }

    }

    for (size_t i = 0; i < objvert.size(); i++) {
        Vertex v
                {
                        posArr[objvert[i].pos - 1],
                        normArr[objvert[i].norm - 1]
                };
        m->vertices.push_back(v);

        //std::cout << glm::to_string(v.Normal) << "\n";

        m->indices.push_back((uint32_t)i);

    }

    fclose(f);
    m->prepare();
    return m;
}