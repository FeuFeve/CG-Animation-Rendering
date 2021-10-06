// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <GL/glut.h>
#include <float.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "src/Camera.h"
#include "src/Vec3.h"
#include "src/jmkdtree.h"

using namespace std;

vector<Vec3> positions;
vector<Vec3> normals;

vector<Vec3> positions2;
vector<Vec3> normals2;

enum KernelType { KERNEL_CLASSIC,
                  KERNEL_GAUSSIAN };

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 640;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX = 0, lastY = 0, lastZoom = 0;
static bool fullScreen = false;

// ------------------------------------------------------------------------------------------------------------
// i/o and some stuff
// ------------------------------------------------------------------------------------------------------------

void loadPN(const string &filename, vector<Vec3> &o_positions, vector<Vec3> &o_normals) {
    unsigned int surfelSize = 6;
    FILE *in = fopen(filename.c_str(), "rb");
    if (in == NULL) {
        cout << filename << " is not a valid PN file." << endl;
        return;
    }
    size_t READ_BUFFER_SIZE = 1000;  // for example...
    float *pn = new float[surfelSize * READ_BUFFER_SIZE];
    o_positions.clear();
    o_normals.clear();
    while (!feof(in)) {
        unsigned numOfPoints = fread(pn, 4, surfelSize * READ_BUFFER_SIZE, in);
        for (unsigned int i = 0; i < numOfPoints; i += surfelSize) {
            o_positions.push_back(Vec3(pn[i], pn[i + 1], pn[i + 2]));
            o_normals.push_back(Vec3(pn[i + 3], pn[i + 4], pn[i + 5]));
        }

        if (numOfPoints < surfelSize * READ_BUFFER_SIZE)
            break;
    }
    fclose(in);
    delete[] pn;
}

void savePN(const string &filename, vector<Vec3> const &o_positions, vector<Vec3> const &o_normals) {
    if (o_positions.size() != o_normals.size()) {
        cout << "The pointset you are trying to save does not contain the same number of points and normals." << endl;
        return;
    }
    FILE *outfile = fopen(filename.c_str(), "wb");
    if (outfile == NULL) {
        cout << filename << " is not a valid PN file." << endl;
        return;
    }
    for (unsigned int pIt = 0; pIt < o_positions.size(); ++pIt) {
        fwrite(&(o_positions[pIt]), sizeof(float), 3, outfile);
        fwrite(&(o_normals[pIt]), sizeof(float), 3, outfile);
    }
    fclose(outfile);
}

void scaleAndCenter(vector<Vec3> &io_positions) {
    Vec3 bboxMin(FLT_MAX, FLT_MAX, FLT_MAX);
    Vec3 bboxMax(FLT_MIN, FLT_MIN, FLT_MIN);
    for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
        for (unsigned int coord = 0; coord < 3; ++coord) {
            bboxMin[coord] = min<float>(bboxMin[coord], io_positions[pIt][coord]);
            bboxMax[coord] = max<float>(bboxMax[coord], io_positions[pIt][coord]);
        }
    }
    Vec3 bboxCenter = (bboxMin + bboxMax) / 2.f;
    float bboxLongestAxis = max<float>(bboxMax[0] - bboxMin[0], max<float>(bboxMax[1] - bboxMin[1], bboxMax[2] - bboxMin[2]));
    for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
        io_positions[pIt] = (io_positions[pIt] - bboxCenter) / bboxLongestAxis;
    }
}

void applyRandomRigidTransformation(vector<Vec3> &io_positions, vector<Vec3> &io_normals) {
    srand(time(NULL));
    Mat3 R = Mat3::RandRotation();
    Vec3 t = Vec3::Rand(1.f);
    for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
        io_positions[pIt] = R * io_positions[pIt] + t;
        io_normals[pIt] = R * io_normals[pIt];
    }
}

void subsample(vector<Vec3> &i_positions, vector<Vec3> &i_normals, float minimumAmount = 0.1f, float maximumAmount = 0.2f) {
    vector<Vec3> newPos, newNormals;
    vector<unsigned int> indices(i_positions.size());
    for (unsigned int i = 0; i < indices.size(); ++i)
        indices[i] = i;
    srand(time(NULL));
    random_shuffle(indices.begin(), indices.end());
    unsigned int newSize = indices.size() * (minimumAmount + (maximumAmount - minimumAmount) * (float)(rand()) / (float)(RAND_MAX));
    newPos.resize(newSize);
    newNormals.resize(newSize);
    for (unsigned int i = 0; i < newPos.size(); ++i) {
        newPos[i] = i_positions[indices[i]];
        newNormals[i] = i_normals[indices[i]];
    }
    i_positions = newPos;
    i_normals = newNormals;
}

bool save(const string &filename, vector<Vec3> &vertices, vector<unsigned int> &triangles) {
    ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open()) {
        cout << filename << " cannot be opened" << endl;
        return false;
    }

    myfile << "OFF" << endl;

    unsigned int n_vertices = vertices.size(), n_triangles = triangles.size() / 3;
    myfile << n_vertices << " " << n_triangles << " 0" << endl;

    for (unsigned int v = 0; v < n_vertices; ++v) {
        myfile << vertices[v][0] << " " << vertices[v][1] << " " << vertices[v][2] << endl;
    }
    for (unsigned int f = 0; f < n_triangles; ++f) {
        myfile << 3 << " " << triangles[3 * f] << " " << triangles[3 * f + 1] << " " << triangles[3 * f + 2];
        myfile << endl;
    }
    myfile.close();
    return true;
}

// ------------------------------------------------------------------------------------------------------------
// rendering.
// ------------------------------------------------------------------------------------------------------------

void initLight() {
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f, -16.0f, -50.0f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};

    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, color1);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);
}

void init() {
    camera.resize(SCREENWIDTH, SCREENHEIGHT);
    initLight();
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
}

void drawTriangleMesh(vector<Vec3> const &i_positions, vector<unsigned int> const &i_triangles) {
    glBegin(GL_TRIANGLES);
    for (unsigned int tIt = 0; tIt < i_triangles.size() / 3; ++tIt) {
        Vec3 p0 = i_positions[3 * tIt];
        Vec3 p1 = i_positions[3 * tIt + 1];
        Vec3 p2 = i_positions[3 * tIt + 2];
        Vec3 n = Vec3::cross(p1 - p0, p2 - p0);
        n.normalize();
        glNormal3f(n[0], n[1], n[2]);
        glVertex3f(p0[0], p0[1], p0[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);
    }
    glEnd();
}

void drawPointSet(vector<Vec3> const &i_positions, vector<Vec3> const &i_normals) {
    glBegin(GL_POINTS);
    for (unsigned int pIt = 0; pIt < i_positions.size(); ++pIt) {
        glNormal3f(i_normals[pIt][0], i_normals[pIt][1], i_normals[pIt][2]);
        glVertex3f(i_positions[pIt][0], i_positions[pIt][1], i_positions[pIt][2]);
    }
    glEnd();
}

void draw() {
    glPointSize(2);  // for example...

    glColor3f(0.8, 0.8, 1);
    drawPointSet(positions, normals);

    glColor3f(1, 0.0, 0.0);
    drawPointSet(positions2, normals2);
}

void display() {
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply();
    draw();
    glFlush();
    glutSwapBuffers();
}

void idle() {
    glutPostRedisplay();
}

void key(unsigned char keyPressed, int x, int y) {
    switch (keyPressed) {
        case 'f':
            if (fullScreen == true) {
                glutReshapeWindow(SCREENWIDTH, SCREENHEIGHT);
                fullScreen = false;
            } else {
                glutFullScreen();
                fullScreen = true;
            }
            break;

        case 'w':
            GLint polygonMode[2];
            glGetIntegerv(GL_POLYGON_MODE, polygonMode);
            if (polygonMode[0] != GL_FILL)
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            else
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            break;

        default:
            break;
    }
    idle();
}

void mouse(int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate(x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle();
}

void motion(int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate(x, y);
    } else if (mouseMovePressed == true) {
        camera.move((x - lastX) / static_cast<float>(SCREENWIDTH), (lastY - y) / static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    } else if (mouseZoomPressed == true) {
        camera.zoom(float(y - lastZoom) / SCREENHEIGHT);
        lastZoom = y;
    }
}

void reshape(int w, int h) {
    camera.resize(w, h);
}

// ------------------------------------------------------------------------------------------------------------
// added in TP1
// ------------------------------------------------------------------------------------------------------------

float gauss(Vec3 x, Vec3 pi, float h) {
    float r = (x - pi).length();
    return exp(-pow(r, 2) / pow(h, 2));
}

void HPSS(Vec3 inputPoint, Vec3 &outputPoint, Vec3 &outputNormal, vector<Vec3> const &positions, vector<Vec3> const &normals,
          BasicANNkdTree const &kdtree, int kernelType, float h, unsigned int nbIterations = 10, unsigned int knn = 20) {
    Vec3 centroidNormal = Vec3(0, 0, 0);
    for (unsigned int i = 0; i < nbIterations; i++) {
        // Find neighbor points in kdtree
        ANNidxArray idNearestNeighbors = new ANNidx[knn];
        ANNdistArray distanceToNeighbors = new ANNdist[knn];
        kdtree.knearest(inputPoint, knn, idNearestNeighbors, distanceToNeighbors);

        // Get the inputPoint's projection on those points
        int id;
        float gaussR;
        Vec3 normal, projectedPoint, centroid = Vec3(0, 0, 0);

        switch (kernelType) {
            case KERNEL_CLASSIC:
                for (unsigned int j = 0; j < knn; j++) {
                    id = idNearestNeighbors[j];

                    normal = normals[id];
                    projectedPoint = (positions[id] - distanceToNeighbors[j] * normal);

                    centroid += projectedPoint;
                    if (i == nbIterations - 1)
                        centroidNormal += normal;
                }
                break;

            case KERNEL_GAUSSIAN:
                for (unsigned int j = 0; j < knn; j++) {
                    id = idNearestNeighbors[j];
                    gaussR = gauss(inputPoint, positions[id], h);

                    normal = normals[id];
                    projectedPoint = (positions[id] - distanceToNeighbors[j] * normal);

                    centroid += gaussR * projectedPoint;
                    if (i == nbIterations - 1)
                        centroidNormal += gaussR * normal;
                }
                break;

            default:
                break;
        }

        if (i == nbIterations - 1)
            centroidNormal.normalize();

        centroid /= knn;
        inputPoint = centroid;
    }

    outputPoint = inputPoint - Vec3(-1, 0, 0);
    outputNormal = centroidNormal;
}

double randDistrib(double step, int nbStep) {
    double toReturn = 0;
    for (int i = 0; i < nbStep; i++) {
        toReturn += (-step + step * 2 * (double)(rand()) / (double)(RAND_MAX));
    }
    return toReturn;
}

int main(int argc, char **argv) {
    if (argc > 2) {
        exit(EXIT_FAILURE);
    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow("tp point processing");

    init();
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutReshapeFunc(reshape);
    glutMotionFunc(motion);
    glutMouseFunc(mouse);
    key('?', 0, 0);

    {
        // Load a first pointset, and build a kd-tree:
        loadPN("pointsets/igea.pn", positions, normals);

        BasicANNkdTree kdtree;
        kdtree.build(positions);

        // Ajout de bruit
        for (unsigned int pIt = 0; pIt < positions.size(); pIt++) {
            positions[pIt] += randDistrib(0.015, 10) * normals[pIt];
        }

        // Create a second pointset that is artificial, and project it on pointset1 using MLS techniques:
        int kernelType = KERNEL_GAUSSIAN;
        if (argc == 2)
            positions2.resize(atoi(argv[1]));
        else
            positions2.resize(20000);

        normals2.resize(positions2.size());
        for (unsigned int pIt = 0; pIt < positions2.size(); ++pIt) {
            positions2[pIt] = Vec3(
                -2 + 4 * (double)(rand()) / (double)(RAND_MAX),
                -2 + 4 * (double)(rand()) / (double)(RAND_MAX),
                -2 + 4 * (double)(rand()) / (double)(RAND_MAX));
            // positions2[pIt].normalize();
            // positions2[pIt] = 0.6 * positions2[pIt];
        }

        // PROJECT USING MLS (HPSS and APSS):
        // TODO
        Vec3 inputPoint;
        for (unsigned int pIt = 0; pIt < positions2.size(); ++pIt) {
            inputPoint = positions2[pIt];
            HPSS(inputPoint, positions2[pIt], normals2[pIt], positions, normals, kdtree, kernelType, 1.5f);
            if ((pIt + 1) % 500 == 0)
                cout << "Progress: " << (pIt + 1) << "/" << positions2.size() << endl;
        }
    }

    glutMainLoop();
    return EXIT_SUCCESS;
}
