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

vector<Vec3> meshVertices;
vector<Vec3> meshVertices2;
vector<Vec3> meshVertices3;
vector<Vec3> meshVertices4;
vector<Vec3> meshVertices5;
vector<Vec3> meshVertices6;

int startTime;

enum KernelType {
    KERNEL_CLASSIC,
    KERNEL_GAUSSIAN
};

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

void drawTriangleMesh(vector<Vec3> const &i_positions, bool drawBackFace) {
    glBegin(GL_TRIANGLES);
    for (unsigned int i = 0; i < i_positions.size(); i += 3) {
        Vec3 p0 = i_positions[i];
        Vec3 p1 = i_positions[i + 1];
        Vec3 p2 = i_positions[i + 2];
        Vec3 n = Vec3::cross(p1 - p0, p2 - p0);
        n.normalize();

        // Front face
        glNormal3f(n[0], n[1], n[2]);
        glVertex3f(p0[0], p0[1], p0[2]);
        glVertex3f(p1[0], p1[1], p1[2]);
        glVertex3f(p2[0], p2[1], p2[2]);

        // Back face
        if (drawBackFace) {
            glNormal3f(-n[0], -n[1], -n[2]);
            glVertex3f(p0[0], p0[1], p0[2]);
            glVertex3f(p2[0], p2[1], p2[2]);
            glVertex3f(p1[0], p1[1], p1[2]);
        }
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

void drawAxes() {
    glDisable(GL_LIGHTING);
    // X axis
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.5, 0.0, 0.0);
    glEnd();

    // Y axis
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.5, 0.0);
    glEnd();

    // Z axis
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.5);
    glEnd();
    glEnable(GL_LIGHTING);
}

void drawMeshN(int n) {
    switch (n % 6) {
        case 0:
            drawTriangleMesh(meshVertices, true);
            break;
        case 1:
            meshVertices2.size() == 0 ? drawMeshN(0) : drawTriangleMesh(meshVertices2, true);
            break;
        case 2:
            meshVertices3.size() == 0 ? drawMeshN(1) : drawTriangleMesh(meshVertices3, true);
            break;
        case 3:
            meshVertices4.size() == 0 ? drawMeshN(2) : drawTriangleMesh(meshVertices4, true);
            break;
        case 4:
            meshVertices5.size() == 0 ? drawMeshN(3) : drawTriangleMesh(meshVertices5, true);
            break;
        case 5:
            meshVertices6.size() == 0 ? drawMeshN(4) : drawTriangleMesh(meshVertices6, true);
            break;
        default:
            break;
    }
}

void draw() {
    drawAxes();

    glPointSize(2);  // for example...

    glColor3f(0.8, 0.8, 1.0);
    drawPointSet(positions, normals);

    // glColor3f(1.0, 0.0, 0.0);
    // drawPointSet(positions2, normals2);

    glColor3f(0.66, 0.82, 0.76);
    drawMeshN(time(nullptr));
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

Vec3 projectToPlan(Vec3 point, Vec3 planePoint, Vec3 planeNormal) {
    return point - Vec3::dot(planeNormal, (point - planePoint)) * planeNormal;
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

        for (unsigned int j = 0; j < knn; j++) {
            id = idNearestNeighbors[j];

            normal = normals[id];
            // projectedPoint = (positions[id] - distanceToNeighbors[j] * normal);
            projectedPoint = projectToPlan(inputPoint, positions[id], normal);

            switch (kernelType) {
                case KERNEL_CLASSIC:
                    centroid += projectedPoint;
                    if (i == nbIterations - 1)
                        centroidNormal += normal;
                    break;

                case KERNEL_GAUSSIAN:
                    gaussR = gauss(inputPoint, positions[id], h);
                    centroid += gaussR * projectedPoint;
                    if (i == nbIterations - 1)
                        centroidNormal += gaussR * normal;
                    break;

                default:
                    break;
            }
        }

        if (i == nbIterations - 1)
            centroidNormal.normalize();

        centroid /= knn;
        inputPoint = centroid;
    }

    outputPoint = inputPoint;
    outputNormal = centroidNormal;
}

void APSS(Vec3 inputPoint, Vec3 &outputPoint, Vec3 &outputNormal, vector<Vec3> const &positions, vector<Vec3> const &normals,
          BasicANNkdTree const &kdtree, int kernelType, float h, unsigned int nbIterations = 10, unsigned int knn = 20) {
}

double randDistrib(double step, int nbStep) {
    double toReturn = 0;
    for (int i = 0; i < nbStep; i++) {
        toReturn += (-step + step * 2 * (double)(rand()) / (double)(RAND_MAX));
    }
    return toReturn;
}

void addTriangleToMesh(Vec3 const &A, Vec3 const &B, Vec3 const &C) {
    meshVertices.push_back(A);
    meshVertices.push_back(B);
    meshVertices.push_back(C);
}

void projectAndAdd(Vec3 A, Vec3 B, Vec3 C, Vec3 D, vector<Vec3> const &positions, vector<Vec3> const &normals,
                   BasicANNkdTree const &kdtree, int kernelType, float h, unsigned int nbIterations, unsigned int knn) {
    Vec3 n;

    HPSS(A, A, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);
    HPSS(B, B, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);
    HPSS(C, C, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);
    HPSS(D, D, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);

    addTriangleToMesh(A, B, C);
    addTriangleToMesh(A, C, D);
}

void dualContouring(int gridSize, BasicANNkdTree const &kdtree, int kernelType, float h, unsigned int nbIterations = 10,
                    unsigned int knn = 20) {
    // Initialize min/max bbox points
    float minFloat = numeric_limits<float>::min();
    float maxFloat = numeric_limits<float>::max();
    Vec3 bboxMin(maxFloat, maxFloat, maxFloat);
    Vec3 bboxMax(minFloat, minFloat, minFloat);

    // Find the min/max bbox points by analyzing the model
    for (auto &point : positions) {
        bboxMin[0] = min(point[0], bboxMin[0]);
        bboxMin[1] = min(point[1], bboxMin[1]);
        bboxMin[2] = min(point[2], bboxMin[2]);
        bboxMax[0] = max(point[0], bboxMax[0]);
        bboxMax[1] = max(point[1], bboxMax[1]);
        bboxMax[2] = max(point[2], bboxMax[2]);
    }
    cout << "min: (" << bboxMin[0] << " " << bboxMin[1] << " " << bboxMin[2] << ")" << endl;
    cout << "max: (" << bboxMax[0] << " " << bboxMax[1] << " " << bboxMax[2] << ")" << endl;

    Vec3 bboxOffset = Vec3(0.01, 0.01, 0.01);
    bboxMin -= bboxOffset;
    bboxMax += bboxOffset;

    // Create a voxel grid
    int steps = gridSize - 1;
    float xStep = (bboxMax[0] - bboxMin[0]) / steps;
    float yStep = (bboxMax[1] - bboxMin[1]) / steps;
    float zStep = (bboxMax[2] - bboxMin[2]) / steps;

    float xCurrent, yCurrent, zCurrent;

    unsigned int index = 0;
    for (int x = 0; x < gridSize; x++) {
        xCurrent = bboxMin[0] + x * xStep;

        for (int y = 0; y < gridSize; y++) {
            yCurrent = bboxMin[1] + y * yStep;

            for (int z = 0; z < gridSize; z++) {
                zCurrent = bboxMin[2] + z * zStep;
                positions2[index] = Vec3(xCurrent, yCurrent, zCurrent);
                normals2[index++] = Vec3(0, 1, 0);
            }
        }
    }

    // Evaluate each points of each voxels
    Vec3 px, nx;
    vector<bool> isInside;
    isInside.resize(positions2.size());

    for (unsigned int i = 0; i < positions2.size(); i++) {
        Vec3 x = positions2[i];
        HPSS(x, px, nx, positions, normals, kdtree, kernelType, h, nbIterations, knn);

        isInside[i] = Vec3::dot(x - px, nx) <= 0;

        // Print progress
        if ((i + 1) % 2500 == 0)
            cout << "Evaluation progress: " << (i + 1) << "/" << positions2.size() << endl;
    }

    // Find the voxels intersected by the model
    int pointIndex, hasPointIndex = 0, gridSizeSqr = pow(gridSize, 2), cptrIn, cptrOut;

    int outputVectorsSize = pow(gridSize - 1, 3);
    vector<bool> hasPoint;
    hasPoint.resize(outputVectorsSize);

    vector<Vec3> outputPoints;
    vector<Vec3> outputNormals;

    for (int x = 0; x < gridSize - 1; x++) {
        for (int y = 0; y < gridSize - 1; y++) {
            for (int z = 0; z < gridSize - 1; z++) {
                cptrIn = 0;
                cptrOut = 0;

                pointIndex = x * gridSizeSqr + y * gridSize + z;
                isInside[pointIndex] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + 1] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSize] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSize + 1] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSizeSqr] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSizeSqr + 1] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSizeSqr + gridSize] ? cptrIn++ : cptrOut++;
                isInside[pointIndex + gridSizeSqr + gridSize + 1] ? cptrIn++ : cptrOut++;

                hasPoint[hasPointIndex] = cptrIn != 0 && cptrOut != 0;  // True if at least 1 point inside and 1 point outside

                if (hasPoint[hasPointIndex]) {
                    outputPoints.push_back(positions2[pointIndex] + Vec3(xStep / 2, yStep / 2, zStep / 2));
                    outputNormals.push_back(Vec3(0, 1, 0));
                }
                hasPointIndex++;
            }
        }
    }

    // Create the mesh
    int nextIndex, maxIndex = pow(gridSize - 1, 3);
    float xCoor, yCoor, zCoor;
    Vec3 A, B, C, D, n;

    for (int x = 0; x < gridSize - 1; x++) {
        for (int y = 0; y < gridSize - 1; y++) {
            for (int z = 0; z < gridSize - 1; z++) {
                index = x * gridSizeSqr + y * gridSize + z;

                if (y != 0 && z != 0) {  // x edge
                    nextIndex = index + gridSizeSqr;
                    if (isInside[index] != isInside[nextIndex]) {  // Link the 4 points
                        xCoor = (positions2[index][0] + positions2[nextIndex][0]) / 2;
                        yCoor = positions2[index][1] - yStep / 2;
                        zCoor = positions2[index][2] - zStep / 2;

                        A = Vec3(xCoor, yCoor, zCoor);
                        B = Vec3(xCoor, yCoor + yStep, zCoor);
                        C = Vec3(xCoor, yCoor + yStep, zCoor + zStep);
                        D = Vec3(xCoor, yCoor, zCoor + zStep);

                        projectAndAdd(A, B, C, D, positions, normals, kdtree, kernelType, h, nbIterations, knn);
                    }
                }

                if (x != 0 && z != 0) {  // y edge
                    nextIndex = index + gridSize;
                    if (isInside[index] != isInside[nextIndex]) {  // Link the 4 points
                        xCoor = positions2[index][0] - xStep / 2;
                        yCoor = (positions2[index][1] + positions2[nextIndex][1]) / 2;
                        zCoor = positions2[index][2] - zStep / 2;

                        A = Vec3(xCoor, yCoor, zCoor);
                        B = Vec3(xCoor + xStep, yCoor, zCoor);
                        C = Vec3(xCoor + xStep, yCoor, zCoor + zStep);
                        D = Vec3(xCoor, yCoor, zCoor + zStep);

                        projectAndAdd(A, B, C, D, positions, normals, kdtree, kernelType, h, nbIterations, knn);
                    }
                }

                if (x != 0 && y != 0) {  // z edge
                    nextIndex = index + 1;
                    if (isInside[index] != isInside[nextIndex]) {  // Link the 4 points
                        xCoor = positions2[index][0] - xStep / 2;
                        yCoor = positions2[index][1] - yStep / 2;
                        zCoor = (positions2[index][2] + positions2[nextIndex][2]) / 2;

                        A = Vec3(xCoor, yCoor, zCoor);
                        B = Vec3(xCoor + xStep, yCoor, zCoor);
                        C = Vec3(xCoor + xStep, yCoor + yStep, zCoor);
                        D = Vec3(xCoor, yCoor + yStep, zCoor);

                        projectAndAdd(A, B, C, D, positions, normals, kdtree, kernelType, h, nbIterations, knn);
                    }
                }

                // Print progress
                if ((index + 1) % 2500 == 0)
                    cout << "Mesh progress: " << (index + 1) << "/" << maxIndex << endl;
            }
        }
    }

    cout << "Mesh creation triangles: " << (meshVertices.size() / 3) << endl;

    positions2 = outputPoints;
    normals2 = outputNormals;
}

vector<Vec3> subdivideMeshVertices(vector<Vec3> const &vertices, BasicANNkdTree const &kdtree, int kernelType, float h, unsigned int nbIterations = 10,
                                   unsigned int knn = 20) {
    Vec3 A, B, C, D, E, F, n;
    vector<Vec3> newMeshVertices;
    for (unsigned int i = 0; i < vertices.size(); i += 3) {
        A = vertices[i];
        B = vertices[i + 1];
        C = vertices[i + 2];

        D = (A + B) / 2;
        E = (B + C) / 2;
        F = (C + A) / 2;

        // Project D, E and F on the model
        HPSS(D, D, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);
        HPSS(E, E, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);
        HPSS(F, F, n, positions, normals, kdtree, kernelType, h, nbIterations, knn);

        // T1
        newMeshVertices.push_back(A);
        newMeshVertices.push_back(D);
        newMeshVertices.push_back(F);
        // T2
        newMeshVertices.push_back(B);
        newMeshVertices.push_back(E);
        newMeshVertices.push_back(D);
        // T3
        newMeshVertices.push_back(C);
        newMeshVertices.push_back(F);
        newMeshVertices.push_back(E);
        // T4
        newMeshVertices.push_back(D);
        newMeshVertices.push_back(E);
        newMeshVertices.push_back(F);

        // Print progress
        if ((i + 1) % 2500 == 0)
            cout << "Subdivision progress: " << (i + 1) << "/" << vertices.size() << endl;
    }

    return newMeshVertices;
}

// /!\ CALL LAST IN MAIN
// If points are moved and calculs are made on those points afterwards, results might be wrong
void movePointsToVisualize() {
    float offset = 0.5f;
    // Move the model by -0.5 on the x axis
    for (auto &point : positions) {
        point[0] -= offset;
    }

    // Move the resulting points by 0.5 on the x axis
    for (auto &point : positions2) {
        point[0] += offset;
    }

    // Move the mesh by 0.5 on the x axis
    for (auto &vertex : meshVertices) {
        vertex[0] += offset;
    }
    for (auto &vertex : meshVertices2) {
        vertex[0] += offset;
    }
    for (auto &vertex : meshVertices3) {
        vertex[0] += offset;
    }
    for (auto &vertex : meshVertices4) {
        vertex[0] += offset;
    }
    for (auto &vertex : meshVertices5) {
        vertex[0] += offset;
    }
    for (auto &vertex : meshVertices6) {
        vertex[0] += offset;
    }
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

        // Create a second pointset that is artificial, and project it on pointset1 using MLS techniques:
        int gridSize = 8;
        if (argc == 2) {
            gridSize = atoi(argv[1]);
        }

        positions2.resize(pow(gridSize, 3));
        normals2.resize(positions2.size());

        dualContouring(gridSize, kdtree, KERNEL_CLASSIC, 1.0f);

        meshVertices2 = subdivideMeshVertices(meshVertices, kdtree, KERNEL_CLASSIC, 1.0f);
        meshVertices3 = subdivideMeshVertices(meshVertices2, kdtree, KERNEL_CLASSIC, 1.0f);
        meshVertices4 = subdivideMeshVertices(meshVertices3, kdtree, KERNEL_CLASSIC, 1.0f);
        meshVertices5 = subdivideMeshVertices(meshVertices4, kdtree, KERNEL_CLASSIC, 1.0f);
        meshVertices6 = subdivideMeshVertices(meshVertices5, kdtree, KERNEL_CLASSIC, 1.0f);

        movePointsToVisualize();

        startTime = time(nullptr);
    }

    glutMainLoop();
    return EXIT_SUCCESS;
}
