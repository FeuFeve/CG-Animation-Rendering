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
#include <iomanip>
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

vector<Vec3> positions3;
vector<Vec3> normals3;

vector<vector<Vec3>> animPositions;
vector<vector<Vec3>> animNormals;

unsigned int animId = 0;
unsigned int lastAnimId = 1;
int startTime;
float lastTimeUpdate = 0;

float fps = 30.0f;
int iterations = 100;

bool showAnimation = true;
bool showBaseModel = true;
bool showResultingModel = true;

// Vec3 p;
// Mat3 rot;

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
// utils.
// ------------------------------------------------------------------------------------------------------------

float clamp(float x, float lowerlimit, float upperlimit) {
    if (x < lowerlimit)
        x = lowerlimit;
    if (x > upperlimit)
        x = upperlimit;
    return x;
}

float smoothstep(float edge0, float edge1, float x) {
    // Scale, bias and saturate x to 0..1 range
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x * x * (3 - 2 * x);
}

void updateAnimIndex() {
    int deltaTime = glutGet(GLUT_ELAPSED_TIME) - startTime - lastTimeUpdate;
    float fpsDeltaTime = 1000 / fps;
    if (deltaTime > fpsDeltaTime) {
        lastTimeUpdate += fpsDeltaTime;
        animId++;
        if (animId == animPositions.size())
            animId = 0;
    }
}

float calculateScore(float meanDistance) {
    return 100.0f / (1 + 100 * meanDistance);
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
    srand(time(nullptr));
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

void drawAnim() {
    updateAnimIndex();
    if (animPositions.size() == 0)
        return;

    if (lastAnimId != animId) {
        lastAnimId = animId;
    }

    if (showAnimation)
        drawPointSet(animPositions[animId], animNormals[animId]);
}

void draw() {
    drawAxes();

    glPointSize(2);

    glColor3f(0.8, 0.8, 1.0);
    drawPointSet(positions, normals);

    if (showBaseModel) {
        glColor3f(0, 0, 1);
        drawPointSet(positions2, normals2);
    }

    if (showResultingModel) {
        glColor3f(0, 1, 0);
        drawPointSet(positions3, normals3);
    }

    glColor3f(1.0, 0.41, 0.38);
    drawAnim();
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
        case 'f':  // FULLSCREEN
            if (fullScreen == true) {
                glutReshapeWindow(SCREENWIDTH, SCREENHEIGHT);
                fullScreen = false;
            } else {
                glutFullScreen();
                fullScreen = true;
            }
            break;

        case 'w':  // SHOW TRIANGLE GEOMETRY/FILL
            GLint polygonMode[2];
            glGetIntegerv(GL_POLYGON_MODE, polygonMode);
            if (polygonMode[0] != GL_FILL)
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            else
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            break;

        case 'a':  // SHOW ANIMATION
            showAnimation = !showAnimation;
            break;
        case 'b':  // SHOW BASE MODEL
            showBaseModel = !showBaseModel;
            break;
        case 'r':  // SHOW RESULTING MODEL
            showResultingModel = !showResultingModel;
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
// Added in TP1
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

// ------------------------------------------------------------------------------------------------------------
// Added in TP3
// ------------------------------------------------------------------------------------------------------------

void addAnimationFrame(vector<Vec3> const &Q, vector<Vec3> const &Qn, Vec3 const &centroidQ, Vec3 const &translation,
                       unsigned const int iteration, unsigned const int maxIterations) {
    vector<Vec3> framePositions;
    framePositions.resize(Q.size());

    float smoothTranslationFactor = smoothstep(0, maxIterations, iteration);
    Vec3 frameTranslation = smoothTranslationFactor * translation;

    for (unsigned int i = 0; i < Q.size(); i++) {
        framePositions[i] = Q[i] + centroidQ + frameTranslation;
    }

    animPositions[iteration] = framePositions;
    animNormals[iteration] = Qn;
}

void ICP(vector<Vec3> const &ps, vector<Vec3> const &nps, vector<Vec3> const &qs, vector<Vec3> const &nqs,
         Mat3 &rotation, Vec3 &translation, unsigned int nbIterations) {
    // TODO:
    // Gérer l'INIT
    // Gérer les cas où P.size() != Q.size()
    animPositions.clear();
    animPositions.resize(nbIterations + 1);
    animNormals.clear();
    animNormals.resize(nbIterations + 1);

    // INIT

    // ICP
    float meanDistance;
    Vec3 centroidP = Vec3(0, 0, 0), centroidQ = Vec3(0, 0, 0), tempPoint;
    Mat3 iterationRotation = Mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    vector<Vec3> P, Q, Qn;
    vector<unsigned int> correspondingIds;
    BasicANNkdTree qKdTree;

    // Resize
    P.resize(ps.size());
    Q.resize(qs.size());
    Qn.resize(nqs.size());
    correspondingIds.resize(P.size());

    // Copy
    for (unsigned int i = 0; i < P.size(); i++) {
        P[i] = ps[i];
    }
    for (unsigned int i = 0; i < Q.size(); i++) {
        Q[i] = qs[i];
    }
    for (unsigned int i = 0; i < Qn.size(); i++) {
        Qn[i] = nqs[i];
    }

    // Calculate the centroids of P and Q
    for (unsigned int i = 0; i < P.size(); i++) {  // Centroid of P
        centroidP += P[i];
    }
    centroidP /= P.size();
    for (unsigned int i = 0; i < Q.size(); i++) {  // Centroid of Q
        centroidQ += Q[i];
    }
    centroidQ /= Q.size();

    // Center P and Q around the origin (0,0,0) using their centroids
    for (unsigned int i = 0; i < P.size(); i++) {  // Center P around the origin
        P[i] -= centroidP;
    }
    for (unsigned int i = 0; i < Q.size(); i++) {  // Center Q around the origin
        Q[i] -= centroidQ;
    }

    translation = centroidP - centroidQ;

    // Add the first animation frame (Q in initial position)
    addAnimationFrame(Q, Qn, centroidQ, translation, 0, nbIterations);

    // Begin iterations
    for (unsigned int iteration = 0; iteration < nbIterations; iteration++) {
        // Rebuild the kdTree
        qKdTree.build(Q);

        // For each point in P, find the closest point in Q. Q points can have 0, 1 or more associated P points
        meanDistance = 0;
        for (unsigned int i = 0; i < P.size(); i++) {
            correspondingIds[i] = qKdTree.nearest(P[i]);
            meanDistance += (P[i] - Q[correspondingIds[i]]).length();
        }
        meanDistance /= P.size();
        cout << "ICP iteration: " << (iteration + 1) << "/" << nbIterations << ", score = " << calculateScore(meanDistance) << "%" << endl;

        // Calculate the matrix product
        iterationRotation = Mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
        for (unsigned int i = 0; i < P.size(); i++) {
            iterationRotation += Mat3::tensor(P[i], Q[correspondingIds[i]]);
        }
        iterationRotation.setRotation();

        iteration == 0 ? rotation = iterationRotation : rotation = iterationRotation * rotation;

        // Apply the rotation and trto the points
        for (unsigned int i = 0; i < Q.size(); i++) {
            Q[i] = iterationRotation * Q[i];
            Qn[i] = iterationRotation * Qn[i];
            Qn[i].normalize();
        }

        // Add the iteration results to the animPositions vector to display it
        addAnimationFrame(Q, Qn, centroidQ, translation, iteration + 1, nbIterations);
    }

    cout << "End of ICP." << endl;
    cout << "Translation vector is: (" << translation << ")" << endl;
    cout << "Rotation matrix is:" << endl;
    cout << "{ " << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " }" << endl;
    cout << "{ " << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " }" << endl;
    cout << "{ " << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " }" << endl;
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
        loadPN("pointsets/african_statue_subsampled_extreme.pn", positions, normals);
        loadPN("pointsets/african_statue_subsampled_extreme.pn", positions2, normals2);

        // Randomly move the two pointsets
        applyRandomRigidTransformation(positions, normals);
        applyRandomRigidTransformation(positions2, normals2);

        // Apply an ICP on the models
        Mat3 resultingRotation;
        Vec3 resultingTranslation;
        ICP(positions, normals, positions2, normals2, resultingRotation, resultingTranslation, iterations);

        // ## TEST IF THE RESULTS ARE CORRECT ##

        positions3.resize(positions2.size());
        normals3.resize(normals2.size());
        Vec3 pos3Centroid = Vec3(0, 0, 0);

        // Copy
        for (unsigned int i = 0; i < positions2.size(); i++) {
            positions3[i] = positions2[i];
            normals3[i] = normals2[i];
            pos3Centroid += positions2[i];
        }
        pos3Centroid /= positions3.size();

        // Center around 0, rotate, push back and translate
        Vec3 translationFromOrigin = pos3Centroid + resultingTranslation;
        for (unsigned int i = 0; i < positions2.size(); i++) {
            positions3[i] = resultingRotation * (positions2[i] - pos3Centroid) + translationFromOrigin;
            normals3[i] = resultingRotation * normals2[i];
        }

        // ## END OF TEST ##

        startTime = glutGet(GLUT_ELAPSED_TIME);
    }

    glutMainLoop();
    return EXIT_SUCCESS;
}
