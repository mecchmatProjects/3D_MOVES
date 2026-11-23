#define NOMINMAX

#include <windows.h>
#include <windowsx.h>
#include <GL/gl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "AStar3D.h"
#include "Physics.h"
#include "Telemetry.h"
#include "AnalysisWindow.h"
#include "RRT3D.h"

constexpr double PI = 3.14159265358979323846;

constexpr int NX = 10;
constexpr int NY = 7;
constexpr int NZ = 6;

const Vector3 DEFAULT_START{0, 2, 1};
const Vector3 DEFAULT_GOAL{9, 2, 1};

enum class MotionModel {
    Guided = 0,
    Uniform,
    ConstantAcceleration,
    GuidedGravity,
    GuidedDrag,
    GuidedGravityDrag
};

enum ControlID {
    ID_LIST = 100,
    ID_OBSTACLE_TYPE,
    ID_X, ID_Y, ID_Z,
    ID_A, ID_B, ID_C,
    ID_ADD, ID_DELETE, ID_RESET,
    ID_MOVING_TYPE,
    ID_MOVING_A, ID_MOVING_B, ID_MOVING_C,
    ID_SPEED, ID_ACCEL, ID_BRAKE,
    ID_METHOD, ID_PAUSE,
    ID_EXPORT, ID_SUMMARY, ID_CLEAR,
    ID_NEW_RUN, ID_ANALYSIS,
    ID_TAB_OBSTACLES,
    ID_TAB_MOVING,
    ID_TAB_PHYSICS,
    ID_OB_ROT_X, ID_OB_ROT_Y, ID_OB_ROT_Z,
    ID_MOV_ROT_X, ID_MOV_ROT_Y, ID_MOV_ROT_Z,
    ID_TAB_ROUTE, ID_TAB_MODELS,
    ID_START_X, ID_START_Y, ID_START_Z,
    ID_GOAL_X, ID_GOAL_Y, ID_GOAL_Z,
    ID_ROUTE_CRITERION, ID_ROUTE_WEIGHT, ID_APPLY_ROUTE,
    ID_VEL_X, ID_VEL_Y, ID_VEL_Z,
    ID_INITIAL_ACCEL_X, ID_INITIAL_ACCEL_Y, ID_INITIAL_ACCEL_Z,
    ID_TIME_STEP, ID_SAFETY_MARGIN, ID_MOTION_MODEL,
    ID_GRAVITY, ID_DRAG, ID_APPLY_MODEL,
    ID_MAX_TURN_RATE, ID_REPLAY, ID_STEP, ID_ROUTE_ALGORITHM
};

struct Camera {
    double yaw = -35.0;
    double pitch = 25.0;
    double distance = 21.0;
};

struct App {
    HWND scene = nullptr;
    HWND panel = nullptr;

    HWND list = nullptr;
    HWND obstacleType = nullptr;
    HWND obstacleEdits[6]{};
    HWND obstacleLabels[6]{};
    HWND obstacleRotationEdits[3]{};

    HWND movingType = nullptr;
    HWND movingEdits[3]{};
    HWND movingLabels[3]{};
    HWND movingRotationEdits[3]{};

    HWND routeStartEdits[3]{};
    HWND routeGoalEdits[3]{};
    HWND criterionCombo = nullptr;
    HWND timeWeightEdit = nullptr;
    HWND routeAlgorithmCombo = nullptr;

    HWND initialVelocityEdits[3]{};
    HWND initialAccelerationEdits[3]{};
    HWND integrationStepEdit = nullptr;
    HWND safetyMarginEdit = nullptr;
    HWND motionModelCombo = nullptr;
    HWND gravityEdit = nullptr;
    HWND dragEdit = nullptr;

    HWND speedEdit = nullptr;
    HWND accelEdit = nullptr;
    HWND brakeEdit = nullptr;
    HWND methodCombo = nullptr;
    HWND pauseButton = nullptr;
    HWND turnRateEdit = nullptr;
    HWND replayButton = nullptr;
    HWND stepButton = nullptr;
    HWND status = nullptr;
    HWND routeStatus = nullptr;
    HWND modelStatus = nullptr;

    HWND tabButtons[5]{};
    std::vector<HWND> tabControls[5];
    int activeTab = 0;

    HDC dc = nullptr;
    HGLRC gl = nullptr;

    std::vector<Obstacle> obstacles;
    std::vector<Vector3> rawPath;
    std::vector<Vector3> path;

    Obstacle moving{
        ShapeType::Sphere,
        DEFAULT_START,
        {0.35, 0.35, 0.35}
    };

    Physics physics;
    State3D state;

    Vector3 start = DEFAULT_START;
    Vector3 goal = DEFAULT_GOAL;
    Vector3 initialVelocity{};
    Vector3 initialAcceleration{};
    double integrationStep = 0.01;
    double safetyMargin = 0.02;
    double gravity = 9.81;
    double linearDrag = 0.25;
    double timeWeight = 1.0;
    double simulationAccumulator = 0.0;
    PathCriterion pathCriterion = PathCriterion::Distance;
    SearchAlgorithm algorithm = SearchAlgorithm::AStar;
    bool useRRT = false;
    bool replaying = false;
    std::vector<MotionSample> replaySamples;
    size_t replayIndex = 0;
    double replayClock = 0.0;
    MotionModel motionModel = MotionModel::Guided;

    IntegrationMethod method =
        IntegrationMethod::RK4;

    Telemetry telemetry;
    AnalysisWindow analysis;
    Camera camera;

    double totalLength = 0.0;
    double finishPause = 0.0;
    double searchMilliseconds = 0.0;

    int searchNodes = 0;

    bool running = true;
    bool paused = false;
    bool collision = false;
    bool smooth = true;
    bool updating = false;
    bool dragging = false;
    bool finished = false;

    int draggedIndex = -1;
    POINT lastMouse{};

    bool previousE = false;
    bool previousK = false;
    bool previousP = false;
    bool previousS = false;
    bool previousG = false;
    bool previousSpace = false;
};

App app;





void replan();
void newRun();
void resetScene();
void loadObstacle();
void loadMovingFields();
void loadPhysicsFields();
void loadRouteFields();
void loadModelFields();
void applyRouteFields();
void applyModelFields();
void showSettingsTab(int tab);
void updatePauseButton();
void startReplay();
void stopReplay();
void advanceOneStep(double dt);
void advance(double dt);

LRESULT CALLBACK panelProc(
    HWND, UINT, WPARAM, LPARAM
);





Vector3 toVector(GridPoint p) {
    return {
        double(p.x),
        double(p.y),
        double(p.z)
    };
}

std::string number(double value) {
    char buffer[64];

    std::snprintf(
        buffer, sizeof(buffer),
        "%.3f", value
    );

    return buffer;
}

bool readNumber(HWND field, double& result) {
    if (!field) return false;

    char buffer[128]{};
    GetWindowTextA(field, buffer, sizeof(buffer));

    char* end = nullptr;
    double value = std::strtod(buffer, &end);

    if (end == buffer ||
        *end != '\0' ||
        !std::isfinite(value)) {
        return false;
    }

    result = value;
    return true;
}

void setStatus(const std::string& message) {
    if (app.status) SetWindowTextA(app.status, message.c_str());
    if (app.routeStatus) SetWindowTextA(app.routeStatus, message.c_str());
    if (app.modelStatus) SetWindowTextA(app.modelStatus, message.c_str());
    std::cout << message << '\n';
}

void updatePauseButton() {
    if (app.pauseButton) {
        SetWindowTextA(
            app.pauseButton,
            app.paused ? "Resume" : "Pause"
        );
    }
}

void stopReplay() {
    if (!app.replaying) return;
    app.replaying=false;
    app.replaySamples.clear();
    app.replayIndex=0;
    app.replayClock=0.0;
    newRun();
}

void startReplay() {

    int last=-1;
    for (const auto& r:app.telemetry.runs())
        if(r.samples>=2 && r.status!=RunStatus::Running) last=r.run;
    if(last<0 && app.telemetry.run()>0) last=app.telemetry.run();
    if(last<0) {setStatus("No run available for replay");return;}
    const auto samples=app.telemetry.samplesForRun(last);
    if(samples.size()<2) {setStatus("Need at least two samples for replay");return;}
    app.telemetry.interruptRun();
    app.replaySamples=samples;
    app.replayClock=0.0;
    app.replayIndex=0;
    app.replaying=true;
    app.paused=false;
    updatePauseButton();
    app.moving.position={samples[0].x,samples[0].y,samples[0].z};
    setStatus("Replaying run "+std::to_string(last));
}

void interruptExperiment(
    const std::string& reason
) {
    app.telemetry.interruptRun();
    app.paused = true;
    app.state.velocity = {};
    app.state.acceleration = {};

    updatePauseButton();
    app.analysis.refresh();
    setStatus(reason);
}

void completeExperiment() {
    if (app.finished) return;

    app.finished = true;

    app.moving.position = app.goal;
    app.state.position = app.goal;
    app.state.velocity = {};
    app.state.acceleration = {};


    app.telemetry.record(app.state, app.method);


    app.telemetry.finishRun();

    app.finishPause = 1.3;

    app.analysis.refresh();

    setStatus(
        "COMPLETED: goal reached"
    );
}





double routeLength(
    const std::vector<Vector3>& points
) {
    double length = 0.0;

    for (size_t i = 1; i < points.size(); ++i) {
        length += points[i].distanceTo(
            points[i - 1]
        );
    }

    return length;
}

void replan() {
    Vector3 start = app.moving.position;

    auto began = std::chrono::steady_clock::now();
    if (app.useRRT) {
        const auto rrt=planRRT(NX,NY,NZ,app.moving,start,app.goal,
                               app.obstacles,app.safetyMargin,12000,42);
        app.rawPath=rrt.path;
        app.searchNodes=rrt.nodes;
    } else {
        AStar3D planner(NX,NY,NZ,app.moving,app.obstacles,app.safetyMargin,
                        app.physics.maxSpeed,app.physics.acceleration,
                        app.pathCriterion,app.timeWeight,app.algorithm);
        app.rawPath=planner.findPathFrom(start,app.goal);
        app.searchNodes=planner.visitedNodes();
    }
    app.searchMilliseconds=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-began).count();



    app.path = app.smooth && (app.pathCriterion == PathCriterion::Distance || app.useRRT)
        ? smoothPath(app.rawPath, app.moving, app.obstacles, app.safetyMargin)
        : app.rawPath;

    app.totalLength = routeLength(app.path);
    app.state.position = start;

    app.collision =
        sceneSeparation(
            app.moving,
            app.obstacles
        ) <= app.safetyMargin;

    if (app.collision) {
        interruptExperiment(
            "INTERRUPTED: collision with obstacle"
        );
        return;
    }

    if (app.path.empty()) {
        if (app.motionModel == MotionModel::Uniform ||
            app.motionModel == MotionModel::ConstantAcceleration) {
            setStatus("No A* path: free-flight motion remains active");
            return;
        }
        interruptExperiment("INTERRUPTED: no available path");
        return;
    }

    std::ostringstream message;

    message
        << "Path "
        << app.rawPath.size()
        << " -> "
        << app.path.size()
        << " | L = "
        << number(app.totalLength)
        << " | Nodes = "
        << app.searchNodes
        << " | "
        << number(app.searchMilliseconds)
        << " ms | criterion = "
        << (app.pathCriterion == PathCriterion::Distance ? "distance" :
            app.pathCriterion == PathCriterion::Time ? "time-est" : "combined");

    setStatus(message.str());
}

struct Progress {
    double distance = 0.0;
    double error = 1e100;
};

Progress closestOnRoute(Vector3 position) {
    Progress best;
    double covered = 0.0;

    for (size_t i = 1; i < app.path.size(); ++i) {
        Vector3 a = app.path[i - 1];
        Vector3 d = app.path[i] - a;

        double length = d.length();

        if (length < 1e-10) continue;

        double u = std::clamp(
            dot(position - a, d) / (length * length),
            0.0,
            1.0
        );

        double error =
            position.distanceTo(a + d * u);

        if (error < best.error) {
            best.distance = covered + length * u;
            best.error = error;
        }

        covered += length;
    }

    return best;
}

Vector3 positionOnRoute(double distance) {
    if (app.path.empty()) {
        return app.moving.position;
    }

    distance = std::max(0.0, distance);

    for (size_t i = 1; i < app.path.size(); ++i) {
        Vector3 a = app.path[i - 1];
        Vector3 b = app.path[i];

        double length = a.distanceTo(b);

        if (length < 1e-10) continue;

        if (distance <= length) {
            return a + (b - a) * (distance / length);
        }

        distance -= length;
    }

    return app.path.back();
}





int selectedObstacle() {
    if (!app.list) return -1;

    return int(
        SendMessageA(
            app.list,
            LB_GETCURSEL,
            0, 0
        )
    );
}

void updateLabels(
    HWND combo,
    HWND labels[3]
) {
    const char* names[3][3] = {
        {"Radius", "Unused", "Unused"},
        {"Half X", "Half Y", "Half Z"},
        {"Radius", "Height", "Unused"}
    };

    int type = std::clamp(
        int(SendMessageA(
            combo, CB_GETCURSEL, 0, 0
        )),
        0, 2
    );

    for (int i = 0; i < 3; ++i) {
        if (labels[i]) {
            SetWindowTextA(
                labels[i], names[type][i]
            );
        }
    }
}

void updateObstacleLabels() {
    updateLabels(app.obstacleType, app.obstacleLabels + 3);
    int type = int(SendMessageA(app.obstacleType, CB_GETCURSEL, 0, 0));
    for (HWND field : app.obstacleRotationEdits) {
        if (field) EnableWindow(field, type != int(ShapeType::Sphere));
    }
}

void updateMovingLabels() {
    updateLabels(app.movingType, app.movingLabels);
    int type = int(SendMessageA(app.movingType, CB_GETCURSEL, 0, 0));
    for (HWND field : app.movingRotationEdits) {
        if (field) EnableWindow(field, type != int(ShapeType::Sphere));
    }
}

void loadObstacle() {
    int index = selectedObstacle();

    if (index < 0 ||
        index >= int(app.obstacles.size())) {
        return;
    }

    app.updating = true;

    const Obstacle& o = app.obstacles[index];

    SendMessageA(
        app.obstacleType,
        CB_SETCURSEL,
        WPARAM(o.type), 0
    );

    double values[6] = {
        o.position.x,
        o.position.y,
        o.position.z,
        o.size.x,
        o.size.y,
        o.size.z
    };

    for (int i = 0; i < 6; ++i) {
        SetWindowTextA(
            app.obstacleEdits[i],
            number(values[i]).c_str()
        );
    }

    const double rotationValues[3] = {
        o.rotation.x, o.rotation.y, o.rotation.z
    };
    for (int i = 0; i < 3; ++i) {
        SetWindowTextA(app.obstacleRotationEdits[i],
                       number(rotationValues[i]).c_str());
    }
    updateObstacleLabels();
    app.updating = false;
}

void refreshList(
    int selection,
    bool reloadFields = true
) {
    if (!app.list) return;

    app.updating = true;

    SendMessageA(
        app.list,
        LB_RESETCONTENT,
        0, 0
    );

    for (size_t i = 0; i < app.obstacles.size(); ++i) {
        const auto& o = app.obstacles[i];

        std::ostringstream item;

        item
            << i + 1
            << ". "
            << shapeName(o.type)
            << " ("
            << number(o.position.x)
            << ", "
            << number(o.position.y)
            << ", "
            << number(o.position.z)
            << ")";

        std::string value = item.str();

        SendMessageA(
            app.list,
            LB_ADDSTRING,
            0,
            reinterpret_cast<LPARAM>(value.c_str())
        );
    }

    if (!app.obstacles.empty()) {
        selection = std::clamp(
            selection,
            0,
            int(app.obstacles.size()) - 1
        );

        SendMessageA(
            app.list,
            LB_SETCURSEL,
            selection, 0
        );
    }

    app.updating = false;

    if (reloadFields) loadObstacle();
}

bool validSize(
    ShapeType type,
    Vector3 size
) {
    if (type == ShapeType::Sphere) {
        return size.x > 0.0;
    }

    if (type == ShapeType::Box) {
        return size.x > 0.0 &&
               size.y > 0.0 &&
               size.z > 0.0;
    }

    return size.x > 0.0 &&
           size.y > 0.0;
}

void applyObstacleFields() {
    if (app.updating) return;

    int index = selectedObstacle();

    if (index < 0 ||
        index >= int(app.obstacles.size())) {
        return;
    }

    double values[6];

    for (int i = 0; i < 6; ++i) {
        if (!readNumber(
            app.obstacleEdits[i],
            values[i]
        )) {
            return;
        }
    }

    int type = int(SendMessageA(
        app.obstacleType,
        CB_GETCURSEL,
        0, 0
    ));

    if (type < 0 || type > 2) return;

    Obstacle obstacle{
        ShapeType(type),
        {values[0], values[1], values[2]},
        {values[3], values[4], values[5]}
    };

    if (!validSize(
        obstacle.type,
        obstacle.size
    )) {
        return;
    }

    double angles[3]{};
    for (int i = 0; i < 3; ++i) {
        if (!readNumber(app.obstacleRotationEdits[i], angles[i])) return;
    }
    obstacle.rotation = {angles[0], angles[1], angles[2]};
    app.obstacles[index] = obstacle;

    refreshList(index, false);
    replan();
}

void addObstacle() {
    int type = std::clamp(
        int(SendMessageA(
            app.obstacleType,
            CB_GETCURSEL, 0, 0
        )),
        0, 2
    );

    int index = int(app.obstacles.size());

    Obstacle obstacle;

    obstacle.type = ShapeType(type);

    obstacle.position = {
        2.0 + index % 5,
        1.0 + (index / 5) % 4,
        1.0 + index % 3
    };

    if (obstacle.type == ShapeType::Sphere) {
        obstacle.size = {0.6, 0.6, 0.6};
    }
    else if (obstacle.type == ShapeType::Box) {
        obstacle.size = {0.5, 0.5, 0.5};
    }
    else {
        obstacle.size = {0.5, 1.4, 0.0};
    }

    app.obstacles.push_back(obstacle);

    refreshList(
        int(app.obstacles.size()) - 1
    );

    replan();
}

void deleteObstacle() {
    int index = selectedObstacle();

    if (index < 0 ||
        index >= int(app.obstacles.size())) {
        return;
    }

    app.obstacles.erase(
        app.obstacles.begin() + index
    );

    refreshList(index);
    replan();
}

void resetScene() {
    app.obstacles = {
        {
            ShapeType::Sphere,
            {3, 2, 1},
            {0.8, 0, 0}
        },
        {
            ShapeType::Box,
            {5, 2, 1},
            {0.55, 0.55, 0.55}
        },
        {
            ShapeType::Cylinder,
            {7, 2, 1},
            {0.55, 1.6, 0}
        }
    };

    app.moving = {
        ShapeType::Sphere,
        app.start,
        {0.35, 0.35, 0.35}
    };

    app.physics.maxSpeed = 2.0;
    app.physics.acceleration = 1.5;
    app.physics.braking = 2.0;
    app.physics.maxTurnRate = 180.0;

    app.start = DEFAULT_START;
    app.goal = DEFAULT_GOAL;
    app.initialVelocity = {};
    app.initialAcceleration = {};
    app.integrationStep = 0.01;
    app.safetyMargin = 0.02;
    app.gravity = 9.81;
    app.linearDrag = 0.25;
    app.timeWeight = 1.0;
    app.pathCriterion = PathCriterion::Distance;
    app.algorithm = SearchAlgorithm::AStar;
    app.useRRT = false;
    app.replaying = false;
    app.motionModel = MotionModel::Guided;
    app.simulationAccumulator = 0.0;
    app.telemetry.setGoal(app.goal);

    app.method = IntegrationMethod::RK4;

    app.state = {};
    app.state.position = app.moving.position;

    app.finished = false;
    app.finishPause = 0.0;
    app.collision = false;
    app.paused = false;

    refreshList(0);
    updatePauseButton();
    replan();
}





void applyMovingFields() {
    if (app.updating) return;

    double values[3];

    for (int i = 0; i < 3; ++i) {
        if (!readNumber(
            app.movingEdits[i],
            values[i]
        )) {
            return;
        }
    }

    int type = int(SendMessageA(
        app.movingType,
        CB_GETCURSEL,
        0, 0
    ));

    if (type < 0 || type > 2) return;

    Vector3 size{
        values[0],
        values[1],
        values[2]
    };

    if (!validSize(ShapeType(type), size)) {
        return;
    }

    double angles[3]{};
    for (int i = 0; i < 3; ++i) {
        if (!readNumber(app.movingRotationEdits[i], angles[i])) return;
    }
    app.moving.type = ShapeType(type);
    app.moving.size = size;
    app.moving.rotation = {angles[0], angles[1], angles[2]};

    app.state.velocity = {};
    app.state.acceleration = {};

    replan();
}

void loadMovingFields() {
    app.updating = true;

    SendMessageA(
        app.movingType,
        CB_SETCURSEL,
        WPARAM(app.moving.type),
        0
    );

    double values[3] = {
        app.moving.size.x,
        app.moving.size.y,
        app.moving.size.z
    };

    for (int i = 0; i < 3; ++i) {
        SetWindowTextA(
            app.movingEdits[i],
            number(values[i]).c_str()
        );
    }

    const double rotationValues[3] = {
        app.moving.rotation.x, app.moving.rotation.y, app.moving.rotation.z
    };
    for (int i = 0; i < 3; ++i) {
        SetWindowTextA(app.movingRotationEdits[i],
                       number(rotationValues[i]).c_str());
    }
    updateMovingLabels();
    app.updating = false;
}





void applyPhysicsFields() {
    if (app.updating) return;

    double speed;
    double acceleration;
    double braking;
    double maxTurnRate;

    if (
        readNumber(app.turnRateEdit,maxTurnRate) && maxTurnRate>0.0 && maxTurnRate<=720.0 &&
        readNumber(app.speedEdit, speed) &&
        readNumber(app.accelEdit, acceleration) &&
        readNumber(app.brakeEdit, braking) &&
        speed > 0.0 &&
        acceleration > 0.0 &&
        braking > 0.0
    ) {
        const bool routeCostChanged =
            app.physics.maxSpeed != speed || app.physics.acceleration != acceleration;
        app.physics.maxSpeed = speed;
        app.physics.acceleration = acceleration;
        app.physics.braking = braking;
        app.physics.maxTurnRate = maxTurnRate;
        if (routeCostChanged && app.pathCriterion != PathCriterion::Distance &&
            app.telemetry.run() > 0) {
            replan();
        }
    }

    int selectedMethod = int(
        SendMessageA(
            app.methodCombo,
            CB_GETCURSEL,
            0, 0
        )
    );

    IntegrationMethod nextMethod =
        selectedMethod == 0
        ? IntegrationMethod::Euler
        : IntegrationMethod::RK4;



    if (nextMethod != app.method) {
        app.method = nextMethod;

        if (app.telemetry.run() > 0) {
            newRun();
        }
    }
}

void loadPhysicsFields() {
    app.updating = true;

    SetWindowTextA(
        app.speedEdit,
        number(app.physics.maxSpeed).c_str()
    );

    SetWindowTextA(
        app.accelEdit,
        number(app.physics.acceleration).c_str()
    );

    SetWindowTextA(
        app.brakeEdit,
        number(app.physics.braking).c_str()
    );

    SetWindowTextA(app.turnRateEdit,number(app.physics.maxTurnRate).c_str());

    SendMessageA(
        app.methodCombo,
        CB_SETCURSEL,
        app.method == IntegrationMethod::Euler
            ? 0 : 1,
        0
    );

    app.updating = false;
}





void loadRouteFields() {
    app.updating = true;
    const double from[3] = {app.start.x, app.start.y, app.start.z};
    const double to[3] = {app.goal.x, app.goal.y, app.goal.z};
    for (int i = 0; i < 3; ++i) {
        SetWindowTextA(app.routeStartEdits[i], number(from[i]).c_str());
        SetWindowTextA(app.routeGoalEdits[i], number(to[i]).c_str());
    }
    SendMessageA(app.criterionCombo, CB_SETCURSEL, WPARAM(app.pathCriterion), 0);
    SendMessageA(app.routeAlgorithmCombo,CB_SETCURSEL,
        app.useRRT?2:app.algorithm==SearchAlgorithm::Dijkstra?1:0,0);
    SetWindowTextA(app.timeWeightEdit, number(app.timeWeight).c_str());
    app.updating = false;
}

void applyRouteFields() {
    double start[3], goal[3], weight;
    for (int i = 0; i < 3; ++i) {
        if (!readNumber(app.routeStartEdits[i], start[i]) ||
            !readNumber(app.routeGoalEdits[i], goal[i])) {
            setStatus("Invalid Start/Goal coordinates"); return;
        }
    }
    if (!readNumber(app.timeWeightEdit, weight) || weight < 0.0 || weight > 100.0) {
        setStatus("Time weight must be between 0 and 100"); return;
    }
    const int bounds[3] = {NX, NY, NZ};
    for (int i = 0; i < 3; ++i) {
        if (start[i] < 0.0 || start[i] > double(bounds[i]-1) ||
            goal[i] < 0.0 || goal[i] > double(bounds[i]-1)) {
            setStatus("Start and Goal must be inside grid bounds.");
            return;
        }
    }
    const int selected = int(SendMessageA(app.criterionCombo, CB_GETCURSEL, 0, 0));
    if (selected < 0 || selected > 2) return;
    app.start = {start[0], start[1], start[2]};
    app.goal = {goal[0],goal[1],goal[2]};
    app.pathCriterion = PathCriterion(selected);
    app.timeWeight = weight;
    const int search=int(SendMessageA(app.routeAlgorithmCombo,CB_GETCURSEL,0,0));
    app.useRRT=search==2;
    app.algorithm=search==1?SearchAlgorithm::Dijkstra:SearchAlgorithm::AStar;
    newRun();
}





void loadModelFields() {
    app.updating = true;
    const double velocity[3] = {
        app.initialVelocity.x, app.initialVelocity.y, app.initialVelocity.z
    };
    const double acceleration[3] = {
        app.initialAcceleration.x, app.initialAcceleration.y, app.initialAcceleration.z
    };
    for (int i = 0; i < 3; ++i) {
        SetWindowTextA(app.initialVelocityEdits[i], number(velocity[i]).c_str());
        SetWindowTextA(app.initialAccelerationEdits[i], number(acceleration[i]).c_str());
    }
    SetWindowTextA(app.integrationStepEdit, number(app.integrationStep).c_str());
    SetWindowTextA(app.safetyMarginEdit, number(app.safetyMargin).c_str());
    SetWindowTextA(app.gravityEdit, number(app.gravity).c_str());
    SetWindowTextA(app.dragEdit, number(app.linearDrag).c_str());
    SendMessageA(app.motionModelCombo, CB_SETCURSEL, WPARAM(app.motionModel), 0);
    app.updating = false;
}

void applyModelFields() {
    double v[3], a[3], step, margin, gravity, drag;
    for (int i = 0; i < 3; ++i) {
        if (!readNumber(app.initialVelocityEdits[i], v[i]) ||
            !readNumber(app.initialAccelerationEdits[i], a[i])) {
            setStatus("Initial velocity/acceleration must be numeric"); return;
        }
        if (std::abs(v[i]) > 100.0 || std::abs(a[i]) > 100.0) {
            setStatus("Initial conditions must be between -100 and 100"); return;
        }
    }
    if (!readNumber(app.integrationStepEdit, step) ||
        !readNumber(app.safetyMarginEdit, margin) ||
        !readNumber(app.gravityEdit, gravity) ||
        !readNumber(app.dragEdit, drag) ||
        step < 0.001 || step > 0.1 || margin < 0.0 || margin > 2.0 ||
        gravity < 0.0 || gravity > 100.0 || drag < 0.0 || drag > 20.0) {
        setStatus("dt: 0.001..0.1, margin: 0..2, gravity: 0..100, drag: 0..20");
        return;
    }
    const int selected = int(SendMessageA(app.motionModelCombo, CB_GETCURSEL, 0, 0));
    if (selected < 0 || selected > 5) return;
    app.initialVelocity = {v[0], v[1], v[2]};
    app.initialAcceleration = {a[0], a[1], a[2]};
    app.integrationStep = step;
    app.safetyMargin = margin;
    app.gravity = gravity;
    app.linearDrag = drag;
    app.motionModel = MotionModel(selected);
    newRun();
    if (app.motionModel == MotionModel::Uniform &&
        app.initialVelocity.length() < 1e-10) {
        setStatus("Uniform model: enter a non-zero initial velocity");
    }
}

void newRun() {
    app.telemetry.interruptRun();
    app.simulationAccumulator = 0.0;
    app.replaying = false;
    app.replaySamples.clear();
    app.moving.position = app.start;

    app.state = {};
    app.state.position = app.moving.position;
    app.state.velocity = app.initialVelocity;
    app.state.acceleration = app.initialAcceleration;
    app.telemetry.setGoal(app.goal);

    app.finished = false;
    app.finishPause = 0.0;
    app.collision = false;
    app.paused = false;

    updatePauseButton();

    app.telemetry.newRun();

    replan();

    if (!app.collision && !app.paused) {
        app.telemetry.record(
            app.state,
            app.method
        );
    }

    app.analysis.refresh();
}





bool projectPoint(
    Vector3 position,
    double& sx,
    double& sy
) {
    GLdouble model[16]{};
    GLdouble projection[16]{};
    GLint viewport[4]{};

    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    double point[4] = {
        position.x,
        position.y,
        position.z,
        1.0
    };

    double eye[4]{};
    double clip[4]{};

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            eye[row] +=
                model[col * 4 + row] * point[col];
        }
    }

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            clip[row] +=
                projection[col * 4 + row] * eye[col];
        }
    }

    if (clip[3] <= 0.0) return false;

    sx = viewport[0] +
        (clip[0] / clip[3] + 1.0) *
        viewport[2] / 2.0;

    sy = viewport[3] -
        (clip[1] / clip[3] + 1.0) *
        viewport[3] / 2.0;

    return true;
}





void beginDrag(
    HWND hwnd,
    int mouseX,
    int mouseY
) {
    int selected = -1;
    double nearest = 45.0;

    for (int i = 0;
         i < int(app.obstacles.size());
         ++i) {
        double sx;
        double sy;

        if (!projectPoint(
            app.obstacles[i].position,
            sx, sy
        )) {
            continue;
        }

        double distance = std::hypot(
            sx - mouseX,
            sy - mouseY
        );

        if (distance < nearest) {
            nearest = distance;
            selected = i;
        }
    }

    if (selected < 0) return;

    app.dragging = true;
    app.draggedIndex = selected;
    app.lastMouse = {mouseX, mouseY};

    SetCapture(hwnd);
    refreshList(selected);
}

void dragObstacle(
    int mouseX,
    int mouseY
) {
    int index = app.draggedIndex;

    if (!app.dragging ||
        index < 0 ||
        index >= int(app.obstacles.size())) {
        return;
    }

    double dx = mouseX - app.lastMouse.x;
    double dy = mouseY - app.lastMouse.y;

    app.lastMouse = {mouseX, mouseY};

    double yaw =
        app.camera.yaw * PI / 180.0;

    double pitch =
        app.camera.pitch * PI / 180.0;

    double scale =
        app.camera.distance * 0.002;

    Obstacle& obstacle = app.obstacles[index];

    if (
        GetAsyncKeyState(VK_SHIFT) & 0x8000
    ) {
        obstacle.position.y -= dy * scale;
    }
    else {
        double factor = std::max(
            0.2,
            std::abs(std::sin(pitch))
        );

        obstacle.position.x +=
            dx * scale * std::cos(yaw) +
            dy * scale * std::sin(yaw) / factor;

        obstacle.position.z +=
            dx * scale * std::sin(yaw) -
            dy * scale * std::cos(yaw) / factor;
    }

    refreshList(index);
    replan();
}





LRESULT CALLBACK sceneProc(
    HWND hwnd,
    UINT msg,
    WPARAM wp,
    LPARAM lp
) {
    switch (msg) {
        case WM_LBUTTONDOWN:
            beginDrag(
                hwnd,
                GET_X_LPARAM(lp),
                GET_Y_LPARAM(lp)
            );
            return 0;

        case WM_MOUSEMOVE:
            if (app.dragging && (wp & MK_LBUTTON)) {
                dragObstacle(
                    GET_X_LPARAM(lp),
                    GET_Y_LPARAM(lp)
                );
            }
            return 0;

        case WM_LBUTTONUP:
        case WM_CAPTURECHANGED:
            app.dragging = false;
            app.draggedIndex = -1;

            if (
                msg == WM_LBUTTONUP &&
                GetCapture() == hwnd
            ) {
                ReleaseCapture();
            }
            return 0;

        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;

        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;

        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) {
                DestroyWindow(hwnd);
                return 0;
            }
            break;
    }

    return DefWindowProcA(
        hwnd, msg, wp, lp
    );
}





LRESULT CALLBACK panelProc(
    HWND hwnd,
    UINT msg,
    WPARAM wp,
    LPARAM lp
) {
    if (msg == WM_CLOSE) {
        ShowWindow(hwnd, SW_HIDE);
        return 0;
    }

    if (msg == WM_COMMAND) {
        int id = LOWORD(wp);
        int event = HIWORD(wp);

        if (event == BN_CLICKED) {
            if (id == ID_TAB_OBSTACLES) {
                showSettingsTab(0);
                return 0;
            }

            if (id == ID_TAB_MOVING) {
                showSettingsTab(1);
                return 0;
            }

            if (id == ID_TAB_PHYSICS) {
                showSettingsTab(2);
                return 0;
            }
            if (id == ID_TAB_ROUTE) {
                showSettingsTab(3);
                return 0;
            }
            if (id == ID_TAB_MODELS) {
                showSettingsTab(4);
                return 0;
            }
        }

        if (
            id == ID_LIST &&
            event == LBN_SELCHANGE
        ) {
            loadObstacle();
            return 0;
        }

        if (
            id == ID_OBSTACLE_TYPE &&
            event == CBN_SELCHANGE
        ) {
            updateObstacleLabels();
            applyObstacleFields();
            return 0;
        }

        if (
            id >= ID_X &&
            id <= ID_C &&
            event == EN_CHANGE
        ) {
            applyObstacleFields();
            return 0;
        }

        if (id >= ID_OB_ROT_X && id <= ID_OB_ROT_Z && event == EN_CHANGE) {
            applyObstacleFields();
            return 0;
        }

        if (id >= ID_MOV_ROT_X && id <= ID_MOV_ROT_Z && event == EN_CHANGE) {
            applyMovingFields();
            return 0;
        }

        if (
            id == ID_MOVING_TYPE &&
            event == CBN_SELCHANGE
        ) {
            updateMovingLabels();
            applyMovingFields();
            return 0;
        }

        if (
            id >= ID_MOVING_A &&
            id <= ID_MOVING_C &&
            event == EN_CHANGE
        ) {
            applyMovingFields();
            return 0;
        }

        if (
            id >= ID_SPEED &&
            id <= ID_BRAKE &&
            event == EN_CHANGE
        ) {
            applyPhysicsFields();
            return 0;
        }

        if (id==ID_MAX_TURN_RATE && event==EN_CHANGE) {
            applyPhysicsFields();return 0;
        }

        if (
            id == ID_METHOD &&
            event == CBN_SELCHANGE
        ) {
            applyPhysicsFields();
            return 0;
        }

        if (event == BN_CLICKED) {
            switch (id) {
                case ID_ADD:
                    addObstacle();
                    return 0;

                case ID_DELETE:
                    deleteObstacle();
                    return 0;

                case ID_RESET:
                    app.telemetry.interruptRun();

                    resetScene();
                    loadMovingFields();
                    loadPhysicsFields();
                    loadRouteFields();
                    loadModelFields();
                    newRun();
                    return 0;

                case ID_EXPORT:
                    if (app.telemetry.exportCSV(
                        "3d_moves_telemetry.csv"
                    )) {
                        setStatus("Telemetry CSV exported");
                    }
                    else {
                        setStatus("ERROR: telemetry export");
                    }
                    return 0;

                case ID_SUMMARY:
                    if (app.telemetry.exportSummaryCSV(
                        "3d_moves_summary.csv"
                    )) {
                        setStatus("Summary CSV exported");
                    }
                    else {
                        setStatus("ERROR: summary export");
                    }
                    return 0;

                case ID_CLEAR:
                    app.telemetry.clear();
                    newRun();

                    setStatus("Telemetry cleared");
                    return 0;

                case ID_NEW_RUN:
                    newRun();
                    return 0;

                case ID_APPLY_ROUTE:
                    applyRouteFields();
                    return 0;

                case ID_APPLY_MODEL:
                    applyModelFields();
                    return 0;

                case ID_ANALYSIS:
                    app.analysis.toggle();
                    return 0;

                case ID_REPLAY:
                    startReplay();return 0;
                case ID_STEP: {
                    const bool was=app.paused;
                    app.paused=false;
                    advanceOneStep(app.integrationStep);
                    app.paused=was;
                    updatePauseButton();
                    return 0;
                }
                case ID_PAUSE:
                    app.paused = !app.paused;
                    updatePauseButton();
                    return 0;
            }
        }
    }

    return DefWindowProcA(
        hwnd, msg, wp, lp
    );
}





HWND control(
    HWND parent,
    const char* cls,
    const char* caption,
    DWORD style,
    int x,
    int y,
    int width,
    int height,
    int id = 0
) {
    return CreateWindowExA(
        0,
        cls,
        caption,
        WS_CHILD | WS_VISIBLE | style,
        x, y, width, height,
        parent,
        reinterpret_cast<HMENU>(
            static_cast<INT_PTR>(id)
        ),
        GetModuleHandleA(nullptr),
        nullptr
    );
}

HWND label(
    HWND parent,
    const char* caption,
    int x,
    int y,
    int width = 120,
    int height = 22
) {
    return control(
        parent,
        "STATIC", caption, 0,
        x, y, width, height
    );
}

HWND edit(
    HWND parent,
    int id,
    int x,
    int y,
    int width = 245
) {
    return control(
        parent,
        "EDIT", "0",
        WS_BORDER | ES_AUTOHSCROLL,
        x, y, width, 25,
        id
    );
}

void fillCombo(
    HWND combo,
    std::initializer_list<const char*> values
) {
    for (const char* value : values) {
        SendMessageA(
            combo,
            CB_ADDSTRING,
            0,
            reinterpret_cast<LPARAM>(value)
        );
    }

    SendMessageA(combo, CB_SETCURSEL, 0, 0);
}

void registerTabControl(HWND hwnd, int tab) {
    if (hwnd && tab >= 0 && tab < 5) {
        app.tabControls[tab].push_back(hwnd);
    }
}

HWND tabLabel(
    int tab,
    const char* caption,
    int x,
    int y,
    int width = 120,
    int height = 22
) {
    HWND hwnd = label(
        app.panel,
        caption,
        x, y, width, height
    );

    registerTabControl(hwnd, tab);
    return hwnd;
}

HWND tabEdit(
    int tab,
    int id,
    int x,
    int y,
    int width = 245
) {
    HWND hwnd = edit(
        app.panel,
        id,
        x, y, width
    );

    registerTabControl(hwnd, tab);
    return hwnd;
}

HWND tabButton(
    int tab,
    int id,
    const char* caption,
    int x,
    int y,
    int width = 120
) {
    HWND hwnd = control(
        app.panel,
        "BUTTON",
        caption,
        BS_PUSHBUTTON,
        x, y, width, 30,
        id
    );

    registerTabControl(hwnd, tab);
    return hwnd;
}

HWND tabCombo(
    int tab,
    int id,
    int x,
    int y,
    int width = 245
) {
    HWND hwnd = control(
        app.panel,
        "COMBOBOX",
        "",
        CBS_DROPDOWNLIST | WS_VSCROLL,
        x, y, width, 130,
        id
    );

    registerTabControl(hwnd, tab);
    return hwnd;
}





void showSettingsTab(int tab) {
    if (tab < 0 || tab >= 5) return;

    app.activeTab = tab;

    for (int i = 0; i < 5; ++i) {
        for (HWND hwnd : app.tabControls[i]) {
            if (hwnd) {
                ShowWindow(
                    hwnd,
                    i == tab ? SW_SHOW : SW_HIDE
                );
            }
        }

        if (app.tabButtons[i]) {
            EnableWindow(
                app.tabButtons[i],
                i != tab
            );
        }
    }

    InvalidateRect(
        app.panel, nullptr, TRUE
    );
}





void createPanel(HINSTANCE instance) {
    WNDCLASSA wc{};

    wc.lpfnWndProc = panelProc;
    wc.hInstance = instance;

    wc.lpszClassName =
        "3DMovesFinalSettingsV1";

    wc.hbrBackground =
        HBRUSH(COLOR_BTNFACE + 1);

    wc.hCursor =
        LoadCursor(nullptr, IDC_ARROW);

    RegisterClassA(&wc);

    app.panel = CreateWindowExA(
        0,
        wc.lpszClassName,
        "3D_MOVES - Settings",
        WS_OVERLAPPED |
        WS_CAPTION |
        WS_SYSMENU |
        WS_MINIMIZEBOX |
        WS_VISIBLE,
        1015, 40,
        430, 755,
        nullptr, nullptr,
        instance, nullptr
    );

    if (!app.panel) return;

    const char* tabs[5] = {"Obstacles", "Moving", "Physics", "Route", "Models"};
    const int tabIds[5] = {
        ID_TAB_OBSTACLES, ID_TAB_MOVING, ID_TAB_PHYSICS,
        ID_TAB_ROUTE, ID_TAB_MODELS
    };
    for (int i = 0; i < 5; ++i) {
        app.tabButtons[i] = control(
            app.panel, "BUTTON", tabs[i], BS_PUSHBUTTON,
            10 + i * 81, 12, 78, 32, tabIds[i]
        );
    }





    constexpr int OBSTACLES = 0;

    tabLabel(
        OBSTACLES,
        "OBSTACLES",
        15, 62, 300
    );

    app.list = control(
        app.panel,
        "LISTBOX", "",
        WS_BORDER |
        WS_VSCROLL |
        LBS_NOTIFY,
        15, 90, 385, 115,
        ID_LIST
    );

    registerTabControl(
        app.list, OBSTACLES
    );

    tabLabel(
        OBSTACLES,
        "Shape",
        15, 225
    );

    app.obstacleType = tabCombo(
        OBSTACLES,
        ID_OBSTACLE_TYPE,
        145, 220, 255
    );

    fillCombo(
        app.obstacleType,
        {"Sphere", "Box", "Cylinder"}
    );

    const char* obstacleNames[6] = {
        "Position X",
        "Position Y",
        "Position Z",
        "Radius",
        "Unused",
        "Unused"
    };

    for (int i = 0; i < 6; ++i) {
        int y = 252 + i * 31;

        app.obstacleLabels[i] = tabLabel(
            OBSTACLES,
            obstacleNames[i],
            15, y + 4
        );

        app.obstacleEdits[i] = tabEdit(
            OBSTACLES,
            ID_X + i,
            145, y, 255
        );
    }

    const char* rotationNames[3] = {
        "Rotation X (deg)", "Rotation Y (deg)", "Rotation Z (deg)"
    };
    for (int i = 0; i < 3; ++i) {
        int y = 438 + i * 31;
        tabLabel(OBSTACLES, rotationNames[i], 15, y + 4, 125);
        app.obstacleRotationEdits[i] =
            tabEdit(OBSTACLES, ID_OB_ROT_X + i, 145, y, 255);
    }

    tabButton(
        OBSTACLES,
        ID_ADD, "Add",
        15, 545, 115
    );

    tabButton(
        OBSTACLES,
        ID_DELETE, "Delete",
        145, 545, 115
    );

    tabButton(
        OBSTACLES,
        ID_RESET, "Reset",
        275, 545, 125
    );

    tabLabel(
        OBSTACLES,
        "Select an obstacle to edit it.",
        15, 590, 380
    );

    tabLabel(
        OBSTACLES,
        "Drag obstacles in the 3D scene.",
        15, 614, 380
    );





    constexpr int MOVING = 1;

    tabLabel(
        MOVING,
        "MOVING OBJECT",
        15, 70, 300
    );

    tabLabel(
        MOVING,
        "Shape",
        15, 120
    );

    app.movingType = tabCombo(
        MOVING,
        ID_MOVING_TYPE,
        145, 115, 255
    );

    fillCombo(
        app.movingType,
        {"Sphere", "Box", "Cylinder"}
    );

    for (int i = 0; i < 3; ++i) {
        int y = 175 + i * 48;

        app.movingLabels[i] = tabLabel(
            MOVING,
            "Size",
            15, y + 4
        );

        app.movingEdits[i] = tabEdit(
            MOVING,
            ID_MOVING_A + i,
            145, y, 255
        );
    }

    for (int i = 0; i < 3; ++i) {
        int y = 330 + i * 43;
        tabLabel(MOVING, rotationNames[i], 15, y + 4, 125);
        app.movingRotationEdits[i] =
            tabEdit(MOVING, ID_MOV_ROT_X + i, 145, y, 255);
    }

    tabLabel(
        MOVING,
        "Changes are applied automatically.",
        15, 485, 385
    );

    tabLabel(
        MOVING,
        "A* recalculates the route after edits.",
        15, 510, 385
    );





    constexpr int PHYSICS = 2;

    tabLabel(
        PHYSICS,
        "PHYSICS SETTINGS",
        15, 65, 300
    );

    tabLabel(
        PHYSICS,
        "Max speed",
        15, 105
    );

    app.speedEdit = tabEdit(
        PHYSICS,
        ID_SPEED,
        145, 100, 255
    );

    tabLabel(
        PHYSICS,
        "Acceleration",
        15, 150
    );

    app.accelEdit = tabEdit(
        PHYSICS,
        ID_ACCEL,
        145, 145, 255
    );

    tabLabel(
        PHYSICS,
        "Braking",
        15, 195
    );

    app.brakeEdit = tabEdit(
        PHYSICS,
        ID_BRAKE,
        145, 190, 255
    );

    tabLabel(
        PHYSICS,
        "Method",
        15, 240
    );

    app.methodCombo = tabCombo(
        PHYSICS,
        ID_METHOD,
        145, 235, 145
    );

    fillCombo(
        app.methodCombo,
        {"Euler", "RK4"}
    );

    app.pauseButton = tabButton(
        PHYSICS,
        ID_PAUSE, "Pause",
        305, 235, 95
    );

    tabLabel(
        PHYSICS,
        "EXPERIMENTS",
        15, 290, 300
    );

    tabButton(
        PHYSICS,
        ID_NEW_RUN, "New run",
        15, 325, 120
    );

    tabButton(
        PHYSICS,
        ID_CLEAR, "Clear data",
        145, 325, 120
    );

    tabButton(
        PHYSICS,
        ID_EXPORT, "Export CSV",
        275, 325, 125
    );

    tabButton(
        PHYSICS,
        ID_SUMMARY, "Export summary CSV",
        15, 370, 385
    );

    tabButton(
        PHYSICS,
        ID_ANALYSIS, "Show graphs (C++)",
        15, 415, 385
    );

    tabLabel(
        PHYSICS,
        "STATUS",
        15, 465, 300
    );

    app.status = tabLabel(
        PHYSICS,
        "Ready",
        15, 490, 390, 65
    );
    tabLabel(PHYSICS,"Max turn rate (deg/s)",15,562,145);
    app.turnRateEdit=tabEdit(PHYSICS,ID_MAX_TURN_RATE,165,557,235);
    app.replayButton=tabButton(PHYSICS,ID_REPLAY,"Replay last run",15,603,185);
    app.stepButton=tabButton(PHYSICS,ID_STEP,"Single step",215,603,185);





    constexpr int ROUTE = 3;
    tabLabel(ROUTE, "START (continuous grid coordinates)", 15, 63, 380);
    const char* axes[3] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i) {
        int y = 95 + i * 36;
        tabLabel(ROUTE, axes[i], 15, y + 3, 115);
        app.routeStartEdits[i] = tabEdit(ROUTE, ID_START_X + i, 145, y, 255);
    }
    tabLabel(ROUTE, "GOAL (continuous grid coordinates)", 15, 211, 380);
    for (int i = 0; i < 3; ++i) {
        int y = 243 + i * 36;
        tabLabel(ROUTE, axes[i], 15, y + 3, 115);
        app.routeGoalEdits[i] = tabEdit(ROUTE, ID_GOAL_X + i, 145, y, 255);
    }
    tabLabel(ROUTE, "Optimization criterion", 15, 365, 125);
    app.criterionCombo = tabCombo(ROUTE, ID_ROUTE_CRITERION, 145, 360, 255);
    fillCombo(app.criterionCombo,
              {"Shortest distance", "Estimated travel time", "Combined"});
    tabLabel(ROUTE, "Time weight", 15, 411, 125);
    app.timeWeightEdit = tabEdit(ROUTE, ID_ROUTE_WEIGHT, 145, 406, 255);
    tabButton(ROUTE, ID_APPLY_ROUTE, "Apply route and new run", 15, 468, 385);
    tabLabel(ROUTE,"Search algorithm",15,523,125);
    app.routeAlgorithmCombo=tabCombo(ROUTE,ID_ROUTE_ALGORITHM,145,518,255);
    fillCombo(app.routeAlgorithmCombo,{"A*", "Dijkstra", "RRT (experimental)"});
    tabLabel(ROUTE,"Time criterion is an estimate; RRT ignores this cost.",
             15,575,390,28);
    tabLabel(ROUTE,"A*/Dijkstra use 6-neighbor grid and turn penalties.",
             15,601,390,26);
    app.routeStatus = tabLabel(ROUTE, "Ready", 15, 633, 390, 26);




    constexpr int MODELS = 4;
    tabLabel(MODELS, "INITIAL VELOCITY (units/s)", 15, 60, 380);
    for (int i = 0; i < 3; ++i) {
        int y = 88 + i * 32;
        tabLabel(MODELS, axes[i], 15, y + 4, 115);
        app.initialVelocityEdits[i] = tabEdit(MODELS, ID_VEL_X + i, 145, y, 255);
    }
    tabLabel(MODELS, "INITIAL ACCELERATION (units/s^2)", 15, 190, 380);
    for (int i = 0; i < 3; ++i) {
        int y = 217 + i * 32;
        tabLabel(MODELS, axes[i], 15, y + 4, 115);
        app.initialAccelerationEdits[i] = tabEdit(
            MODELS, ID_INITIAL_ACCEL_X + i, 145, y, 255);
    }
    tabLabel(MODELS, "Fixed dt (seconds)", 15, 324, 125);
    app.integrationStepEdit = tabEdit(MODELS, ID_TIME_STEP, 145, 320, 255);
    tabLabel(MODELS, "Safety margin", 15, 357, 125);
    app.safetyMarginEdit = tabEdit(MODELS, ID_SAFETY_MARGIN, 145, 353, 255);
    tabLabel(MODELS, "Motion model", 15, 391, 125);
    app.motionModelCombo = tabCombo(MODELS, ID_MOTION_MODEL, 145, 386, 255);
    fillCombo(app.motionModelCombo, {
        "Guided A*", "Uniform velocity", "Constant acceleration",
        "Guided + gravity", "Guided + linear drag", "Guided + gravity + drag"
    });
    tabLabel(MODELS, "Gravity (down Y)", 15, 430, 125);
    app.gravityEdit = tabEdit(MODELS, ID_GRAVITY, 145, 425, 255);
    tabLabel(MODELS, "Linear drag k", 15, 464, 125);
    app.dragEdit = tabEdit(MODELS, ID_DRAG, 145, 459, 255);
    tabButton(MODELS, ID_APPLY_MODEL, "Apply models and new run", 15, 510, 385);
    tabLabel(MODELS, "Uniform and Constant acceleration are free-flight modes.",
             15, 557, 390, 33);
    tabLabel(MODELS, "In free-flight mode A* is a reference, not steering.",
             15, 588, 390, 26);
    app.modelStatus = tabLabel(MODELS, "Ready", 15, 620, 390, 42);

    HFONT font = HFONT(
        GetStockObject(DEFAULT_GUI_FONT)
    );

    EnumChildWindows(
        app.panel,
        [](HWND child, LPARAM value) -> BOOL {
            SendMessageA(
                child,
                WM_SETFONT,
                WPARAM(value),
                TRUE
            );
            return TRUE;
        },
        LPARAM(font)
    );

    showSettingsTab(0);
}





void drawSphere(
    Vector3 position,
    double radius,
    float red,
    float green,
    float blue
) {
    constexpr int stacks = 16;
    constexpr int segments = 28;

    glColor3f(red, green, blue);

    for (int i = 0; i < stacks; ++i) {
        double lat1 =
            -PI / 2.0 + PI * i / stacks;

        double lat2 =
            -PI / 2.0 + PI * (i + 1) / stacks;

        glBegin(GL_QUAD_STRIP);

        for (int j = 0; j <= segments; ++j) {
            double lon =
                2.0 * PI * j / segments;

            double c = std::cos(lon);
            double s = std::sin(lon);

            glVertex3d(
                position.x +
                    radius * std::cos(lat1) * c,
                position.y +
                    radius * std::sin(lat1),
                position.z +
                    radius * std::cos(lat1) * s
            );

            glVertex3d(
                position.x +
                    radius * std::cos(lat2) * c,
                position.y +
                    radius * std::sin(lat2),
                position.z +
                    radius * std::cos(lat2) * s
            );
        }

        glEnd();
    }
}





void drawBox(
    const Obstacle& o,
    float red,
    float green,
    float blue
) {
    const auto& p = o.position;
    const auto& h = o.size;

    double vertices[8][3] = {
        {p.x-h.x, p.y-h.y, p.z-h.z},
        {p.x+h.x, p.y-h.y, p.z-h.z},
        {p.x+h.x, p.y+h.y, p.z-h.z},
        {p.x-h.x, p.y+h.y, p.z-h.z},
        {p.x-h.x, p.y-h.y, p.z+h.z},
        {p.x+h.x, p.y-h.y, p.z+h.z},
        {p.x+h.x, p.y+h.y, p.z+h.z},
        {p.x-h.x, p.y+h.y, p.z+h.z}
    };

    const int faces[6][4] = {
        {0, 3, 2, 1},
        {4, 5, 6, 7},
        {0, 4, 7, 3},
        {1, 2, 6, 5},
        {3, 7, 6, 2},
        {0, 1, 5, 4}
    };

    for (int i = 0; i < 6; ++i) {
        float shade = 1.0f - 0.07f * i;

        glColor3f(
            red * shade,
            green * shade,
            blue * shade
        );

        glBegin(GL_QUADS);

        for (int j = 0; j < 4; ++j) {
            glVertex3dv(
                vertices[faces[i][j]]
            );
        }

        glEnd();
    }
}





void drawCylinder(
    const Obstacle& o,
    float red,
    float green,
    float blue
) {
    double bottom =
        o.position.y - o.size.y / 2.0;

    double top =
        o.position.y + o.size.y / 2.0;

    constexpr int segments = 36;

    glColor3f(red, green, blue);

    glBegin(GL_QUAD_STRIP);

    for (int i = 0; i <= segments; ++i) {
        double angle =
            2.0 * PI * i / segments;

        double x =
            o.position.x +
            o.size.x * std::cos(angle);

        double z =
            o.position.z +
            o.size.x * std::sin(angle);

        glVertex3d(x, bottom, z);
        glVertex3d(x, top, z);
    }

    glEnd();

    for (int cap = 0; cap < 2; ++cap) {
        float shade = cap ? 1.0f : 0.7f;

        glColor3f(
            red * shade,
            green * shade,
            blue * shade
        );

        glBegin(GL_TRIANGLE_FAN);

        glVertex3d(
            o.position.x,
            cap ? top : bottom,
            o.position.z
        );

        for (int i = 0; i <= segments; ++i) {
            double angle =
                2.0 * PI *
                (cap ? i : segments - i) /
                segments;

            glVertex3d(
                o.position.x +
                    o.size.x * std::cos(angle),
                cap ? top : bottom,
                o.position.z +
                    o.size.x * std::sin(angle)
            );
        }

        glEnd();
    }
}

void drawShape(const Obstacle& o, float red, float green, float blue) {
    if (o.type == ShapeType::Sphere) {
        drawSphere(o.position, o.size.x, red, green, blue);
        return;
    }

    glPushMatrix();
    glTranslated(o.position.x, o.position.y, o.position.z);

    glRotated(o.rotation.z, 0.0, 0.0, 1.0);
    glRotated(o.rotation.y, 0.0, 1.0, 0.0);
    glRotated(o.rotation.x, 1.0, 0.0, 0.0);

    Obstacle local = o;
    local.position = {0.0, 0.0, 0.0};
    local.rotation = {0.0, 0.0, 0.0};
    if (o.type == ShapeType::Box) {
        drawBox(local, red, green, blue);
    } else {
        drawCylinder(local, red, green, blue);
    }
    glPopMatrix();
}





void drawGrid() {
    glColor3f(0.3f, 0.3f, 0.35f);

    glBegin(GL_LINES);

    for (int x = 0; x < NX; ++x) {
        glVertex3d(x, 0, 0);
        glVertex3d(x, 0, NZ - 1);
    }

    for (int z = 0; z < NZ; ++z) {
        glVertex3d(0, 0, z);
        glVertex3d(NX - 1, 0, z);
    }

    glEnd();

    glBegin(GL_LINES);

    glColor3f(1, 0, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(NX, 0, 0);

    glColor3f(0, 1, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(0, NY, 0);

    glColor3f(0, 0.5f, 1);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, NZ);

    glEnd();
}





void render(int width, int height) {
    glViewport(0, 0, width, height);

    glClear(
        GL_COLOR_BUFFER_BIT |
        GL_DEPTH_BUFFER_BIT
    );

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double aspect = double(width) / height;

    glFrustum(
        -0.7 * aspect,
         0.7 * aspect,
        -0.7,
         0.7,
         1.0,
         100.0
    );

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslated(
        0, 0,
        -app.camera.distance
    );

    glRotated(
        app.camera.pitch,
        1, 0, 0
    );

    glRotated(
        app.camera.yaw,
        0, 1, 0
    );

    glTranslated(
        -4.5,
        -2.7,
        -2.5
    );

    drawGrid();

    int selected = selectedObstacle();

    for (int i = 0;
         i < int(app.obstacles.size());
         ++i) {
        const auto& obstacle =
            app.obstacles[i];

        if (i == selected) {
            drawShape(
                obstacle,
                1.0f, 0.9f, 0.25f
            );
        }
        else if (
            obstacle.type == ShapeType::Sphere
        ) {
            drawShape(
                obstacle,
                0.95f, 0.2f, 0.2f
            );
        }
        else if (
            obstacle.type == ShapeType::Box
        ) {
            drawShape(
                obstacle,
                0.95f, 0.55f, 0.15f
            );
        }
        else {
            drawShape(
                obstacle,
                0.65f, 0.35f, 0.9f
            );
        }
    }

    if (!app.path.empty()) {
        glColor3f(
            1.0f, 0.85f, 0.2f
        );

        glLineWidth(3.0f);

        glBegin(GL_LINE_STRIP);

        for (Vector3 point : app.path) {
            glVertex3d(
                point.x,
                point.y,
                point.z
            );
        }

        glEnd();
        glLineWidth(1.0f);
    }

    drawSphere(app.start, 0.14, 0.15f, 0.95f, 0.95f);
    drawSphere(
        app.goal,
        0.22,
        0.2f, 1.0f, 0.3f
    );

    if (app.collision) {
        drawShape(
            app.moving,
            1.0f, 0.1f, 0.1f
        );
    }
    else {
        drawShape(app.moving,0.2f,0.8f,1.0f);
    }

    if(app.state.velocity.length()>1e-6) {
        const Vector3 p=app.moving.position;
        const Vector3 v=app.state.velocity;
        const Vector3 end=p+v*0.38;
        const Vector3 dir=normalized(v);
        const Vector3 projected=Vector3{-dir.z,0,dir.x};
        const Vector3 side=projected.length()>1e-8?normalized(projected):Vector3{1,0,0};
        glColor3f(1.0f,1.0f,0.18f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        glVertex3d(p.x,p.y,p.z);glVertex3d(end.x,end.y,end.z);
        const Vector3 b=end-dir*0.16;
        const Vector3 l=b+side*0.08;
        const Vector3 r=b-side*0.08;
        glVertex3d(end.x,end.y,end.z);glVertex3d(l.x,l.y,l.z);
        glVertex3d(end.x,end.y,end.z);glVertex3d(r.x,r.y,r.z);
        glEnd();glLineWidth(1.0f);
    }
}





Vector3 calculateAcceleration(
    const State3D& state
) {
    if (app.motionModel == MotionModel::Uniform) return {};
    if (app.motionModel == MotionModel::ConstantAcceleration)
        return app.initialAcceleration;
    if (app.path.empty()) return {};

    Progress progress =
        closestOnRoute(state.position);

    double lookAhead = std::clamp(
        0.55 + state.velocity.length() * 0.25,
        0.55,
        1.2
    );

    Vector3 target = positionOnRoute(
        std::min(
            app.totalLength,
            progress.distance + lookAhead
        )
    );

    Vector3 direction =
        normalized(target - state.position);

    double routeLeft = std::max(
        0.0,
        app.totalLength - progress.distance
    );

    double goalDistance =
        state.position.distanceTo(
            app.goal
        );

    double desiredSpeed = std::min(
        app.physics.maxSpeed,
        std::sqrt(
            2.0 *
            app.physics.braking *
            std::min(
                routeLeft,
                goalDistance + 0.1
            )
        )
    );

    Vector3 desiredVelocity =
        direction * desiredSpeed;

    Vector3 result =
        (desiredVelocity - state.velocity) *
        app.physics.response;

    double limit =
        dot(result, state.velocity) < 0.0
        ? app.physics.braking
        : app.physics.acceleration;

    if (result.length() > limit) {
        result = normalized(result) * limit;
    }

    if (app.motionModel == MotionModel::GuidedGravity ||
        app.motionModel == MotionModel::GuidedGravityDrag) {
        result.y -= app.gravity;
    }
    if (app.motionModel == MotionModel::GuidedDrag ||
        app.motionModel == MotionModel::GuidedGravityDrag) {
        result = result - state.velocity * app.linearDrag;
    }


    const double totalLimit=dot(result,state.velocity)<0.0
        ?app.physics.braking:app.physics.acceleration;
    if(result.length()>totalLimit) result=normalized(result)*totalLimit;
    const double speed=state.velocity.length();
    if(speed>1e-5) {
        const Vector3 axis=state.velocity/speed;
        const Vector3 parallel=axis*dot(result,axis);
        Vector3 lateral=result-parallel;
        const double maxLateral=speed*app.physics.maxTurnRate*PI/180.0;
        if(lateral.length()>maxLateral) lateral=normalized(lateral)*maxLateral;
        result=parallel+lateral;
    }
    return result;
}





void advanceOneStep(double dt) {advance(dt);}

void advance(double dt) {
    if (app.replaying) {
        if (app.paused || app.replaySamples.empty()) return;
        app.replayClock += dt;
        while(app.replayIndex+1<app.replaySamples.size() &&
              app.replaySamples[app.replayIndex+1].t <= app.replayClock) ++app.replayIndex;
        const auto& sm=app.replaySamples[app.replayIndex];
        const auto& next=app.replaySamples[
            std::min(app.replayIndex+1,app.replaySamples.size()-1)];
        const double u=next.t>sm.t?
            std::clamp((app.replayClock-sm.t)/(next.t-sm.t),0.0,1.0):0.0;
        app.moving.position=Vector3{sm.x,sm.y,sm.z}*(1.0-u)+
            Vector3{next.x,next.y,next.z}*u;
        app.state.position=app.moving.position;
        app.state.velocity=Vector3{sm.vx,sm.vy,sm.vz}*(1.0-u)+
            Vector3{next.vx,next.vy,next.vz}*u;
        app.state.acceleration={sm.ax,sm.ay,sm.az};
        app.state.time=app.replayClock;
        if(app.replayIndex+1>=app.replaySamples.size()) {
            app.paused=true;updatePauseButton();setStatus("Replay finished; press New run");
        }
        return;
    }


    if (app.finished) {
        if (app.finishPause > 0.0) {
            app.finishPause -= dt;

            if (app.finishPause <= 0.0) {
                newRun();
            }
        }

        return;
    }

    const bool freeFlight = app.motionModel == MotionModel::Uniform ||
                            app.motionModel == MotionModel::ConstantAcceleration;
    if (app.paused || app.collision || (app.path.empty() && !freeFlight)) {
        return;
    }

    double remaining = dt;

    while (remaining > 1e-8) {
        double step = remaining;

        Vector3 position =
            app.moving.position;

        Vector3 goal = app.goal;

        double goalDistance =
            position.distanceTo(goal);


        if (
            goalDistance < 0.045 &&
            app.state.velocity.length() < 0.25
        ) {
            if (shapeSegmentFree(
                app.moving,
                goal,
                app.obstacles,
                app.safetyMargin
            )) {
                completeExperiment();
            }
            else {
                app.collision = true;

                interruptExperiment(
                    "INTERRUPTED: collision near goal"
                );
            }

            return;
        }

        State3D current = app.state;
        current.position = position;

        current.acceleration =
            calculateAcceleration(current);


        State3D next =
            app.method == IntegrationMethod::Euler

            ? Physics3D::euler(
                current,
                step,
                calculateAcceleration
            )

            : Physics3D::rk4(
                current,
                step,
                calculateAcceleration
            );


        if (
            app.motionModel != MotionModel::Uniform &&
            app.motionModel != MotionModel::ConstantAcceleration &&
            next.velocity.length() >
            app.physics.maxSpeed
        ) {
            next.velocity =
                normalized(next.velocity) *
                app.physics.maxSpeed;
        }


        if(next.position.x < -0.01 || next.position.y < -0.01 ||
           next.position.z < -0.01 || next.position.x > NX-1+0.01 ||
           next.position.y > NY-1+0.01 || next.position.z > NZ-1+0.01) {
            interruptExperiment("INTERRUPTED: object left grid bounds");
            return;
        }
        if (!shapeSegmentFree(
            app.moving,
            next.position,
            app.obstacles,
            app.safetyMargin
        )) {
            app.collision = true;

            interruptExperiment(
                "INTERRUPTED: collision during movement"
            );

            return;
        }

        if (app.motionModel == MotionModel::Uniform ||
            app.motionModel == MotionModel::ConstantAcceleration) {
            const Vector3 movement = next.position - position;
            const double squared = dot(movement, movement);
            const double t = squared > 1e-15
                ? std::clamp(dot(goal - position, movement) / squared, 0.0, 1.0)
                : 0.0;
            if ((position + movement * t).distanceTo(goal) < 0.08) {
                app.state = next;
                completeExperiment();
                return;
            }
        }

        app.state = next;
        app.moving.position = next.position;


        app.telemetry.record(
            app.state,
            app.method
        );

        remaining -= step;
    }
}





bool isTyping() {
    HWND focus = GetFocus();

    for (HWND hwnd : app.obstacleEdits) {
        if (hwnd == focus) return true;
    }

    for (HWND hwnd : app.obstacleRotationEdits) {
        if (hwnd == focus) return true;
    }
    for (HWND hwnd : app.movingRotationEdits) {
        if (hwnd == focus) return true;
    }

    for (HWND hwnd : app.movingEdits) {
        if (hwnd == focus) return true;
    }

    for (HWND hwnd : app.routeStartEdits) if (hwnd == focus) return true;
    for (HWND hwnd : app.routeGoalEdits) if (hwnd == focus) return true;
    for (HWND hwnd : app.initialVelocityEdits) if (hwnd == focus) return true;
    for (HWND hwnd : app.initialAccelerationEdits) if (hwnd == focus) return true;
    return focus == app.speedEdit || focus == app.accelEdit ||
           focus == app.turnRateEdit || focus == app.brakeEdit || focus == app.timeWeightEdit ||
           focus == app.integrationStepEdit || focus == app.safetyMarginEdit ||
           focus == app.gravityEdit || focus == app.dragEdit;
}

bool keyDown(int key) {
    return (
        GetAsyncKeyState(key) & 0x8000
    ) != 0;
}





int main() {
    HINSTANCE instance =
        GetModuleHandleA(nullptr);

    WNDCLASSA wc{};

    wc.lpfnWndProc = sceneProc;
    wc.hInstance = instance;
    wc.style = CS_OWNDC;

    wc.lpszClassName =
        "3DMovesFinalSceneV1";

    wc.hCursor =
        LoadCursor(nullptr, IDC_ARROW);

    if (!RegisterClassA(&wc)) {
        return 1;
    }

    app.scene = CreateWindowExA(
        0,
        wc.lpszClassName,
        "3D_MOVES",
        WS_OVERLAPPEDWINDOW |
        WS_VISIBLE,
        50, 70,
        950, 700,
        nullptr, nullptr,
        instance, nullptr
    );

    if (!app.scene) return 1;

    app.dc = GetDC(app.scene);

    PIXELFORMATDESCRIPTOR pfd{};

    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;

    pfd.dwFlags =
        PFD_DRAW_TO_WINDOW |
        PFD_SUPPORT_OPENGL |
        PFD_DOUBLEBUFFER;

    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    int format =
        ChoosePixelFormat(app.dc, &pfd);

    if (
        !format ||
        !SetPixelFormat(
            app.dc, format, &pfd
        )
    ) {
        return 1;
    }

    app.gl = wglCreateContext(app.dc);

    if (
        !app.gl ||
        !wglMakeCurrent(
            app.dc, app.gl
        )
    ) {
        return 1;
    }

    glEnable(GL_DEPTH_TEST);

    glClearColor(
        0.07f,
        0.08f,
        0.12f,
        1.0f
    );


    createPanel(instance);


    app.analysis.create(
        instance,
        &app.telemetry
    );


    app.telemetry.setGoal(app.goal);
    app.telemetry.setEnvironment(&app.obstacles,&app.moving);

    resetScene();
    loadMovingFields();
    loadPhysicsFields();
    loadRouteFields();
    loadModelFields();


    newRun();

    auto previous =
        std::chrono::steady_clock::now();

    double graphRefresh = 0.0;

    MSG msg{};

    while (app.running) {
        while (PeekMessageA(
            &msg,
            nullptr,
            0, 0,
            PM_REMOVE
        )) {
            if (msg.message == WM_QUIT) {
                app.running = false;
                break;
            }

            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }

        if (!app.running) break;

        auto now =
            std::chrono::steady_clock::now();

        double dt = std::clamp(
            std::chrono::duration<double>(
                now - previous
            ).count(),
            0.0,
            0.05
        );

        previous = now;

        HWND foreground =
            GetForegroundWindow();

        bool active =
            foreground == app.scene ||
            foreground == app.panel;

        bool keys =
            active && !isTyping();

        if (keys) {
            if (keyDown(VK_LEFT)) {
                app.camera.yaw -= 80.0 * dt;
            }

            if (keyDown(VK_RIGHT)) {
                app.camera.yaw += 80.0 * dt;
            }

            if (keyDown(VK_UP)) {
                app.camera.pitch += 80.0 * dt;
            }

            if (keyDown(VK_DOWN)) {
                app.camera.pitch -= 80.0 * dt;
            }

            if (
                keyDown('1') ||
                keyDown(VK_NUMPAD1)
            ) {
                app.camera.distance -= 11.0 * dt;
            }

            if (
                keyDown('2') ||
                keyDown(VK_NUMPAD2)
            ) {
                app.camera.distance += 11.0 * dt;
            }

            if (keyDown('R')) {
                app.camera = Camera{};
            }
        }

        bool e = keys && keyDown('E');
        bool k = keys && keyDown('K');
        bool p = keys && keyDown('P');
        bool s = keys && keyDown('S');
        bool g = keys && keyDown('G');

        bool space =
            keys && keyDown(VK_SPACE);


        if (e && !app.previousE) {
            if (
                app.method !=
                IntegrationMethod::Euler
            ) {
                app.method =
                    IntegrationMethod::Euler;

                loadPhysicsFields();
                newRun();
            }
        }


        if (k && !app.previousK) {
            if (
                app.method !=
                IntegrationMethod::RK4
            ) {
                app.method =
                    IntegrationMethod::RK4;

                loadPhysicsFields();
                newRun();
            }
        }


        if (
            p &&
            !app.previousP &&
            app.panel
        ) {
            ShowWindow(
                app.panel,
                IsWindowVisible(app.panel)
                    ? SW_HIDE
                    : SW_SHOW
            );
        }


        if (s && !app.previousS) {
            app.smooth = !app.smooth;
            replan();
        }


        if (g && !app.previousG) {
            app.analysis.toggle();
        }


        if (space && !app.previousSpace) {
            app.paused = !app.paused;
            updatePauseButton();
        }

        app.previousE = e;
        app.previousK = k;
        app.previousP = p;
        app.previousS = s;
        app.previousG = g;
        app.previousSpace = space;

        app.camera.pitch = std::clamp(
            app.camera.pitch,
            -10.0, 85.0
        );

        app.camera.distance = std::clamp(
            app.camera.distance,
            6.0, 50.0
        );


        if (app.paused) {
            app.simulationAccumulator = 0.0;
        } else {
            app.simulationAccumulator += dt;
            int steps = 0;
            while (app.simulationAccumulator + 1e-12 >= app.integrationStep && steps < 200) {
                advance(app.integrationStep);
                app.simulationAccumulator -= app.integrationStep;
                ++steps;
                if (app.paused) {
                    app.simulationAccumulator = 0.0;
                    break;
                }
            }

            app.simulationAccumulator = std::max(0.0, std::min(app.simulationAccumulator, 0.2));
        }

        graphRefresh += dt;

        if (graphRefresh >= 0.12) {
            app.analysis.refresh();
            graphRefresh = 0.0;
        }

        RECT rect{};

        GetClientRect(
            app.scene,
            &rect
        );

        int width = int(std::max(
            1L,
            rect.right - rect.left
        ));

        int height = int(std::max(
            1L,
            rect.bottom - rect.top
        ));

        render(width, height);

        SwapBuffers(app.dc);

        Sleep(16);
    }

    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(app.gl);
    ReleaseDC(app.scene, app.dc);

    return 0;
}