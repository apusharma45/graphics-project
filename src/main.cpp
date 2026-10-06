#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

// World units: X is court width, Z is court length, and Y is up.
constexpr float COURT_WIDTH = 20.0f;
constexpr float COURT_LENGTH = 40.0f;
constexpr float COURT_MIN_X = -COURT_WIDTH / 2.0f;
constexpr float COURT_MAX_X =  COURT_WIDTH / 2.0f;
constexpr float COURT_MIN_Z = -COURT_LENGTH / 2.0f;
constexpr float COURT_MAX_Z =  COURT_LENGTH / 2.0f;

// Width and height describe the clear opening between/under the posts.
// Goal frames are centered on the end lines at Z = COURT_MIN/MAX_Z.
constexpr float GOAL_WIDTH = 3.0f;
constexpr float GOAL_HEIGHT = 2.0f;
constexpr float GOALPOST_THICKNESS = 0.15f;
constexpr float MARKING_WIDTH = 0.10f;
constexpr float MARKING_Y = 0.015f;
constexpr float CENTER_CIRCLE_RADIUS = 3.0f;
constexpr float CENTER_SPOT_RADIUS = 0.15f;
constexpr int CIRCLE_SEGMENTS = 96;

constexpr float ROBOT_WIDTH = 2.0f; // full arm-to-arm width
constexpr float ROBOT_HEIGHT = 3.0f;
constexpr float ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0f;
constexpr float ROBOT_HALF_DEPTH = 0.60f;
constexpr float ROBOT_BALL_COLLISION_RADIUS = 0.62f;
constexpr float ROBOT_MOVE_SPEED = 5.0f; // world units per second
const float ROBOT_TURN_SPEED = glm::radians(80.0f); // radians per second
constexpr float BALL_RADIUS = 0.28f;
constexpr float KICK_DISTANCE = ROBOT_BALL_COLLISION_RADIUS + BALL_RADIUS + 0.35f;
constexpr float KICK_SPEED = 12.0f; // world units per second
constexpr float BALL_FRICTION = 4.0f; // shot deceleration, units per second squared
bool ballIsShot = false;
bool spaceWasDown = false;
bool dribbleMode = false;
bool ballPossessed = false;
bool dribbleKeyWasDown = false;
glm::vec3 robotPosition(-2.0f, 0.0f, 1.0f); // root is at foot level
float robotYaw = glm::radians(25.0f); // local +Z is forward
glm::vec3 ballPosition(-0.8f, BALL_RADIUS, 3.0f);
glm::vec3 ballVelocity(0.0f);
glm::quat ballOrientation(1.0f, 0.0f, 0.0f, 0.0f);
int positiveGoalScore = 0;
int negativeGoalScore = 0;
float goalBannerTimeRemaining = 0.0f;
constexpr float POST_RESTITUTION = 0.5f;
constexpr float DRIBBLE_DISTANCE = ROBOT_BALL_COLLISION_RADIUS + BALL_RADIUS + 0.10f;

std::string scoreTitle()
{
    return "Futsal Simulation | Goal -Z: " + std::to_string(negativeGoalScore)
         + " | Goal +Z: " + std::to_string(positiveGoalScore);
}

void resetBallAfterGoal()
{
    // These midfield candidates are inside the court and far enough apart
    // that a single robot cannot overlap all of them.
    for (const glm::vec3& candidate : {glm::vec3(0, BALL_RADIUS, 0),
                                      glm::vec3(-3, BALL_RADIUS, 0),
                                      glm::vec3(3, BALL_RADIUS, 0)})
    {
        if (glm::length(glm::vec2(candidate.x - robotPosition.x,
                                   candidate.z - robotPosition.z)) >
            ROBOT_BALL_COLLISION_RADIUS + BALL_RADIUS + 0.1f)
        {
            ballPosition = candidate;
            break;
        }
    }
    ballVelocity = glm::vec3(0);
    ballOrientation = glm::quat(1, 0, 0, 0);
    ballIsShot = false;
    ballPossessed = false;
}

bool collideWithPost(float x, float z)
{
    const glm::vec2 halfSize(GOALPOST_THICKNESS / 2.0f);
    const glm::vec2 minimum = glm::vec2(x, z) - halfSize;
    const glm::vec2 maximum = glm::vec2(x, z) + halfSize;
    const glm::vec2 center(ballPosition.x, ballPosition.z);
    const glm::vec2 closest = glm::clamp(center, minimum, maximum);
    const glm::vec2 offset = center - closest;
    const float distance = glm::length(offset);
    if (distance >= BALL_RADIUS)
        return false;
    glm::vec2 normal;
    float penetration;
    if (distance > 0.000001f)
    {
        normal = offset / distance;
        penetration = BALL_RADIUS - distance;
    }
    else
    {
        // Center inside/on rectangle: push through the nearest face.
        const float gaps[] = {center.x - minimum.x, maximum.x - center.x,
                              center.y - minimum.y, maximum.y - center.y};
        const glm::vec2 normals[] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        int face = 0;
        for (int i = 1; i < 4; ++i)
            if (gaps[i] < gaps[face]) face = i;
        normal = normals[face];
        penetration = BALL_RADIUS + gaps[face];
    }
    const glm::vec3 normal3(normal.x, 0, normal.y);
    ballPosition += normal3 * (penetration + 0.00001f);
    const float inwardSpeed = glm::dot(ballVelocity, normal3);
    if (inwardSpeed < 0)
        ballVelocity -= (1.0f + POST_RESTITUTION) * inwardSpeed * normal3;
    return true;
}

bool detectGoal(const glm::vec3& previousPosition, bool hitPost)
{
    if (hitPost) return false;
    for (float sign : {-1.0f, 1.0f})
    {
        const float completeCrossing = COURT_LENGTH / 2.0f + BALL_RADIUS;
        const float before = sign * previousPosition.z;
        const float after = sign * ballPosition.z;
        constexpr float crossingTolerance = 0.0001f;
        if (before < completeCrossing
            && after + crossingTolerance >= completeCrossing)
        {
            const float fraction = glm::clamp(
                (completeCrossing - before) / (after - before), 0.0f, 1.0f);
            const float crossingX = glm::mix(previousPosition.x, ballPosition.x, fraction);
            if (std::abs(crossingX) + BALL_RADIUS <= GOAL_WIDTH / 2.0f &&
                std::abs(ballPosition.x) + BALL_RADIUS <= GOAL_WIDTH / 2.0f)
            {
                if (sign > 0) ++positiveGoalScore;
                else ++negativeGoalScore;
                goalBannerTimeRemaining = 2.0f;
                std::cout << scoreTitle() << '\n' << std::flush;
                resetBallAfterGoal();
                return true;
            }
        }
    }
    return false;
}

const glm::vec3 LIGHT_POSITIONS[] = {
    {-9.0f, 8.0f, -17.0f}, {9.0f, 8.0f, -17.0f},
    {-9.0f, 8.0f,  17.0f}, {9.0f, 8.0f,  17.0f}
};
// The spotlight starts at the front of the robot's head and points ahead with
// a slight downward tilt, so moving or turning the robot also moves the beam.
constexpr float HEAD_SPOTLIGHT_HEIGHT = 2.88f;
constexpr float HEAD_SPOTLIGHT_FORWARD_OFFSET = 0.36f;
constexpr float HEAD_SPOTLIGHT_DOWN_ANGLE = 18.0f;
constexpr float HEAD_SPOTLIGHT_INNER_ANGLE = 12.0f;
constexpr float HEAD_SPOTLIGHT_OUTER_ANGLE = 22.0f;
constexpr float BOARD_OFFSET = 2.0f;
constexpr float BOARD_HEIGHT = 0.45f;
constexpr float BOARD_THICKNESS = 0.20f;
constexpr float END_BOARD_GAP = 6.0f; // leaves both goal openings unobstructed
constexpr int VERTEX_FLOATS = 8; // position xyz + normal xyz + texture uv
int emissiveLocation = -1;
int diffuseStrengthLocation = -1;
int specularStrengthLocation = -1;
int shininessLocation = -1;
int textureEnabledLocation = -1;

constexpr float DEFAULT_CAMERA_AZIMUTH = 78.0f;
constexpr float DEFAULT_CAMERA_ELEVATION = 42.0f;
constexpr float DEFAULT_CAMERA_DISTANCE = 65.0f;
constexpr float CAMERA_MIN_DISTANCE = 22.0f;
constexpr float CAMERA_MAX_DISTANCE = 105.0f;
constexpr float CAMERA_MIN_ELEVATION = 25.0f;
constexpr float CAMERA_MAX_ELEVATION = 80.0f;
constexpr float CAMERA_FOV = 27.5f;
constexpr float CAMERA_REFERENCE_ASPECT = 16.0f / 9.0f;
const glm::vec3 CAMERA_TARGET(0.0f, 1.0f, 0.0f);
constexpr float PLAYER_CAMERA_HEIGHT = 2.65f;
constexpr float PLAYER_CAMERA_FORWARD_OFFSET = 0.55f;
constexpr float PLAYER_CAMERA_FOV = 65.0f;
constexpr float PLAYER_CAMERA_MIN_FOV = 35.0f;
constexpr float PLAYER_CAMERA_MAX_FOV = 80.0f;
constexpr float DEFAULT_PLAYER_LOOK_PITCH = -12.0f;
constexpr float PLAYER_LOOK_MIN_PITCH = -85.0f;
constexpr float PLAYER_LOOK_MAX_PITCH = 50.0f;
constexpr float PLAYER_LOOK_MAX_YAW = 85.0f;

void addVertex(std::vector<float>& vertices, const glm::vec3& position,
               const glm::vec3& normal)
{
    // World-space UVs make adjacent court-grid cells share a continuous pattern.
    const glm::vec2 textureCoordinate(position.x * 0.35f, position.z * 0.35f);
    vertices.insert(vertices.end(), {
        position.x, position.y, position.z, normal.x, normal.y, normal.z,
        textureCoordinate.x, textureCoordinate.y
    });
}

void updateRobot(float moveInput, float turnInput, float deltaTime)
{
    // Yaw stays in radians, matching drawRobot's GLM root rotation.
    // Positive Y rotation turns local +Z toward +X (the robot's left).
    robotYaw = std::remainder(robotYaw + turnInput * ROBOT_TURN_SPEED * deltaTime,
                              glm::radians(360.0f));
    const glm::vec3 forward(std::sin(robotYaw), 0.0f, std::cos(robotYaw));
    robotPosition += forward * moveInput * ROBOT_MOVE_SPEED * deltaTime;

    // Rotate the robot's simple XZ footprint into world-aligned extents. This
    // keeps the body inside the court without an oversized circular margin.
    const float absoluteCosine = std::abs(std::cos(robotYaw));
    const float absoluteSine = std::abs(std::sin(robotYaw));
    const float extentX = ROBOT_HALF_WIDTH * absoluteCosine
                        + ROBOT_HALF_DEPTH * absoluteSine;
    const float extentZ = ROBOT_HALF_WIDTH * absoluteSine
                        + ROBOT_HALF_DEPTH * absoluteCosine;
    robotPosition.x = glm::clamp(robotPosition.x,
        COURT_MIN_X + extentX, COURT_MAX_X - extentX);
    robotPosition.z = glm::clamp(robotPosition.z,
        COURT_MIN_Z + extentZ, COURT_MAX_Z - extentZ);
    robotPosition.y = 0.0f;
}

bool tryKickBall()
{
    const glm::vec2 offset(ballPosition.x - robotPosition.x,
                           ballPosition.z - robotPosition.z);
    if (glm::length(offset) > KICK_DISTANCE)
        return false;
    // Same local +Z/radian convention as the rendered robot and W movement.
    ballVelocity = glm::vec3(std::sin(robotYaw), 0, std::cos(robotYaw)) * KICK_SPEED;
    ballIsShot = true;
    ballPossessed = false;
    return true;
}

void handleKickInput(bool spaceDown)
{
    if (spaceDown && !spaceWasDown)
        tryKickBall();
    spaceWasDown = spaceDown; // release required before another kick
}

void handleDribbleInput(bool keyDown)
{
    if (keyDown && !dribbleKeyWasDown)
    {
        dribbleMode = !dribbleMode;
        if (!dribbleMode)
            ballPossessed = false;
    }
    dribbleKeyWasDown = keyDown;
}

void updateDribbledBall(float deltaTime)
{
    const glm::vec3 previousPosition = ballPosition;
    const glm::vec3 forward(std::sin(robotYaw), 0.0f, std::cos(robotYaw));
    ballPosition = robotPosition + forward * DRIBBLE_DISTANCE;
    ballPosition.y = BALL_RADIUS;
    ballVelocity = deltaTime > 0.0f
        ? (ballPosition - previousPosition) / deltaTime : glm::vec3(0.0f);

    bool hitPost = false;
    const float postX = (GOAL_WIDTH + GOALPOST_THICKNESS) / 2.0f;
    for (float x : {-postX, postX})
        for (float z : {COURT_MIN_Z, COURT_MAX_Z})
            hitPost = collideWithPost(x, z) || hitPost;

    ballPosition.x = glm::clamp(
        ballPosition.x, COURT_MIN_X + BALL_RADIUS, COURT_MAX_X - BALL_RADIUS);
    if (std::abs(ballPosition.x) + BALL_RADIUS > GOAL_WIDTH / 2.0f)
        ballPosition.z = glm::clamp(
            ballPosition.z, COURT_MIN_Z + BALL_RADIUS, COURT_MAX_Z - BALL_RADIUS);

    if (detectGoal(previousPosition, hitPost))
        return;

    const glm::vec3 travel = ballPosition - previousPosition;
    const float travelDistance = glm::length(travel);
    if (travelDistance > 0.000001f)
    {
        const glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 1, 0), travel));
        ballOrientation = glm::normalize(
            glm::angleAxis(travelDistance / BALL_RADIUS, axis) * ballOrientation);
    }
}

void updateBallStep(float deltaTime)
{
    ballPosition.y = BALL_RADIUS;
    if (deltaTime <= 0.0f)
        return;
    const glm::vec3 previousPosition = ballPosition;
    if (ballIsShot)
    {
        // Integrate constant friction up to the stopping time (never reverse).
        const float speed = glm::length(ballVelocity);
        if (speed > 0.0f)
        {
            const float movingTime = glm::min(deltaTime, speed / BALL_FRICTION);
            const glm::vec3 direction = ballVelocity / speed;
            ballPosition += direction * (speed * movingTime
                - 0.5f * BALL_FRICTION * movingTime * movingTime);
            ballVelocity = direction * glm::max(0.0f, speed - BALL_FRICTION * deltaTime);
        }
    }
    else
        ballVelocity = glm::vec3(0.0f); // preserve contact-only dribbling

    const glm::vec2 offset(ballPosition.x - robotPosition.x,
                           ballPosition.z - robotPosition.z);
    const float distance = glm::length(offset);
    const float contactDistance = ROBOT_BALL_COLLISION_RADIUS + BALL_RADIUS;
    if (distance < contactDistance)
    {
    // Resolve penetration along the contact normal, not a fixed world axis.
    // Coincident centers need a fallback to avoid dividing by zero.
    const glm::vec2 normal = distance > 0.00001f
        ? offset / distance : glm::vec2(std::sin(robotYaw), std::cos(robotYaw));
    const glm::vec3 displacement(normal.x * (contactDistance - distance), 0.0f,
                                  normal.y * (contactDistance - distance));
        ballPosition += displacement;
        if (!ballIsShot)
            ballVelocity = displacement / deltaTime;
        else
        {
            // Stop the inward component if a shot meets the robot.
            const glm::vec3 contactNormal(normal.x, 0, normal.y);
            const float inwardSpeed = glm::dot(ballVelocity, contactNormal);
            if (inwardSpeed < 0)
                ballVelocity -= inwardSpeed * contactNormal;
        }
    }

    bool hitPost = false;
    const float postX = (GOAL_WIDTH + GOALPOST_THICKNESS) / 2.0f;
    for (float x : {-postX, postX})
        for (float z : {COURT_MIN_Z, COURT_MAX_Z})
            hitPost = collideWithPost(x, z) || hitPost;

    // Sidelines stay solid. End lines open only where the full ball fits.
    for (int axis : {0, 2})
    {
        if (axis == 2 && std::abs(ballPosition.x) + BALL_RADIUS <= GOAL_WIDTH / 2.0f)
            continue;
        const float minPosition = (axis == 0 ? COURT_MIN_X : COURT_MIN_Z) + BALL_RADIUS;
        const float maxPosition = (axis == 0 ? COURT_MAX_X : COURT_MAX_Z) - BALL_RADIUS;
        const float bounded = glm::clamp(ballPosition[axis], minPosition, maxPosition);
        if (bounded != ballPosition[axis])
        {
            ballPosition[axis] = bounded;
            ballVelocity[axis] = 0.0f;
        }
    }
    if (ballIsShot && glm::length(ballVelocity) == 0.0f)
        ballIsShot = false;

    if (detectGoal(previousPosition, hitPost))
        return; // reset orientation must not include the teleport to midfield

    // Rolling without slipping: angle = distance / radius.
    // Apply the world-space rotation before the accumulated orientation.
    const glm::vec3 travel = ballPosition - previousPosition;
    const float travelDistance = glm::length(travel);
    if (travelDistance > 0.000001f)
    {
        const glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 1, 0), travel));
        ballOrientation = glm::normalize(
            glm::angleAxis(travelDistance / BALL_RADIUS, axis) * ballOrientation);
    }
}

void updateBall(float deltaTime)
{
    if (deltaTime <= 0) return;
    if (dribbleMode && !ballIsShot)
    {
        const float distanceToRobot = glm::length(glm::vec2(
            ballPosition.x - robotPosition.x, ballPosition.z - robotPosition.z));
        if (!ballPossessed && distanceToRobot <= KICK_DISTANCE)
            ballPossessed = true;
        if (ballPossessed)
        {
            updateDribbledBall(deltaTime);
            return;
        }
    }
    if (!ballIsShot)
    {
        // Robot movement is already capped at 0.25 units per frame, less than
        // the ball diameter. Preserve the dribbling velocity for that frame.
        updateBallStep(deltaTime);
        return;
    }
    // Limit each shot step to a quarter of a post's thickness, even at high
    // speeds. Smaller time steps also keep friction and contact easy to follow.
    const float speed = ballIsShot ? glm::length(ballVelocity) : ROBOT_MOVE_SPEED;
    const float maxStep = glm::min(1.0f / 120.0f,
        GOALPOST_THICKNESS * 0.25f / glm::max(speed, ROBOT_MOVE_SPEED));
    const int steps = static_cast<int>(std::ceil(deltaTime / maxStep));
    const int scoreBefore = positiveGoalScore + negativeGoalScore;
    for (int i = 0; i < steps; ++i)
    {
        updateBallStep(deltaTime / steps);
        if (positiveGoalScore + negativeGoalScore != scoreBefore)
            break; // one goal, then leave the reset ball at rest
    }
}

// All meshes share an interleaved position/normal VBO.
void addRectangle(std::vector<float>& vertices, float minX, float maxX,
                  float minZ, float maxZ, float y)
{
    const glm::vec3 normal(0, 1, 0);
    for (const glm::vec3& position : {
        glm::vec3(minX, y, minZ), glm::vec3(minX, y, maxZ), glm::vec3(maxX, y, maxZ),
        glm::vec3(minX, y, minZ), glm::vec3(maxX, y, maxZ), glm::vec3(maxX, y, minZ)})
        addVertex(vertices, position, normal);
}

void addCourtGrid(std::vector<float>& vertices, int columns, int rows)
{
    const float cellWidth = COURT_WIDTH / columns;
    const float cellLength = COURT_LENGTH / rows;
    for (int z = 0; z < rows; ++z)
        for (int x = 0; x < columns; ++x)
            addRectangle(vertices,
                COURT_MIN_X + x * cellWidth,
                COURT_MIN_X + (x + 1) * cellWidth,
                COURT_MIN_Z + z * cellLength,
                COURT_MIN_Z + (z + 1) * cellLength, 0.0f);
}

void addUnitCube(std::vector<float>& vertices)
{
    const glm::vec3 corners[] = {
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
        {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},
        {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };
    const int faces[][6] = {
        {0, 2, 1, 0, 3, 2}, {4, 5, 6, 4, 6, 7},
        {0, 4, 7, 0, 7, 3}, {1, 2, 6, 1, 6, 5},
        {0, 1, 5, 0, 5, 4}, {3, 7, 6, 3, 6, 2}
    };
    const glm::vec3 normals[] = {
        {0, 0, -1}, {0, 0, 1}, {-1, 0, 0},
        {1, 0, 0}, {0, -1, 0}, {0, 1, 0}
    };
    for (int face = 0; face < 6; ++face)
        for (int index : faces[face])
            addVertex(vertices, corners[index], normals[face]);
}

void drawMesh(int first, int count, int modelLocation, int colorLocation,
              const glm::mat4& model, const glm::vec3& color,
              bool emissive = false, float diffuseStrength = 0.75f,
              float specularStrength = 0.22f, float shininess = 24.0f,
              bool textured = false)
{
    glUniformMatrix4fv(modelLocation, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(colorLocation, 1, glm::value_ptr(color));
    glUniform1i(emissiveLocation, emissive ? 1 : 0);
    glUniform1f(diffuseStrengthLocation, diffuseStrength);
    glUniform1f(specularStrengthLocation, specularStrength);
    glUniform1f(shininessLocation, shininess);
    glUniform1i(textureEnabledLocation, textured ? 1 : 0);
    glDrawArrays(GL_TRIANGLES, first, count);
}

void drawCube(int cubeFirst, int modelLocation, int colorLocation,
              const glm::vec3& position, const glm::vec3& size,
              const glm::vec3& color = glm::vec3(0.95f),
              const glm::mat4& parent = glm::mat4(1.0f))
{
    glm::mat4 model = glm::translate(parent, position);
    model = glm::scale(model, size);
    drawMesh(cubeFirst, 36, modelLocation, colorLocation, model, color);
}

void drawRobot(int cubeFirst, int modelLocation, int colorLocation,
               bool firstPersonView = false, bool headSpotlightEnabled = true)
{
    glm::mat4 root = glm::translate(glm::mat4(1.0f), robotPosition);
    root = glm::rotate(root, robotYaw, glm::vec3(0.0f, 1.0f, 0.0f));
    // Dimensions below are fractions of the robot's overall width/height.
    // Each child inherits root translation/yaw before its local translation/scale.
    auto part = [&](glm::vec3 position, glm::vec3 size, glm::vec3 color)
    {
        const glm::vec3 dimensions(ROBOT_WIDTH, ROBOT_HEIGHT, 1.0f);
        drawCube(cubeFirst, modelLocation, colorLocation,
                 position * dimensions, size * dimensions, color, root);
    };
    const glm::vec3 blue(0.10f, 0.35f, 0.85f), gray(0.60f, 0.67f, 0.75f);
    const glm::vec3 dark(0.08f, 0.12f, 0.20f), front(0.15f, 0.85f, 1.0f);
    // A head-mounted camera can see the body below it, but not the geometry
    // containing the camera itself.
    part({0, 0.57f, 0}, {0.60f, 0.34f, 0.65f}, blue); // torso
    if (!firstPersonView)
    {
        part({0, 0.77f, 0}, {0.18f, 0.06f, 0.30f}, dark); // neck
        part({0, 0.90f, 0}, {0.46f, 0.20f, 0.60f}, gray); // head

        glm::mat4 headlamp = glm::translate(
            root, glm::vec3(0.0f, HEAD_SPOTLIGHT_HEIGHT,
                            HEAD_SPOTLIGHT_FORWARD_OFFSET));
        headlamp = glm::scale(headlamp, glm::vec3(0.18f, 0.10f, 0.05f));
        drawMesh(cubeFirst, 36, modelLocation, colorLocation, headlamp,
                 headSpotlightEnabled ? glm::vec3(1.0f, 0.90f, 0.45f) : dark,
                 headSpotlightEnabled);
    }
    for (float side : {-1.0f, 1.0f})
    {
        part({side * 0.41f, 0.56f, 0}, {0.18f, 0.32f, 0.48f}, gray); // arms
        part({side * 0.17f, 0.24f, 0}, {0.22f, 0.32f, 0.45f}, gray); // legs
        part({side * 0.17f, 0.04f, 0.15f}, {0.28f, 0.08f, 0.85f}, dark); // feet: bottom = 0
        if (!firstPersonView)
            part({side * 0.12f, 0.92f, 0.315f}, {0.08f, 0.045f, 0.04f}, dark); // eyes
    }
    part({0, 0.59f, 0.34f}, {0.38f, 0.19f, 0.04f}, front); // chest panel
}

// A unit latitude/longitude sphere, split into two color groups.
// Contrasting patches reveal curvature without lighting or textures.
void addSphere(std::vector<float>& vertices, int& whiteFirst, int& whiteCount,
               int& darkFirst, int& darkCount)
{
    constexpr int stacks = 16, slices = 32;
    std::vector<float> darkVertices;
    whiteFirst = static_cast<int>(vertices.size() / VERTEX_FLOATS);
    auto point = [](int latitude, int longitude)
    {
        const float phi = glm::radians(180.0f) * latitude / stacks;
        const float theta = glm::radians(360.0f) * longitude / slices;
        return glm::vec3(std::sin(phi) * std::cos(theta), std::cos(phi),
                         std::sin(phi) * std::sin(theta));
    };
    for (int row = 0; row < stacks; ++row)
        for (int col = 0; col < slices; ++col)
        {
            auto& target = ((row / 4 + col / 4) % 2 == 0) ? vertices : darkVertices;
            const glm::vec3 a = point(row, col), b = point(row + 1, col);
            const glm::vec3 c = point(row + 1, col + 1), d = point(row, col + 1);
            // Skip degenerate triangles at the poles.
            if (row != stacks - 1)
                for (const auto& p : {a, b, c})
                    addVertex(target, p, glm::normalize(p));
            if (row != 0)
                for (const auto& p : {a, c, d})
                    addVertex(target, p, glm::normalize(p));
        }
    whiteCount = static_cast<int>(vertices.size() / VERTEX_FLOATS) - whiteFirst;
    darkFirst = static_cast<int>(vertices.size() / VERTEX_FLOATS);
    darkCount = static_cast<int>(darkVertices.size() / VERTEX_FLOATS);
    vertices.insert(vertices.end(), darkVertices.begin(), darkVertices.end());
}

void drawSurroundings(int cubeFirst, int modelLocation, int colorLocation)
{
    const glm::vec3 boardColor(0.25f, 0.28f, 0.32f);
    const float outerX = COURT_MAX_X + BOARD_OFFSET;
    const float outerZ = COURT_MAX_Z + BOARD_OFFSET;
    for (float side : {-1.0f, 1.0f})
    {
        drawCube(cubeFirst, modelLocation, colorLocation,
                 {side * outerX, BOARD_HEIGHT / 2, 0},
                 {BOARD_THICKNESS, BOARD_HEIGHT, outerZ * 2}, boardColor);
        const float segmentWidth = outerX - END_BOARD_GAP / 2;
        for (float end : {-1.0f, 1.0f})
            drawCube(cubeFirst, modelLocation, colorLocation,
                     {side * (END_BOARD_GAP / 2 + segmentWidth / 2), BOARD_HEIGHT / 2, end * outerZ},
                     {segmentWidth, BOARD_HEIGHT, BOARD_THICKNESS}, boardColor);
    }

    for (const glm::vec3& light : LIGHT_POSITIONS)
    {
        const float poleX = light.x < 0 ? COURT_MIN_X - 1.0f : COURT_MAX_X + 1.0f;
        drawCube(cubeFirst, modelLocation, colorLocation,
                 {poleX, light.y / 2, light.z}, {0.18f, light.y, 0.18f}, boardColor);
        drawCube(cubeFirst, modelLocation, colorLocation,
                 {(poleX + light.x) / 2, light.y, light.z},
                 {std::abs(poleX - light.x), 0.16f, 0.16f}, boardColor);
        drawCube(cubeFirst, modelLocation, colorLocation,
                 light, {1.2f, 0.30f, 0.8f}, glm::vec3(0.65f, 0.69f, 0.74f));
        // Bright panels bypass ordinary lighting to look emissive.
        for (float side : {-1.0f, 1.0f})
        {
            glm::mat4 panel = glm::translate(glm::mat4(1),
                light + glm::vec3(0, side * 0.16f, 0));
            panel = glm::scale(panel, glm::vec3(1.0f, 0.04f, 0.65f));
            drawMesh(cubeFirst, 36, modelLocation, colorLocation, panel,
                     glm::vec3(1.0f, 0.95f, 0.65f), true);
        }
    }
}

void drawGoal(float z, int cubeFirst, int modelLocation, int colorLocation)
{
    const float t = GOALPOST_THICKNESS;
    const float postX = (GOAL_WIDTH + t) / 2.0f;
    for (float x : {-postX, postX})
        drawCube(cubeFirst, modelLocation, colorLocation,
                 glm::vec3(x, GOAL_HEIGHT / 2.0f, z),
                 glm::vec3(t, GOAL_HEIGHT, t));

    drawCube(cubeFirst, modelLocation, colorLocation,
             glm::vec3(0.0f, GOAL_HEIGHT + t / 2.0f, z),
             glm::vec3(GOAL_WIDTH + 2.0f * t, t, t));
}

const char* glyphPattern(char character)
{
    switch (character)
    {
        case '0': return "01110100011000110001100011000101110";
        case '1': return "00100011000010000100001000010001110";
        case '2': return "01110100010000100010001000100011111";
        case '3': return "11110000010000101110000010000111110";
        case '4': return "00010001100101010010111110001000010";
        case '5': return "11111100001111000001000011000101110";
        case '6': return "00110010001000011110100011000101110";
        case '7': return "11111000010001000100010000100001000";
        case '8': return "01110100011000101110100011000101110";
        case '9': return "01110100011000101111000010001001100";
        case 'A': return "01110100011000111111100011000110001";
        case 'B': return "11110100011000111110100011000111110";
        case 'C': return "01111100001000010000100001000001111";
        case 'D': return "11110100011000110001100011000111110";
        case 'E': return "11111100001000011110100001000011111";
        case 'G': return "01111100001000010111100011000101111";
        case 'I': return "11111001000010000100001000010011111";
        case 'L': return "10000100001000010000100001000011111";
        case 'O': return "01110100011000110001100011000101110";
        case 'R': return "11110100011000111110101001001010001";
        case 'S': return "01111100001000001110000010000111110";
        case '!': return "00100001000010000100001000000000100";
        case '-': return "00000000000000011111000000000000000";
        case ' ': return "00000000000000000000000000000000000";
        default:  return "11111100010001000100000000010000100";
    }
}

float bitmapTextWidth(const std::string& text, float pixelHeight, float aspect)
{
    const float pixelWidth = pixelHeight / aspect;
    return text.empty() ? 0.0f
        : (static_cast<float>(text.size()) * 6.0f - 1.0f) * pixelWidth;
}

void drawOverlayRectangle(int cubeFirst, int modelLocation, int colorLocation,
                          float centerX, float centerY, float width, float height,
                          const glm::vec3& color)
{
    glm::mat4 model = glm::translate(
        glm::mat4(1.0f), glm::vec3(centerX, centerY, 0.0f));
    model = glm::scale(model, glm::vec3(width, height, 0.01f));
    drawMesh(cubeFirst, 36, modelLocation, colorLocation, model, color, true);
}

void drawBitmapText(const std::string& text, int cubeFirst,
                    int modelLocation, int colorLocation, float x, float y,
                    float pixelHeight, float aspect, const glm::vec3& color)
{
    const float pixelWidth = pixelHeight / aspect;
    const float advance = pixelWidth * 6.0f;
    for (std::size_t character = 0; character < text.size(); ++character)
    {
        const char* pattern = glyphPattern(text[character]);
        for (int row = 0; row < 7; ++row)
            for (int column = 0; column < 5; ++column)
                if (pattern[row * 5 + column] == '1')
                    drawOverlayRectangle(
                        cubeFirst, modelLocation, colorLocation,
                        x + character * advance + (column + 0.5f) * pixelWidth,
                        y - (row + 0.5f) * pixelHeight,
                        pixelWidth * 0.86f, pixelHeight * 0.86f, color);
    }
}

void drawHud(int cubeFirst, int modelLocation, int colorLocation,
             int viewLocation, int projectionLocation, float aspect)
{
    glDisable(GL_DEPTH_TEST);
    const glm::mat4 identity(1.0f);
    glUniformMatrix4fv(viewLocation, 1, GL_FALSE, glm::value_ptr(identity));
    glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, glm::value_ptr(identity));

    const std::string score = "SCORE " + std::to_string(negativeGoalScore)
                            + " - " + std::to_string(positiveGoalScore);
    constexpr float scorePixelHeight = 0.0125f;
    const float scoreWidth = bitmapTextWidth(score, scorePixelHeight, aspect);
    const float panelLeft = -0.975f;
    const float panelTop = 0.965f;
    const float panelWidth = scoreWidth + 0.055f;
    constexpr float panelHeight = 0.120f;
    drawOverlayRectangle(cubeFirst, modelLocation, colorLocation,
        panelLeft + panelWidth / 2, panelTop - panelHeight / 2,
        panelWidth + 0.012f, panelHeight + 0.012f, glm::vec3(0.90f, 0.68f, 0.14f));
    drawOverlayRectangle(cubeFirst, modelLocation, colorLocation,
        panelLeft + panelWidth / 2, panelTop - panelHeight / 2,
        panelWidth, panelHeight, glm::vec3(0.035f, 0.055f, 0.09f));
    drawBitmapText(score, cubeFirst, modelLocation, colorLocation,
        panelLeft + 0.026f, panelTop - 0.020f, scorePixelHeight, aspect,
        glm::vec3(0.96f, 0.97f, 1.0f));

    if (dribbleMode)
    {
        const std::string status = "DRIBBLE";
        constexpr float statusPixelHeight = 0.011f;
        const float statusWidth = bitmapTextWidth(status, statusPixelHeight, aspect);
        drawOverlayRectangle(cubeFirst, modelLocation, colorLocation,
            0.965f - statusWidth / 2, 0.925f,
            statusWidth + 0.050f, 0.105f, glm::vec3(0.04f, 0.20f, 0.10f));
        drawBitmapText(status, cubeFirst, modelLocation, colorLocation,
            0.94f - statusWidth, 0.957f, statusPixelHeight, aspect,
            glm::vec3(0.45f, 1.0f, 0.58f));
    }

    if (goalBannerTimeRemaining > 0.0f)
    {
        constexpr float bannerPixelHeight = 0.040f;
        const std::string banner = "GOAL!";
        const float bannerWidth = bitmapTextWidth(banner, bannerPixelHeight, aspect);
        drawOverlayRectangle(cubeFirst, modelLocation, colorLocation,
            0.0f, 0.12f, bannerWidth + 0.18f, 0.39f,
            glm::vec3(0.04f, 0.06f, 0.12f));
        drawOverlayRectangle(cubeFirst, modelLocation, colorLocation,
            0.0f, 0.12f, bannerWidth + 0.15f, 0.36f,
            glm::vec3(0.50f, 0.04f, 0.06f));
        const float textX = -bannerWidth / 2.0f;
        drawBitmapText(banner, cubeFirst, modelLocation, colorLocation,
            textX + 0.012f, 0.275f - 0.012f, bannerPixelHeight, aspect,
            glm::vec3(0.12f, 0.02f, 0.02f));
        drawBitmapText(banner, cubeFirst, modelLocation, colorLocation,
            textX, 0.275f, bannerPixelHeight, aspect,
            glm::vec3(1.0f, 0.82f, 0.20f));
    }

    glEnable(GL_DEPTH_TEST);
}

int main()
{
    // 1. Initialize GLFW
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW\n";
        return -1;
    }

    // OpenGL 3.3 Core
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 2. Create window
    GLFWwindow* window = glfwCreateWindow(
        1000,
        750,
        scoreTitle().c_str(),
        nullptr,
        nullptr
    );

    if (!window)
    {
        std::cout << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwMaximizeWindow(window);
    glfwSwapInterval(1);

    // 3. Load OpenGL functions using GLAD
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    std::cout << "OpenGL: " << glGetString(GL_VERSION)
              << "\nRenderer: " << glGetString(GL_RENDERER)
              << "\nI/K: move | J/L: turn | D: dribble mode | Space: shoot"
              << " | P: player view | Arrows: turn/elevate | Z/X: zoom | R: reset view"
              << "\n1: ambient | 2: ambient + diffuse | 3: full Gouraud lighting"
              << " | H: head spotlight | Escape: exit\n"
              << std::flush;

    // Field: 20 units along X, 40 along Z, with Y pointing up.
    // A small grid provides enough vertices for Gouraud light interpolation.
    std::vector<float> vertices;
    addCourtGrid(vertices, 10, 20);
    const int courtVertexCount = static_cast<int>(vertices.size() / VERTEX_FLOATS);

    const int markingFirst = static_cast<int>(vertices.size() / VERTEX_FLOATS);
    // Boundary strips lie inside the playable limits; no solid walls.
    addRectangle(vertices, COURT_MIN_X, COURT_MIN_X + MARKING_WIDTH,
                 COURT_MIN_Z, COURT_MAX_Z, MARKING_Y);
    addRectangle(vertices, COURT_MAX_X - MARKING_WIDTH, COURT_MAX_X,
                 COURT_MIN_Z, COURT_MAX_Z, MARKING_Y);
    addRectangle(vertices, COURT_MIN_X + MARKING_WIDTH, COURT_MAX_X - MARKING_WIDTH,
                 COURT_MIN_Z, COURT_MIN_Z + MARKING_WIDTH, MARKING_Y);
    addRectangle(vertices, COURT_MIN_X + MARKING_WIDTH, COURT_MAX_X - MARKING_WIDTH,
                 COURT_MAX_Z - MARKING_WIDTH, COURT_MAX_Z, MARKING_Y);
    addRectangle(vertices, COURT_MIN_X + MARKING_WIDTH, COURT_MAX_X - MARKING_WIDTH,
                 -MARKING_WIDTH / 2.0f, MARKING_WIDTH / 2.0f, MARKING_Y);

    // Triangle ring and disk give markings a world-space width (no textures).
    const float inner = CENTER_CIRCLE_RADIUS - MARKING_WIDTH / 2.0f;
    const float outer = CENTER_CIRCLE_RADIUS + MARKING_WIDTH / 2.0f;
    const float circleY = MARKING_Y + 0.002f;
    for (int i = 0; i < CIRCLE_SEGMENTS; ++i)
    {
        const float a = glm::radians(360.0f) * i / CIRCLE_SEGMENTS;
        const float b = glm::radians(360.0f) * (i + 1) / CIRCLE_SEGMENTS;
        const float ca = std::cos(a), sa = std::sin(a);
        const float cb = std::cos(b), sb = std::sin(b);
        const glm::vec3 up(0, 1, 0);
        for (const glm::vec3& position : {
            glm::vec3(inner * ca, circleY, inner * sa),
            glm::vec3(outer * ca, circleY, outer * sa),
            glm::vec3(outer * cb, circleY, outer * sb),
            glm::vec3(inner * ca, circleY, inner * sa),
            glm::vec3(outer * cb, circleY, outer * sb),
            glm::vec3(inner * cb, circleY, inner * sb),
            glm::vec3(0, circleY, 0),
            glm::vec3(CENTER_SPOT_RADIUS * ca, circleY, CENTER_SPOT_RADIUS * sa),
            glm::vec3(CENTER_SPOT_RADIUS * cb, circleY, CENTER_SPOT_RADIUS * sb)})
            addVertex(vertices, position, up);
    }
    const int markingCount = static_cast<int>(vertices.size() / VERTEX_FLOATS) - markingFirst;

    // Unit cube centered at the origin: eight corners, two triangles per face.
    const int cubeFirst = static_cast<int>(vertices.size() / VERTEX_FLOATS);
    addUnitCube(vertices);

    int ballWhiteFirst, ballWhiteCount, ballDarkFirst, ballDarkCount;
    addSphere(vertices, ballWhiteFirst, ballWhiteCount, ballDarkFirst, ballDarkCount);


    // -----------------------------
    // Vertex Shader
    // -----------------------------
    const char* vertexShaderSource = R"(
        #version 330 core

        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec2 aTexCoord;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        uniform vec3 objectColor;
        uniform vec3 lightPositions[4];
        uniform vec3 headSpotlightPosition;
        uniform vec3 headSpotlightDirection;
        uniform float headSpotlightInnerCutoff;
        uniform float headSpotlightOuterCutoff;
        uniform bool headSpotlightEnabled;
        uniform vec3 viewPosition;
        uniform int lightingMode;
        uniform bool emissive;
        uniform float diffuseStrength;
        uniform float specularStrength;
        uniform float shininess;

        out vec3 vertexColor;
        out vec2 textureCoordinate;

        void main()
        {
            vec4 worldPosition = model * vec4(aPos, 1.0);
            vec3 N = normalize(mat3(transpose(inverse(model))) * aNormal);

            if (emissive)
            {
                vertexColor = objectColor;
            }
            else
            {
                // Gouraud shading: the complete illumination model is
                // evaluated here per vertex, then interpolated by OpenGL.
                vec3 ambient = 0.24 * objectColor;
                vec3 diffuse = vec3(0.0);
                vec3 specular = vec3(0.0);
                vec3 V = normalize(viewPosition - worldPosition.xyz);

                for (int i = 0; i < 4; ++i)
                {
                    vec3 toLight = lightPositions[i] - worldPosition.xyz;
                    float distanceToLight = length(toLight);
                    vec3 L = toLight / max(distanceToLight, 0.0001);
                    float attenuation = 1.0 /
                        (1.0 + 0.025 * distanceToLight
                             + 0.003 * distanceToLight * distanceToLight);
                    float diffuseAmount = max(dot(N, L), 0.0);
                    diffuse += diffuseStrength * attenuation
                             * diffuseAmount * objectColor;

                    if (diffuseAmount > 0.0)
                    {
                        vec3 R = reflect(-L, N);
                        float highlight = pow(max(dot(R, V), 0.0), shininess);
                        specular += vec3(specularStrength * attenuation * highlight);
                    }
                }

                if (headSpotlightEnabled)
                {
                    vec3 spotToVertex = worldPosition.xyz - headSpotlightPosition;
                    float spotDistance = length(spotToVertex);
                    vec3 fromSpot = spotToVertex / max(spotDistance, 0.0001);
                    float coneAngle = dot(
                        fromSpot, normalize(headSpotlightDirection));
                    float coneIntensity = smoothstep(
                        headSpotlightOuterCutoff,
                        headSpotlightInnerCutoff,
                        coneAngle);

                    if (coneIntensity > 0.0)
                    {
                        vec3 L = -fromSpot;
                        float attenuation = 1.0 /
                            (1.0 + 0.05 * spotDistance
                                 + 0.012 * spotDistance * spotDistance);
                        float diffuseAmount = max(dot(N, L), 0.0);
                        vec3 spotColor = vec3(1.0, 0.92, 0.68);
                        float spotStrength = 2.2 * coneIntensity * attenuation;
                        diffuse += diffuseStrength * spotStrength
                                 * diffuseAmount * objectColor * spotColor;

                        if (diffuseAmount > 0.0)
                        {
                            vec3 R = reflect(-L, N);
                            float highlight = pow(
                                max(dot(R, V), 0.0), shininess);
                            specular += spotColor * specularStrength
                                      * spotStrength * highlight;
                        }
                    }
                }

                vertexColor = ambient;
                if (lightingMode >= 2) vertexColor += diffuse;
                if (lightingMode >= 3) vertexColor += specular;
                vertexColor = clamp(vertexColor, 0.0, 1.0);
            }

            gl_Position = projection * view * worldPosition;
            textureCoordinate = aTexCoord;
        }
    )";

    // -----------------------------
    // Fragment Shader
    // -----------------------------
    const char* fragmentShaderSource = R"(
        #version 330 core

        in vec3 vertexColor;
        in vec2 textureCoordinate;
        uniform sampler2D surfaceTexture;
        uniform bool textureEnabled;
        out vec4 FragColor;

        void main()
        {
            // Lighting is still Gouraud; the texture only modulates its
            // interpolated result on meshes that explicitly enable it.
            vec3 textureColor = textureEnabled
                ? texture(surfaceTexture, textureCoordinate).rgb : vec3(1.0);
            FragColor = vec4(vertexColor * textureColor, 1.0);
        }
    )";

    // 4. Compile vertex shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(
        vertexShader,
        1,
        &vertexShaderSource,
        nullptr
    );

    glCompileShader(vertexShader);

    // 5. Compile fragment shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(
        fragmentShader,
        1,
        &fragmentShaderSource,
        nullptr
    );

    glCompileShader(fragmentShader);

    // 6. Create shader program
    unsigned int shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);

    glLinkProgram(shaderProgram);

    // Report shader errors explicitly instead of silently showing a blank window.
    int vertexCompiled, fragmentCompiled, linked;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &vertexCompiled);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &fragmentCompiled);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linked);
    if (!vertexCompiled || !fragmentCompiled || !linked)
    {
        char log[2048];
        glGetShaderInfoLog(vertexShader, sizeof(log), nullptr, log);
        std::cerr << "Vertex shader: " << log << '\n';
        glGetShaderInfoLog(fragmentShader, sizeof(log), nullptr, log);
        std::cerr << "Fragment shader: " << log << '\n';
        glGetProgramInfoLog(shaderProgram, sizeof(log), nullptr, log);
        std::cerr << "Shader program: " << log << '\n';
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(shaderProgram);
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Individual shaders no longer needed
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // -----------------------------
    // VAO + VBO
    // -----------------------------

    unsigned int VAO;
    unsigned int VBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // VAO remembers vertex configuration
    glBindVertexArray(VAO);

    // VBO stores vertex data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(
        GL_ARRAY_BUFFER,
        vertices.size() * sizeof(float),
        vertices.data(),
        GL_STATIC_DRAW
    );

    // Tell OpenGL how vertices are structured
    glVertexAttribPointer(
        0,                  // location = 0
        3,                  // x, y, z
        GL_FLOAT,
        GL_FALSE,
        VERTEX_FLOATS * sizeof(float),
        (void*)0
    );

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          VERTEX_FLOATS * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                          VERTEX_FLOATS * sizeof(float),
                          reinterpret_cast<void*>(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // A small procedural weave avoids an external image dependency while still
    // demonstrating UV mapping, repeating, filtering, and mipmapped sampling.
    constexpr int textureSize = 64;
    std::vector<unsigned char> courtTexturePixels(textureSize * textureSize * 3);
    for (int y = 0; y < textureSize; ++y)
        for (int x = 0; x < textureSize; ++x)
        {
            const bool broadTile = ((x / 8) + (y / 8)) % 2 == 0;
            const bool fineFiber = ((x / 2) + (y / 2)) % 2 == 0;
            const unsigned char shade = static_cast<unsigned char>(
                210 + (broadTile ? 20 : 0) + (fineFiber ? 8 : 0));
            const int pixel = (y * textureSize + x) * 3;
            courtTexturePixels[pixel] = shade;
            courtTexturePixels[pixel + 1] = shade;
            courtTexturePixels[pixel + 2] = shade;
        }
    unsigned int courtTexture;
    glGenTextures(1, &courtTexture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, courtTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureSize, textureSize, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, courtTexturePixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    const glm::mat4 model(1.0f);
    float cameraAzimuth = glm::radians(DEFAULT_CAMERA_AZIMUTH);
    float cameraElevation = glm::radians(DEFAULT_CAMERA_ELEVATION);
    float cameraDistance = DEFAULT_CAMERA_DISTANCE;
    constexpr float cameraTurnSpeed = 0.75f;
    const float cameraElevationSpeed = glm::radians(45.0f);
    constexpr float cameraZoomSpeed = 22.0f;
    constexpr float playerZoomSpeed = 40.0f;
    bool playerView = false;
    bool playerViewKeyWasDown = false;
    float playerLookYaw = 0.0f;
    float playerLookPitch = glm::radians(DEFAULT_PLAYER_LOOK_PITCH);
    float playerCameraFov = PLAYER_CAMERA_FOV;
    int lightingMode = 3;
    bool headSpotlightEnabled = false;
    bool headSpotlightKeyWasDown = false;

    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"),
                       1, GL_FALSE, glm::value_ptr(model));
    const int projectionLocation = glGetUniformLocation(shaderProgram, "projection");
    const int viewLocation = glGetUniformLocation(shaderProgram, "view");
    const int modelLocation = glGetUniformLocation(shaderProgram, "model");
    const int colorLocation = glGetUniformLocation(shaderProgram, "objectColor");
    const int viewPositionLocation = glGetUniformLocation(shaderProgram, "viewPosition");
    const int lightingModeLocation = glGetUniformLocation(shaderProgram, "lightingMode");
    const int headSpotlightPositionLocation =
        glGetUniformLocation(shaderProgram, "headSpotlightPosition");
    const int headSpotlightDirectionLocation =
        glGetUniformLocation(shaderProgram, "headSpotlightDirection");
    const int headSpotlightEnabledLocation =
        glGetUniformLocation(shaderProgram, "headSpotlightEnabled");
    emissiveLocation = glGetUniformLocation(shaderProgram, "emissive");
    diffuseStrengthLocation = glGetUniformLocation(shaderProgram, "diffuseStrength");
    specularStrengthLocation = glGetUniformLocation(shaderProgram, "specularStrength");
    shininessLocation = glGetUniformLocation(shaderProgram, "shininess");
    textureEnabledLocation = glGetUniformLocation(shaderProgram, "textureEnabled");
    glUniform1i(glGetUniformLocation(shaderProgram, "surfaceTexture"), 0);
    glUniform3fv(glGetUniformLocation(shaderProgram, "lightPositions[0]"),
                 4, glm::value_ptr(LIGHT_POSITIONS[0]));
    glUniform1f(glGetUniformLocation(shaderProgram, "headSpotlightInnerCutoff"),
                std::cos(glm::radians(HEAD_SPOTLIGHT_INNER_ANGLE)));
    glUniform1f(glGetUniformLocation(shaderProgram, "headSpotlightOuterCutoff"),
                std::cos(glm::radians(HEAD_SPOTLIGHT_OUTER_ANGLE)));

    double previousTime = glfwGetTime();
    int displayedScore = positiveGoalScore + negativeGoalScore;
    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        const double currentTime = glfwGetTime();
        // Cap long pauses (e.g. window dragging) to avoid sudden movement jumps.
        const float deltaTime = glm::clamp(static_cast<float>(currentTime - previousTime),
                                           0.0f, 0.05f);
        previousTime = currentTime;
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        if (width == 0 || height == 0)
        {
            glfwWaitEvents();
            previousTime = glfwGetTime();
            continue;
        }
        if (glfwGetWindowAttrib(window, GLFW_FOCUSED))
        {
            // Opposite keys cancel; turning and moving can happen together.
            const float moveInput = float(glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
                                  - float(glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS);
            const float turnInput = float(glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
                                  - float(glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS);
            updateRobot(moveInput, turnInput, deltaTime);
            handleDribbleInput(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
            handleKickInput(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
            updateBall(deltaTime);
            goalBannerTimeRemaining = glm::max(
                0.0f, goalBannerTimeRemaining - deltaTime);
            const bool playerViewKeyDown =
                glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
            if (playerViewKeyDown && !playerViewKeyWasDown)
                playerView = !playerView;
            playerViewKeyWasDown = playerViewKeyDown;
            const float cameraHorizontal =
                float(glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
              - float(glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
            const float cameraUpDown =
                float(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
              - float(glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS);
            const float cameraZoom =
                float(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
              - float(glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS);
            if (playerView)
            {
                playerLookYaw = glm::clamp(
                    playerLookYaw - cameraHorizontal * cameraTurnSpeed * deltaTime,
                    glm::radians(-PLAYER_LOOK_MAX_YAW),
                    glm::radians(PLAYER_LOOK_MAX_YAW));
                playerLookPitch = glm::clamp(
                    playerLookPitch + cameraUpDown * cameraElevationSpeed * deltaTime,
                    glm::radians(PLAYER_LOOK_MIN_PITCH),
                    glm::radians(PLAYER_LOOK_MAX_PITCH));
                playerCameraFov = glm::clamp(
                    playerCameraFov - cameraZoom * playerZoomSpeed * deltaTime,
                    PLAYER_CAMERA_MIN_FOV, PLAYER_CAMERA_MAX_FOV);
                if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
                {
                    playerLookYaw = 0.0f;
                    playerLookPitch = glm::radians(DEFAULT_PLAYER_LOOK_PITCH);
                    playerCameraFov = PLAYER_CAMERA_FOV;
                }
            }
            else
            {
                cameraAzimuth = std::remainder(
                    cameraAzimuth + cameraHorizontal * cameraTurnSpeed * deltaTime,
                    glm::radians(360.0f));
                cameraElevation = glm::clamp(
                    cameraElevation + cameraUpDown * cameraElevationSpeed * deltaTime,
                    glm::radians(CAMERA_MIN_ELEVATION),
                    glm::radians(CAMERA_MAX_ELEVATION));
                cameraDistance = glm::clamp(
                    cameraDistance - cameraZoom * cameraZoomSpeed * deltaTime,
                    CAMERA_MIN_DISTANCE, CAMERA_MAX_DISTANCE);
                if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
                {
                    cameraAzimuth = glm::radians(DEFAULT_CAMERA_AZIMUTH);
                    cameraElevation = glm::radians(DEFAULT_CAMERA_ELEVATION);
                    cameraDistance = DEFAULT_CAMERA_DISTANCE;
                }
            }
            if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS) lightingMode = 1;
            if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS) lightingMode = 2;
            if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS) lightingMode = 3;
            const bool headSpotlightKeyDown =
                glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS;
            if (headSpotlightKeyDown && !headSpotlightKeyWasDown)
                headSpotlightEnabled = !headSpotlightEnabled;
            headSpotlightKeyWasDown = headSpotlightKeyDown;
            if (displayedScore != positiveGoalScore + negativeGoalScore)
            {
                glfwSetWindowTitle(window, scoreTitle().c_str());
                displayedScore = positiveGoalScore + negativeGoalScore;
            }
        }
        else
        {
            spaceWasDown = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
            dribbleKeyWasDown = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
            playerViewKeyWasDown = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
            headSpotlightKeyWasDown =
                glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS;
        }
        glViewport(0, 0, width, height);
        const float aspect = static_cast<float>(width) / height;
        const float framingScale = glm::max(1.0f, CAMERA_REFERENCE_ASPECT / aspect);
        // A more top-down view exposes more of the court's length. Pull back just
        // enough to keep the surrounding structures framed as elevation changes.
        const float elevationRatio = glm::max(
            1.0f, std::sin(cameraElevation)
                / std::sin(glm::radians(DEFAULT_CAMERA_ELEVATION)));
        const float elevationFramingScale = 1.0f + 0.4f * (elevationRatio - 1.0f);
        const float effectiveCameraDistance =
            cameraDistance * framingScale * elevationFramingScale;
        // Preserve the centered overview at the default distance. As the user
        // zooms closer, progressively target the robot so it remains visible
        // even near either end of the court.
        const float playerFollowAmount = glm::clamp(
            (DEFAULT_CAMERA_DISTANCE - cameraDistance)
                / (DEFAULT_CAMERA_DISTANCE - CAMERA_MIN_DISTANCE),
            0.0f, 1.0f);
        const glm::vec3 birdCameraTarget = glm::mix(
            CAMERA_TARGET,
            glm::vec3(robotPosition.x, CAMERA_TARGET.y, robotPosition.z),
            playerFollowAmount);
        glm::vec3 cameraPosition;
        glm::mat4 view;
        if (playerView)
        {
            const glm::vec3 robotForward(
                std::sin(robotYaw), 0.0f, std::cos(robotYaw));
            cameraPosition = robotPosition
                + robotForward * PLAYER_CAMERA_FORWARD_OFFSET
                + glm::vec3(0, PLAYER_CAMERA_HEIGHT, 0);
            const float lookYaw = robotYaw + playerLookYaw;
            const glm::vec3 lookDirection(
                std::sin(lookYaw) * std::cos(playerLookPitch),
                std::sin(playerLookPitch),
                std::cos(lookYaw) * std::cos(playerLookPitch));
            view = glm::lookAt(
                cameraPosition, cameraPosition + lookDirection, glm::vec3(0, 1, 0));
        }
        else
        {
            const glm::vec3 cameraOffset(
                effectiveCameraDistance * std::cos(cameraElevation) * std::sin(cameraAzimuth),
                effectiveCameraDistance * std::sin(cameraElevation),
                effectiveCameraDistance * std::cos(cameraElevation) * std::cos(cameraAzimuth));
            cameraPosition = birdCameraTarget + cameraOffset;
            view = glm::lookAt(
                cameraPosition, birdCameraTarget, glm::vec3(0, 1, 0));
        }
        glUniformMatrix4fv(viewLocation, 1, GL_FALSE, glm::value_ptr(view));
        glUniform3fv(viewPositionLocation, 1, glm::value_ptr(cameraPosition));
        glUniform1i(lightingModeLocation, lightingMode);
        const glm::vec3 robotForward(
            std::sin(robotYaw), 0.0f, std::cos(robotYaw));
        const glm::vec3 headSpotlightPosition = robotPosition
            + robotForward * HEAD_SPOTLIGHT_FORWARD_OFFSET
            + glm::vec3(0.0f, HEAD_SPOTLIGHT_HEIGHT, 0.0f);
        const float spotlightPitch = glm::radians(HEAD_SPOTLIGHT_DOWN_ANGLE);
        const glm::vec3 headSpotlightDirection = glm::normalize(
            robotForward * std::cos(spotlightPitch)
            + glm::vec3(0.0f, -std::sin(spotlightPitch), 0.0f));
        glUniform3fv(headSpotlightPositionLocation, 1,
                     glm::value_ptr(headSpotlightPosition));
        glUniform3fv(headSpotlightDirectionLocation, 1,
                     glm::value_ptr(headSpotlightDirection));
        glUniform1i(headSpotlightEnabledLocation,
                    headSpotlightEnabled ? 1 : 0);
        const glm::mat4 projection = glm::perspective(
            glm::radians(playerView ? playerCameraFov : CAMERA_FOV),
            aspect, 0.1f, 200.0f);
        glUniformMatrix4fv(projectionLocation, 1, GL_FALSE, glm::value_ptr(projection));

        // background
        glClearColor(
            0.1f,
            0.1f,
            0.1f,
            1.0f
        );

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // use shaders
        glUseProgram(shaderProgram);

        // use field data
        glBindVertexArray(VAO);

        drawMesh(0, courtVertexCount, modelLocation, colorLocation, model,
                 glm::vec3(0.12f, 0.48f, 0.20f), false, 0.72f, 0.0f, 8.0f, true);
        drawMesh(markingFirst, markingCount, modelLocation, colorLocation, model,
                 glm::vec3(1.0f), false, 0.72f, 0.04f, 16.0f);
        drawGoal(COURT_MIN_Z, cubeFirst, modelLocation, colorLocation);
        drawGoal(COURT_MAX_Z, cubeFirst, modelLocation, colorLocation);
        // First-person view hides only the head/neck around the camera; every
        // other body part remains visible when it enters the viewing frustum.
        drawRobot(cubeFirst, modelLocation, colorLocation,
                  playerView, headSpotlightEnabled);
        glm::mat4 ballModel = glm::translate(glm::mat4(1.0f), ballPosition);
        ballModel *= glm::mat4_cast(ballOrientation);
        ballModel = glm::scale(ballModel, glm::vec3(BALL_RADIUS));
        drawMesh(ballWhiteFirst, ballWhiteCount, modelLocation, colorLocation,
                 ballModel, glm::vec3(0.95f), false, 0.80f, 0.65f, 40.0f);
        drawMesh(ballDarkFirst, ballDarkCount, modelLocation, colorLocation,
                 ballModel, glm::vec3(0.10f, 0.13f, 0.18f),
                 false, 0.80f, 0.65f, 40.0f);
        drawSurroundings(cubeFirst, modelLocation, colorLocation);
        drawHud(cubeFirst, modelLocation, colorLocation,
                viewLocation, projectionLocation, aspect);

        glfwSwapBuffers(window);
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteTextures(1, &courtTexture);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
