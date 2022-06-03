#pragma once

#include <QtWidgets/QWidget>
#include <QPoint>
#include <QMouseEvent>
#include "ui_PolyRenderer.h"
#include <unordered_map>
#include <math.h>
#include <fstream>
#include "VecMath.h"
#include "PhysicsEngine.h"

typedef struct _PolysList
{
    _PolysList(std::vector<QPointF> points)
    {
        for (int i = 0; i < points.size(); i++)
            polygon.emplace_back(points[i]);
    };

    std::vector<QPointF> polygon;
    double height = 50;
    double ceil = 2250;

} Poly;

typedef struct _Ray
{
    double x, y, xTo, yTo, dist, angle;
    QPointF point1, point2;
    double point1Dist, point2Dist;
    double point1ScrnX, point2ScrnX;
    int screenColumn;
    double height;
    double ceil;
    bool noRender = false;

} Ray;

struct Color
{
    int red, green, blue;
};

class PolyRenderer : public QWidget
{
    Q_OBJECT

public:
    PolyRenderer(QWidget *parent = Q_NULLPTR);

private:
    enum REND_MODE
    {
        DRAW,
        RENDER
    };

    std::unordered_map<int, bool> m_KeyStates =
    {
        {Qt::Key_W, false},
        {Qt::Key_S, false},
        {Qt::Key_A, false},
        {Qt::Key_D, false},
        {Qt::Key_Space, false},
        {Qt::Key_Control, false},
    };

    const int PEN_SIZE_LINE = 1;
    const int PEN_SIZE_POINT = 8;
    const double RAY_LEN = 2000;
    const double FOV = 60.0 * 3.14 / 180.0;
    const double ANG_MAX = 3.14*2;
    const double MOV_SPEED = 1;
    const double ROT_SPEED = 50;
    const double CEIL_HEIGHT[4] = { 1000, 10000, 50000, 100000 };
    LineSeg m_CollisionVector = {};

    double m_CamPlaneDist = 64 / 2;
    double m_ScrnHalfLen = m_CamPlaneDist * tan(FOV / 2.0);
    double m_MouseLastX = 0;
    double m_PlayerYAxis = 0;
    double m_PlayerVertPos = 0;
    std::unordered_map<float, int> AToXTable = {};

    Ui::PolyRendererClass ui;
    std::vector<QPointF> m_PolyPoints = {};
    std::vector<Ray> m_Rays = {};
    std::vector<QPointF> m_TempPolyPoints = {};
    std::vector<Poly> m_Polygons;
    QPointF m_CurMouseLocalPos = {};
    QPointF m_PlayerPos = {};
    REND_MODE m_Mode = REND_MODE::DRAW;
    double m_PlrAngle = 45.0 / (3.14 * 180);
    bool fpsMode = false;

    //rendering
    void ModeDraw(QMouseEvent* event);
    void DrawPolygons(QPainter& painter);
    void DrawTempVertices(QPainter& painter);
    void DrawCurrentPoly(QPainter& painter);
    void RenderWalls(QPainter& painter);
    void RenderWallSlice(QPainter& painter, Ray& r);
    Color CalcWallSliceColor(double distanceToWall);
    void AddPolygon(Poly p);
    void FinalizePolygon();
    void RayCast(QPointF playerPos);
    void PolyDraw(QPointF playerPos, QPainter& painter);

    int get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
        float p2_x, float p2_y, float p3_x, float p3_y, float* i_x, float* i_y);
    float CalcIntersectionDist
    (
        float x, float y, float xTo, float yTo,
        float rayAngle, float angle
    );

    //physics
    bool DetectCollision();
    void ResolveCollisions(Vec2 plrLastPos);

    //general
    void UpdateFrame();
    void HandleKeyStates();
    void SaveMap();

protected:
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent*);
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void timerEvent(QTimerEvent* event);
};
