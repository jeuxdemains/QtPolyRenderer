#include "PolyRenderer.h"
#include "stdafx.h"

#define max(a, b) (a < b) ? b : a
#define min(a, b) (a < b) ? a : b

PolyRenderer::PolyRenderer(QWidget *parent) : QWidget(parent)
{
    ui.setupUi(this);
    this->grabMouse();

    startTimer(1);
}

void PolyRenderer::HandleKeyStates()
{
    Vec2 plrLastPos;
    plrLastPos = { m_PlayerPos.x(), m_PlayerPos.y() };

    if (m_KeyStates[Qt::Key_W] || m_KeyStates[Qt::Key_S])
    {
        int dir = 1;
        if (m_KeyStates[Qt::Key_S])
            dir = -1;

        double x = m_PlayerPos.x() + (dir * MOV_SPEED) * cos(m_PlrAngle);
        m_PlayerPos.setX(x);
        //if (DetectCollision())
        //    m_PlayerPos.setX(plrLastPos.x);

        double y = m_PlayerPos.y() + (dir * MOV_SPEED) * sin(m_PlrAngle);
        m_PlayerPos.setY(y);
        //if (DetectCollision())
        //    m_PlayerPos.setY(plrLastPos.y);
    }

    if (m_KeyStates[Qt::Key_Control])
        m_PlayerVertPos += 1;
    else if (m_KeyStates[Qt::Key_Space])
        m_PlayerVertPos -= 1;

    if (m_KeyStates[Qt::Key_A] || m_KeyStates[Qt::Key_D])
    {
        double dir = m_PlrAngle - M_PI/2.0;
        if (m_KeyStates[Qt::Key_D])
            dir = m_PlrAngle + M_PI / 2.0;

        double x = m_PlayerPos.x() + (MOV_SPEED * cos(dir));
        m_PlayerPos.setX(x);
        //if (DetectCollision())
        //    m_PlayerPos.setX(plrLastPos.x);

        double y = m_PlayerPos.y() + (MOV_SPEED * sin(dir));
        m_PlayerPos.setY(y);
        //if (DetectCollision())
        //    m_PlayerPos.setY(plrLastPos.y);
    }

    ResolveCollisions(plrLastPos);
}

bool PolyRenderer::DetectCollision()
{
    Vec2 playerPos;
    playerPos.x = m_PlayerPos.x();
    playerPos.y = m_PlayerPos.y();
    double radius = 10;

    for (int polyIdx = 0; polyIdx < m_Polygons.size(); polyIdx++)
    {
        std::vector<QPointF>& points = m_Polygons[polyIdx].polygon;
        for (int i = 0; i < points.size() - 1; i++)
        {
            QPointF p1 = points[i];
            QPointF p2 = points[i + 1];

            LineSeg line;
            line.p1.x = points[i].x();
            line.p1.y = points[i].y();
            line.p2.x = points[i + 1].x();
            line.p2.y = points[i + 1].y();


            if (Physics::LineCircleCollision(line, playerPos, radius) != 0)
                return true;
        }
    }

    return false;
}

void PolyRenderer::ResolveCollisions(Vec2 plrLastPos)
{
    Vec2 playerPos;
    playerPos.x = m_PlayerPos.x();
    playerPos.y = m_PlayerPos.y();

    bool collisionDetected = false;
    int collisionXY = 0;
    LineSeg collidedLine;
    double radius = 10;

    for (int polyIdx = 0; polyIdx < m_Polygons.size(); polyIdx++)
    {
        std::vector<QPointF>& points = m_Polygons[polyIdx].polygon;
        for (int i = 0; i < points.size() - 1; i++)
        {
            QPointF p1 = points[i];
            QPointF p2 = points[i + 1];

            LineSeg line;
            line.p1.x = points[i].x();
            line.p1.y = points[i].y();
            line.p2.x = points[i + 1].x();
            line.p2.y = points[i + 1].y();

            collisionXY = Physics::LineCircleCollision(line, playerPos, radius);

            if (collisionXY != 0)
            {
                collidedLine = line;
                collisionDetected = true;
                break;
            }
        }
    }

    if (collisionDetected)
    {
        Vec2 resolvedXY = Physics::ResolveCollision(plrLastPos, playerPos, collidedLine);
        m_PlayerPos.setX(resolvedXY.x);
        m_PlayerPos.setY(resolvedXY.y);
    }
}

void PolyRenderer::UpdateFrame()
{
    if (!fpsMode)
        RayCast(m_PlayerPos);

    this->update();
}

void PolyRenderer::ModeDraw(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPointF point = event->localPos();
        m_TempPolyPoints.emplace_back(point);

        if (m_TempPolyPoints.size() > 0)
            this->update();
    }
    else if (event->button() == Qt::RightButton)
    {
        FinalizePolygon();
    }
}

void PolyRenderer::PolyDraw(QPointF playerPos, QPainter& painter)
{
    QPen pen(Qt::black, PEN_SIZE_LINE, Qt::SolidLine);
    painter.setPen(pen);

    for (int polyIdx = 0; polyIdx < m_Polygons.size(); polyIdx++)
    {

        std::vector<QPointF>& points = m_Polygons[polyIdx].polygon;
        for (int i = 0; i < points.size() - 1; i++)
        {
            QPointF p1 = points[i];
            QPointF p2 = points[i + 1];

            if (VecMath::IsFrontFace({ playerPos.x(), playerPos.y() } , { p1.x(), p1.y() }, { p2.x(), p2.y() }) > 0)
            {
                continue;
            }

            double ceilH = m_Polygons[polyIdx].ceil;

            double distX1 = p1.x() - playerPos.x();
            double distY1 = p1.y() - playerPos.y();
            double z1 = distX1 * cos(m_PlrAngle) + distY1 * sin(m_PlrAngle); //tz1

            double distX2 = p2.x() - playerPos.x();
            double distY2 = p2.y() - playerPos.y();
            double z2 = distX2 * cos(m_PlrAngle) + distY2 * sin(m_PlrAngle); //tz2

            distX1 = distX1 * sin(m_PlrAngle) - distY1 * cos(m_PlrAngle); //tx1
            distX2 = distX2 * sin(m_PlrAngle) - distY2 * cos(m_PlrAngle); //tx2

            //painter.drawLine(vtx1, vtz1, vtx2, vtz2);

            if (z1 > 0 || z2 > 0)
            {
                double cx1, cy1, cx2, cy2;
                VecMath::Intersection(distX1, z1, distX2, z2, -0.0001, 0.0001, -20, 5, cx1, cy1); //ix1, iz1
                VecMath::Intersection(distX1, z1, distX2, z2, 0.0001, 0.0001, 20, 5, cx2, cy2); //ix2, iz2

                if (z1 <= 0)
                    if (cy1 > 0)
                    {
                        distX1 = cx1;
                        z1 = cy1;
                    }
                    else
                    {
                        distX1 = cx2;
                        z1 = cy2;
                    }

                if (z2 <= 0)
                    if (cy1 > 0)
                    {
                        distX2 = cx1;
                        z2 = cy1;
                    }
                    else
                    {
                        distX2 = cx2;
                        z2 = cy2;
                    }
            }
            else
                continue;

            double widthRatio = this->width()/2;
            double heightRatio = (this->width() * this->height()) / 90.0;
            double centerScreenH = this->height() / 2;
            double centerScreenW = this->width() / 2;

            double x1 = -distX1 * widthRatio / z1;
            double x2 = -distX2 * widthRatio / z2;
            double y1a = (-ceilH + -heightRatio) / z1;
            double y1b = heightRatio / z1;
            double y2a = (-ceilH + -heightRatio) / z2;
            double y2b = heightRatio / z2;


            pen.setColor(QColor(255, 0, 0, 255));
            painter.setPen(pen);

            painter.drawLine(centerScreenW + x1, centerScreenH + y1a, centerScreenW + x2, centerScreenH + y2a); //top

            pen.setColor(QColor(0, 0, 0, 255));
            painter.setPen(pen);

            painter.drawLine(centerScreenW + x1, centerScreenH + y1b, centerScreenW + x2, centerScreenH + y2b); //bottom
            painter.drawLine(centerScreenW + x1, centerScreenH + y1a, centerScreenW + x1, centerScreenH + y1b); //left
            painter.drawLine(centerScreenW + x2, centerScreenH + y2a, centerScreenW + x2, centerScreenH + y2b); //right

            QPolygon poly;
            poly << QPoint(centerScreenW + x1, centerScreenH + y1a);
            poly << QPoint(centerScreenW + x2, centerScreenH + y2a);
            poly << QPoint(centerScreenW + x1, centerScreenH + y1b);
            poly << QPoint(centerScreenW + x2, centerScreenH + y2b);
            poly << QPoint(centerScreenW + x1, centerScreenH + y1a);
            poly << QPoint(centerScreenW + x1, centerScreenH + y1b);
            poly << QPoint(centerScreenW + x2, centerScreenH + y2a);
            poly << QPoint(centerScreenW + x2, centerScreenH + y2b);

            Color c = CalcWallSliceColor(z2);

            QBrush brush;
            brush.setColor(QColor(c.red, c.green, c.blue, 255));
            brush.setStyle(Qt::BrushStyle::SolidPattern);

            QPainterPath path;
            path.addPolygon(poly);

            painter.fillPath(path, brush);

            //for (int x = x1; x < x2; x++)
            //{
            //    double ya = y1a + (x - x1) * int(y2a - y1a) / (x2 - x1);
            //    double yb = y1b + (x - x1) * int(y2b - y1b) / (x2 - x1);

            //    QColor wallColor(255, 255, 0, 255);
            //    pen.setColor(wallColor);
            //    painter.setPen(pen);

            //    //painter.drawLine(centerScreenW + x, 0, centerScreenW + x, centerScreenH - ya); //ceiling slice
            //    //painter.drawLine(centerScreenW + x, centerScreenH + yb, centerScreenW + x, this->height()); //floor slice
            //    painter.drawLine(centerScreenW + x, centerScreenH + ya, centerScreenW + x, centerScreenH + yb); //wall slice
            //}
        }
    }
}

void PolyRenderer::SortPolysDepth()
{

}

void PolyRenderer::SaveMap()
{
    QString filename = QFileDialog::getSaveFileName(this, "Save As");

    if (filename.isEmpty())
        return;

    std::ofstream file;
    file.open(filename.toStdString());

    //polygons array
    for (int pi = 0; pi < m_Polygons.size(); pi++)
    {
        std::vector<QPointF>& points = m_Polygons[pi].polygon;
        for (int vi = 0; vi < points.size(); vi++)
        {
            std::string x = QString::number(points[vi].x(), 'f', 2).toStdString();
            std::string y = QString::number(points[vi].y(), 'f', 2).toStdString();

            file << "vars.polys[" << pi << "].vert[" << vi << "].x = " << x << ";\n";
            file << "vars.polys[" << pi << "].vert[" << vi << "].y = " << y << ";\n";
        }

        file << "vars.polys[" << pi << "].height = " << m_Polygons[pi].ceil << ";\n";
        file << "vars.polys[" << pi << "].vertCnt = " << m_Polygons[pi].polygon.size() << ";\n";
    }


    //player position
    std::string plrX = QString::number(m_PlayerPos.x(), 'f', 2).toStdString();
    std::string plrY = QString::number(m_PlayerPos.y(), 'f', 2).toStdString();
    std::string plrAngle = QString::number(m_PlrAngle, 'f', 2).toStdString();

    file << "player position\n";
    file << plrX << ' ' << plrY << '\n';
    file << plrAngle << '\n';

    file.close();
}

void PolyRenderer::RayCast(QPointF playerPos)
{
    m_Rays.clear();

    int scrnW = this->width();
    double angleDelta = -FOV / 2;

    float rayAngle = m_PlrAngle - (FOV / 2.0); //first ray angle = (cur Cam angle - half the FOV angle)
    float fixedRayAngDelta = FOV / scrnW;

    float dist = m_CamPlaneDist;
    float screen_halflen = dist * tan(FOV / 2.0);
    float seg_len = screen_halflen / (scrnW / 2);

    for (int scrnClmn = 0; scrnClmn < scrnW; scrnClmn++)
    {
        //angleDelta += (FOV / scrnW);
        angleDelta = m_PlrAngle + atan((seg_len * scrnClmn - screen_halflen) / dist);

        QPointF rayEnd;
        rayEnd.setX(playerPos.x() + RAY_LEN * cos(angleDelta));
        rayEnd.setY(playerPos.y() + RAY_LEN * sin(angleDelta));

        Ray r;
        r.x = m_PlayerPos.x();
        r.y = m_PlayerPos.y();
        r.xTo = rayEnd.x();
        r.yTo = rayEnd.y();
        r.dist = RAY_LEN * 2;

        double distance = RAY_LEN * 2;

        for (int polyIdx = 0; polyIdx < m_Polygons.size(); polyIdx++)
        {
            QPointF intersection(0, 0);

            std::vector<QPointF>& points = m_Polygons[polyIdx].polygon;
            QPointF point1;
            QPointF point2;

            //find the two closest points of a polyline
            for (int i = 0; i < points.size() - 1; i++)
            {
                point1 = points[i];
                point2 = points[i + 1];

                float x, y;
                int intersects = get_line_intersection(
                    playerPos.x(), playerPos.y(), rayEnd.x(), rayEnd.y(),
                    point1.x(), point1.y(), point2.x(), point2.y(), &x, &y);

                if (intersects)
                {
                    double raylen = CalcIntersectionDist(r.x, r.y, x, y, angleDelta, m_PlrAngle);
                    
                    //closest poly line intersection
                    if (raylen < distance)
                    {
                        intersection.setX(x);
                        intersection.setY(y);
                        distance = raylen;

                        r.xTo = intersection.x();
                        r.yTo = intersection.y();
                        r.dist = distance;
                        r.angle = angleDelta;
                        r.height = m_Polygons[polyIdx].height;
                        r.ceil = m_Polygons[polyIdx].ceil;
                        r.point1 = point1;
                        r.point2 = point2;
                        r.point1Dist = CalcIntersectionDist(r.x, r.y, point1.x(), point1.y(), angleDelta, m_PlrAngle);
                        r.point2Dist = CalcIntersectionDist(r.x, r.y, point2.x(), point2.y(), angleDelta, m_PlrAngle);
                        r.screenColumn = scrnClmn;

                       //continue;
                    } 
                }
            }

           if (distance == RAY_LEN * 2)
               continue;

           //poly intersection
           if (intersection.x() != 0 && intersection.y() != 0 && distance < r.dist)
           {
               r.xTo = intersection.x();
               r.yTo = intersection.y();
               r.dist = distance;
               r.angle = angleDelta;
               r.height = m_Polygons[polyIdx].height;
               r.ceil = m_Polygons[polyIdx].ceil;
               r.point1 = point1;
               r.point2 = point2;
               r.point1Dist = CalcIntersectionDist(r.x, r.y, point1.x(), point1.y(), angleDelta, m_PlrAngle);
               r.point2Dist = CalcIntersectionDist(r.x, r.y, point2.x(), point2.y(), angleDelta, m_PlrAngle);

               //if (fpsMode) //if FPS mode collect data for walls behind walls
               //    m_Rays.emplace_back(r);
           }
        }

        r.screenColumn = scrnClmn;
        if (distance == RAY_LEN * 2) //if no wall hit - mark it as non renderable
            r.noRender = true;
            
        m_Rays.emplace_back(r);
    }

    m_Rays.size();
}

float PolyRenderer::CalcIntersectionDist(float x, float y, float xTo, float yTo, float rayAngle, float angle)
{
    float fRayLength = 0;
    float xDist, yDist;

    xDist = fabs(x - xTo);
    yDist = fabs(y - yTo);
    if (xDist > yDist)
        fRayLength = fabs(xDist / cosf(rayAngle));
    else
        fRayLength = fabs(yDist / sinf(rayAngle));

    fRayLength *= cosf(angle - rayAngle);

    return fRayLength;
}

int PolyRenderer::get_line_intersection(
    float p0_x, float p0_y, float p1_x, float p1_y,
    float p2_x, float p2_y, float p3_x, float p3_y, 
    float* i_x, float* i_y)
{
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return 0; // Collinear

    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    
    if ((s_numer < 0) == denomPositive)
        return 0; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return 0; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return 0; // No collision

    // Collision detected
    t = t_numer / denom;
    if (i_x != NULL)
        *i_x = p0_x + (t * s10_x);
    if (i_y != NULL)
        *i_y = p0_y + (t * s10_y);

    return 1;
}

void PolyRenderer::FinalizePolygon()
{
    if (m_TempPolyPoints.size() == 0)
        return;

    m_TempPolyPoints.emplace_back(m_TempPolyPoints[0]);
    Poly p(m_TempPolyPoints);
    p.ceil = CEIL_HEIGHT[rand() % 3];
    AddPolygon(p);

    for (int i = 0; i < m_TempPolyPoints.size(); i++)
        m_PolyPoints.emplace_back(m_TempPolyPoints[i]);

    m_TempPolyPoints.clear();

    this->update();
}


void PolyRenderer::RenderWalls(QPainter& painter)
{
    for (int i = 0; i < m_Rays.size(); i++)
    {
        Ray r = m_Rays[i];
        if (r.ceil == 0 || r.noRender)
            continue;

        RenderWallSlice(painter, r);
    }
}

void PolyRenderer::RenderWallSlice(QPainter& painter, Ray& r)
{
    QPen pen(Qt::black, PEN_SIZE_LINE, Qt::SolidLine);
    painter.setPen(pen);

    //double dist1 = r.point1Dist;
    //double dist2 = r.point2Dist;

    Color clr = CalcWallSliceColor(r.dist);
    QColor wallColor(clr.red, clr.green, clr.blue, 255);
    pen.setColor(wallColor);
    painter.setPen(pen);

    double wallH = (this->height() / r.dist) * r.height;
    double y = (this->height() / 2) - (wallH  / 2);
    y += m_PlayerYAxis;
    y -= ((m_PlayerVertPos * 2000) + r.ceil) / r.dist;
    wallH += r.ceil / r.dist;

    painter.drawLine(r.screenColumn, y, r.screenColumn, y + wallH);

    /*double wallH2 = (this->height() / dist2) * r.height;
    double y2 = (this->height() / 2) - (wallH2 / 2);
    y2 += m_PlayerYAxis;
    y2 -= ((m_PlayerVertPos * 2000) + r.ceil) / dist2;
    wallH2 += r.ceil / dist2;

    float anglePoint1 = atan2(r.point1.x() - m_PlayerPos.x(), r.point1.y() - m_PlayerPos.y());
    float anglePoint2 = atan2(r.point2.x() - m_PlayerPos.x(), r.point2.y() - m_PlayerPos.y());
    
    anglePoint1 = floorf(anglePoint1 * 100000) / 100000;
    anglePoint2 = floorf(anglePoint2 * 100000) / 100000;

    int scrnx1 = 0;
    if (AToXTable.count(anglePoint1) != 0)
        scrnx1 = AToXTable[anglePoint1];

    int scrnx2 = 0;
    if (AToXTable.count(anglePoint2) != 0)
        scrnx2 = AToXTable[anglePoint2];

    int x = scrnx1;
    int end = scrnx2;
    if (scrnx1 > scrnx2)
    {
        x = scrnx2;
        end = scrnx1;
    }

    for (int i = x; i < end; i++)
    {
        painter.drawLine(x, y, x, y + wallH);

    }*/

    //painter.drawLine(r.screenColumn, y, r.screenColumn, y + wallH);
   /* painter.drawLine(r.screenColumn + wallW, y2, r.screenColumn + wallW, y2 + wallH2);
    painter.drawLine(r.screenColumn, y, r.screenColumn + wallW, y);
    painter.drawLine(r.screenColumn, y2, r.screenColumn + wallW, y2);*/

    //if (r.distBack > r.dist && r.distBack < RAY_LEN*2)
    //{
    //    double wallHBack = (this->height() / r.distBack) * r.height;
    //    double yBack = (this->height() / 2) - (wallHBack / 2);
    //    yBack += m_PlayerYAxis;
    //    yBack -= ((m_PlayerVertPos * 2000) + r.ceil) / r.distBack;
    //    wallHBack += r.ceil / r.distBack;

    //    if (yBack < y)
    //    {
    //        wallColor.setRed(0);
    //        pen.setColor(wallColor);
    //        painter.setPen(pen);
    //        painter.drawLine(r.screenColumn, y, r.screenColumn, yBack);
    //    }
    //}
}

Color PolyRenderer::CalcWallSliceColor(double distanceToWall)
{
    int clrIntensity = (55 / (1 / distanceToWall * 255));
    int color = 155 - clrIntensity;
    if (color < 0)
        color = 0;
    else if (color > 255)
        color = 255;

    int red = 255 - clrIntensity;
    if (red < 0)
        red = 0;
    else if (red > 255)
        red = 255;

    Color clr;
    clr.red = red;
    clr.green = color;
    clr.blue = color;

    return clr;
}

void PolyRenderer::DrawCurrentPoly(QPainter& painter)
{
    if (m_TempPolyPoints.size() == 0)
        return;

    QPen pen(Qt::black, PEN_SIZE_LINE, Qt::DashLine);
    painter.setPen(pen);

    for (int i = 0; i < m_TempPolyPoints.size() - 1; i++)
    {
        QPointF point1 = m_TempPolyPoints[i];
        QPointF point2 = m_TempPolyPoints[i+1];
        painter.drawLine(point1, point2);
    }
   
    QPointF point1 = m_TempPolyPoints[m_TempPolyPoints.size() - 1];
    painter.drawLine(point1, m_CurMouseLocalPos);
}

void PolyRenderer::DrawTempVertices(QPainter& painter)
{
    QPen pen(Qt::black, PEN_SIZE_POINT, Qt::SolidLine);
    painter.setPen(pen);

    for (int i = 0; i < m_TempPolyPoints.size(); i++)
    {
        QPointF point = m_TempPolyPoints[i];
        painter.drawPoint(point.x(), point.y());
    }
}

void PolyRenderer::DrawPolygons(QPainter& painter)
{
    QPen pen(Qt::black, PEN_SIZE_POINT, Qt::SolidLine);

    for (int i = 0; i < m_Polygons.size(); i++)
    {
        for (int pidx = 0; pidx < m_Polygons[i].polygon.size() - 1; pidx++)
        {
            QPointF point1 = m_Polygons[i].polygon[pidx];
            QPointF point2 = m_Polygons[i].polygon[pidx + 1];

            pen.setColor(QColor(0, 0, 0, 255));

            pen.setWidth(PEN_SIZE_POINT);
            painter.setPen(pen);
            painter.drawPoint(point1.x(), point1.y());
            painter.drawPoint(point2.x(), point2.y());

            pen.setWidth(PEN_SIZE_LINE);
            painter.setPen(pen);
            painter.drawLine(point1, point2);

            pen.setWidth(2);
            pen.setColor(QColor(255, 0, 0, 255));
            painter.setPen(pen);

            LineSeg normal = VecMath::NormalLine({ point1.x(), point1.y() }, { point2.x(), point2.y() }, 40);
            painter.drawLine(normal.p1.x, normal.p1.y, normal.p2.x, normal.p2.y);
        }
    }

    if (m_CollisionVector.p1.x != 0 && m_CollisionVector.p1.y != 0)
    {
        pen.setColor(QColor(0, 255, 0, 255));
        painter.setPen(pen);
        painter.drawLine(m_CollisionVector.p1.x, m_CollisionVector.p1.y, m_CollisionVector.p2.x, m_CollisionVector.p2.y);
    }
}

void PolyRenderer::AddPolygon(Poly p)
{
    m_Polygons.emplace_back(p);
}