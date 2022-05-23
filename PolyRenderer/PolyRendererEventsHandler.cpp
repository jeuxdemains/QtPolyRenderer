#include "PolyRenderer.h"
#include "stdafx.h"

void PolyRenderer::timerEvent(QTimerEvent* event)
{
    HandleKeyStates();
    UpdateFrame();
}

void PolyRenderer::mousePressEvent(QMouseEvent* event)
{
    if (m_Mode == REND_MODE::DRAW)
        ModeDraw(event);

    if (event->button() == Qt::MiddleButton)
    {
        if (m_Mode == REND_MODE::DRAW)
        {
            m_Mode = REND_MODE::RENDER;
            m_PlayerPos = event->localPos();
        }
        else
            m_Mode = REND_MODE::DRAW;
    }
}

void PolyRenderer::mouseMoveEvent(QMouseEvent* event)
{
    if (m_Mode == REND_MODE::RENDER)
    {
        if (!fpsMode)
            m_PlayerPos = event->localPos();
        else
        {
            m_MouseLastX -= event->localPos().x();

            m_PlrAngle -= (10 * m_MouseLastX) / (M_PI * 180);
            if (m_PlrAngle < 0)
                m_PlrAngle = ANG_MAX;
            else if (m_PlrAngle > ANG_MAX)
                m_PlrAngle = 0;

            m_MouseLastX = event->localPos().x();
            m_PlayerYAxis = 4 * (this->height() / 2 - event->localPos().y());

        }
        UpdateFrame();
    }
    else
    {
        m_CurMouseLocalPos = event->localPos();
    }

}

void PolyRenderer::keyPressEvent(QKeyEvent* event)
{
    if (m_KeyStates.count(event->key()) > 0)
        m_KeyStates[event->key()] = true;

    if (event->key() == Qt::Key_F)
        fpsMode = !fpsMode;
    else if (event->key() == Qt::Key_P)
        SaveMap();
}

void PolyRenderer::keyReleaseEvent(QKeyEvent* event)
{
    if (m_KeyStates.count(event->key()) > 0)
    {
        m_KeyStates[event->key()] = false;
    }
}

void PolyRenderer::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    if (m_Mode == REND_MODE::DRAW)
    {
        DrawTempVertices(painter);
        DrawPolygons(painter);
        DrawCurrentPoly(painter);
    }
    else
    {

        if (fpsMode)
        {
            //RenderWalls(painter);
            PolyDraw(m_PlayerPos, painter);

        }
        else
        {
            QPen pen(Qt::black, PEN_SIZE_LINE, Qt::SolidLine);
            painter.setPen(pen);

            for (int i = 0; i < m_Rays.size(); i++)
                painter.drawLine(m_Rays[i].x, m_Rays[i].y, m_Rays[i].xTo, m_Rays[i].yTo);

            DrawPolygons(painter);
        }
    }
}
