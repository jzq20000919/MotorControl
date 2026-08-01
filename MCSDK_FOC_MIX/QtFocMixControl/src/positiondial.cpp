#include "positiondial.h"

#include <QMouseEvent>
#include <QPainter>
#include <QtMath>

#include <cmath>

PositionDial::PositionDial(QWidget *parent)
    : QWidget(parent)
{
    setCursor(Qt::CrossCursor);
    setFocusPolicy(Qt::StrongFocus);
    setToolTip(tr("点击或拖动圆盘设置目标机械角度"));
}

QSize PositionDial::minimumSizeHint() const
{
    return {240, 240};
}

QSize PositionDial::sizeHint() const
{
    return {340, 340};
}

double PositionDial::currentDegree() const
{
    return currentDegree_;
}

double PositionDial::targetDegree() const
{
    return targetDegree_;
}

bool PositionDial::isDragging() const
{
    return dragging_;
}

void PositionDial::setCurrentDegree(double degree)
{
    currentDegree_ = normalized(degree);
    update();
}

void PositionDial::setTargetDegree(double degree)
{
    targetDegree_ = normalized(degree);
    update();
}

void PositionDial::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const qreal side = qMin(width(), height());
    const QPointF center(width() / 2.0, height() / 2.0);
    const qreal radius = side * 0.41;

    QRadialGradient face(center - QPointF(radius * 0.18, radius * 0.22), radius * 1.25);
    face.setColorAt(0.0, QColor(QStringLiteral("#22324a")));
    face.setColorAt(1.0, QColor(QStringLiteral("#101722")));
    painter.setBrush(face);
    painter.setPen(QPen(QColor(QStringLiteral("#334a68")), 2));
    painter.drawEllipse(center, radius, radius);

    painter.save();
    painter.translate(center);
    for (int degree = 0; degree < 360; degree += 5) {
        painter.save();
        painter.rotate(degree);
        const bool major = (degree % 30) == 0;
        painter.setPen(QPen(major ? QColor(QStringLiteral("#94a9c7"))
                                  : QColor(QStringLiteral("#455b78")),
                            major ? 2.0 : 1.0));
        painter.drawLine(QPointF(0, -radius + 10),
                         QPointF(0, -radius + (major ? 27 : 19)));
        painter.restore();
    }
    painter.restore();

    painter.setPen(QColor(QStringLiteral("#8093ad")));
    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);
    for (int degree = 0; degree < 360; degree += 90) {
        const qreal radians = qDegreesToRadians(static_cast<qreal>(degree - 90));
        const QPointF point = center + QPointF(qCos(radians), qSin(radians)) * (radius - 43);
        const QRectF textRect(point.x() - 22, point.y() - 11, 44, 22);
        painter.drawText(textRect, Qt::AlignCenter, QString::number(degree));
    }

    auto drawPointer = [&](double degree, const QColor &color, qreal length, qreal width) {
        painter.save();
        painter.translate(center);
        painter.rotate(degree);
        painter.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(0, 14), QPointF(0, -length));
        painter.restore();
    };

    drawPointer(targetDegree_, QColor(QStringLiteral("#ffad42")), radius - 35, 4.0);
    drawPointer(currentDegree_, QColor(QStringLiteral("#45d1ff")), radius - 52, 7.0);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(QStringLiteral("#dbe9fb")));
    painter.drawEllipse(center, 7, 7);

    painter.setPen(QColor(QStringLiteral("#f2f7ff")));
    QFont valueFont = painter.font();
    valueFont.setPointSize(18);
    valueFont.setBold(true);
    painter.setFont(valueFont);
    painter.drawText(QRectF(center.x() - 80, center.y() + radius * 0.35, 160, 35),
                     Qt::AlignCenter,
                     QStringLiteral("%1°").arg(currentDegree_, 0, 'f', 1));
}

void PositionDial::mousePressEvent(QMouseEvent *event)
{
    if ((event->button() == Qt::LeftButton) && acceptsPoint(event->position())) {
        dragging_ = true;
        setFocus(Qt::MouseFocusReason);
        emit interactionStarted();
        updateTargetFromPoint(event->position());
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void PositionDial::mouseMoveEvent(QMouseEvent *event)
{
    if (dragging_ && event->buttons().testFlag(Qt::LeftButton) &&
        acceptsPoint(event->position())) {
        updateTargetFromPoint(event->position());
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void PositionDial::mouseReleaseEvent(QMouseEvent *event)
{
    if (dragging_ && (event->button() == Qt::LeftButton)) {
        if (acceptsPoint(event->position())) {
            updateTargetFromPoint(event->position());
        }
        dragging_ = false;
        emit interactionFinished(targetDegree_);
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void PositionDial::updateTargetFromPoint(const QPointF &point)
{
    const QPointF center(width() / 2.0, height() / 2.0);
    const QPointF delta = point - center;
    if (QLineF(center, point).length() < 20.0) {
        return;
    }

    targetDegree_ = normalized(qRadiansToDegrees(qAtan2(delta.x(), -delta.y())));
    update();
    emit targetChanged(targetDegree_);
}

bool PositionDial::acceptsPoint(const QPointF &point) const
{
    const QPointF center(width() / 2.0, height() / 2.0);
    const qreal distance = QLineF(center, point).length();
    const qreal radius = qMin(width(), height()) * 0.46;
    return (distance >= 20.0) && (distance <= radius);
}

double PositionDial::normalized(double degree)
{
    double value = std::fmod(degree, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value;
}
