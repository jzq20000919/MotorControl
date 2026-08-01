#pragma once

#include <QWidget>

class PositionDial final : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(double currentDegree READ currentDegree WRITE setCurrentDegree)
    Q_PROPERTY(double targetDegree READ targetDegree WRITE setTargetDegree)

public:
    explicit PositionDial(QWidget *parent = nullptr);

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

    double currentDegree() const;
    double targetDegree() const;
    bool isDragging() const;

public slots:
    void setCurrentDegree(double degree);
    void setTargetDegree(double degree);

signals:
    void targetChanged(double degree);
    void interactionStarted();
    void interactionFinished(double degree);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void updateTargetFromPoint(const QPointF &point);
    bool acceptsPoint(const QPointF &point) const;
    static double normalized(double degree);

    double currentDegree_ = 0.0;
    double targetDegree_ = 0.0;
    bool dragging_ = false;
};
