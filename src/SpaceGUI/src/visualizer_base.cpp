#include "SpaceGUI/visualizer_base.hpp"
#include <QMetaObject>

VisualizerBase::VisualizerBase(QWidget* parent)
: QWidget(parent)
{
}

VisualizerBase::~VisualizerBase()
{
    // clear subscriptions explicitly (optional)
    subscriptions_.clear();
}

void VisualizerBase::setSerializedMessage(const QByteArray & /*serialized*/)
{
    // Default: do nothing. Derived classes can override if needed.
}

void VisualizerBase::runOnQtThread(std::function<void()> fn)
{
    // Use QMetaObject::invokeMethod with QueuedConnection to ensure GUI thread execution.
    // We wrap the std::function into a callable lambda for invokeMethod.
    QMetaObject::invokeMethod(this, [fn = std::move(fn)]() {
        fn();
    }, Qt::QueuedConnection);
}
#include "moc_visualizer_base.cpp"