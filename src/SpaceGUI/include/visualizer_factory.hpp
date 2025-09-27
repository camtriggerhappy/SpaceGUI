// visualizer_factory.hpp
#pragma once
#include <functional>
#include <unordered_map>
#include <QString>
#include <memory>
#include "visualizer_base.hpp"

class VisualizerFactory {
public:
    using Creator = std::function<VisualizerBase*(QWidget* parent)>;

    static VisualizerFactory& instance() {
        static VisualizerFactory s;
        return s;
    }
    void registerCreator(const QString &ros_type, Creator c) {
        creators_[ros_type.toStdString()] = std::move(c);
    }
    VisualizerBase* create(const QString &ros_type, QWidget* parent = nullptr) {
        auto it = creators_.find(ros_type.toStdString());
        if (it != creators_.end()) return it->second(parent);
        return nullptr;
    }

private:
    std::unordered_map<std::string, Creator> creators_;
};
