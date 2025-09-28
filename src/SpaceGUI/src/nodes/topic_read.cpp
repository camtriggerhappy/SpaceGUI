#include "SpaceGUI/topic_read.hpp"

TopicReader::TopicReader(QObject* parent)
: QObject(parent), Node("gui_topic_read")
{
    timer_ = this->create_wall_timer(
        std::chrono::seconds(12), std::bind(&TopicReader::timer_callback, this));
}
/**
 * @warning this method explicitly only supports types that return a single message type
 */
void TopicReader::timer_callback()
{
    auto topics = this->get_topic_names_and_types();
    std::vector<std::pair<std::string, std::string>> data;
    for (const auto & item : topics) {
        data.push_back(std::pair<std::string, std::string>(item.first, item.second[0]));
    }
    emit dataReady(data);
}
#include "moc_topic_read.cpp"