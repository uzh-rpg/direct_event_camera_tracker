#ifndef BAG_SYNCHRONIZER_H_QB3OOYFD
#define BAG_SYNCHRONIZER_H_QB3OOYFD

#include <functional>
#include <tuple>
#include <array>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

////////////////////////////////////////////////////////////////////////////////

template <class ...Ms>
class BagSynchronizer
{
public:
    template <typename ...Ts>
    BagSynchronizer(Ts... topic_names) : topics({topic_names...}) {};


    // GET
    ///////////////////////////////////////////////////////////////////////////

    template <size_t I = 0>
    std::enable_if_t<I == sizeof...(Ms), bool>
    newMessage(const rosbag::MessageInstance& msg) {
        throw "Invalid topic name " + msg.getTopic();
    }

    template <size_t I = 0>
    std::enable_if_t<I < sizeof...(Ms), bool>
    newMessage(const rosbag::MessageInstance& msg) {
        if (topics[I] == msg.getTopic()) {
            //std::cout << "adding a message for " << msg.getTopic() << std::endl;
            std::get<I>(last_messages) =
                msg.instantiate< typename std::tuple_element<I, std::tuple<Ms...>>::type >();
        } else {
            newMessage<I+1>(msg);
        }

        // TODO: check if message complete and publish if so
        if (is_full()) {
            publish(std::index_sequence_for<Ms...>());
            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    void set_view(rosbag::View& view, const rosbag::Bag& bag)
    {
        for (const std::string& topic: topics) {
            view.addQuery(bag, rosbag::TopicQuery(topic));
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    void set_callback(std::function<void(boost::shared_ptr<const Ms>...)> c) { callback = c; }

    ////////////////////////////////////////////////////////////////////////////

    // empty tuple:
    template <size_t I = 0>
    std::enable_if_t<I == sizeof...(Ms), bool>
    is_full(const ros::Time t = ros::Time())
    {
        UNUSED(t);
        return true;
    }

    template <size_t I = 0>
    std::enable_if_t<I < sizeof...(Ms), bool>
    is_full(const ros::Time t = ros::Time())
    {
        auto msg = std::get<I>(last_messages);

        if (!msg) {
            //std::cout << "got null pointer for " << topics[I] << std::endl;
            return false; // found a null-pointer
        } else {
            if (t.isZero()) {
                // check if other messages have same timestamp as the first one
                return is_full<I+1>(msg->header.stamp);
            } else if (t != msg->header.stamp) {
                // current message has different timestamp
                //std::cout << "got different timestamp for " << topics[I] << ": " << t << " and " << msg->header.stamp << std::endl;
                return false;
            } else {
                return is_full<I+1>(t);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////

private:
    std::array<const std::string, sizeof...(Ms)> topics;
    std::tuple<boost::shared_ptr<const Ms>...> last_messages;

    std::function<void(boost::shared_ptr<const Ms>...)> callback;

    template <size_t ...Is>
    void publish(std::index_sequence<Is...>) {
        if (callback) {
            //std::cout << "calling callback for " << topics[0] << std::endl;
            callback( std::get<Is>(last_messages) ... );
        } else {
            std::cerr << "WARNING: No callback set for BagSynchronizer!" << std::endl;
        }
    }
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: BAG_SYNCHRONIZER_H_QB3OOYFD */
