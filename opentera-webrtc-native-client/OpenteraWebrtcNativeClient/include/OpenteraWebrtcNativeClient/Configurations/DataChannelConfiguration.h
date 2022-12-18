#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_DATA_CHANNEL_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_DATA_CHANNEL_CONFIGURATION_H

#include <api/data_channel_interface.h>

#include <string>

namespace opentera
{
    /**
     * @brief Represents a data channel configuration
     */
    class DataChannelConfiguration
    {
        bool m_ordered;
        absl::optional<int> m_maxPacketLifeTime;  // It cannot be set with m_maxRetransmits
        absl::optional<int> m_maxRetransmits;  // It cannot be set with m_maxPacketLifeTime
        std::string m_protocol;

        DataChannelConfiguration(
            bool ordered,
            absl::optional<int> maxPacketLifeTime,
            absl::optional<int> maxRetransmits,
            std::string&& protocol);

    public:
        DataChannelConfiguration(const DataChannelConfiguration& other) = default;
        DataChannelConfiguration(DataChannelConfiguration&& other) = default;
        virtual ~DataChannelConfiguration() = default;

        static DataChannelConfiguration create();
        static DataChannelConfiguration create(bool ordered);
        static DataChannelConfiguration createProtocol(std::string protocol);
        static DataChannelConfiguration create(bool ordered, std::string protocol);

        static DataChannelConfiguration createMaxPacketLifeTime(int maxPacketLifeTime);
        static DataChannelConfiguration createMaxPacketLifeTime(bool ordered, int maxPacketLifeTime);
        static DataChannelConfiguration createMaxPacketLifeTime(int maxPacketLifeTime, std::string protocol);
        static DataChannelConfiguration
            createMaxPacketLifeTime(bool ordered, int maxPacketLifeTime, std::string protocol);

        static DataChannelConfiguration createMaxRetransmits(int maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, int maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(int maxRetransmits, std::string protocol);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, int maxRetransmits, std::string protocol);

        [[nodiscard]] bool ordered() const;
        [[nodiscard]] const absl::optional<int>& maxPacketLifeTime() const;
        [[nodiscard]] const absl::optional<int>& maxRetransmits() const;
        [[nodiscard]] const std::string& protocol() const;

        explicit operator webrtc::DataChannelInit() const;

        DataChannelConfiguration& operator=(const DataChannelConfiguration& other) = default;
        DataChannelConfiguration& operator=(DataChannelConfiguration&& other) = default;
    };

    /**
     * @brief Creates a data channel configuration with default values.
     * @return A data channel configuration with default values
     */
    inline DataChannelConfiguration DataChannelConfiguration::create()
    {
        return {true, absl::nullopt, absl::nullopt, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified value.
     *
     * @param ordered Indicates if the message order must be preserved
     * @return A data channel configuration with the specified value
     */
    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered)
    {
        return {ordered, absl::nullopt, absl::nullopt, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified value.
     *
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified value
     */
    inline DataChannelConfiguration DataChannelConfiguration::createProtocol(std::string protocol)
    {
        return {true, absl::nullopt, absl::nullopt, std::move(protocol)};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param ordered Indicates if the message order must be preserved
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered, std::string protocol)
    {
        return {ordered, absl::nullopt, absl::nullopt, std::move(protocol)};
    }

    /**
     * @brief Creates a data channel configuration with the specified value.
     *
     * @param maxPacketLifeTime Indicates the amount of time a message can be retransmitted (ms)
     * @return A data channel configuration with the specified value
     */
    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(int maxPacketLifeTime)
    {
        return {true, maxPacketLifeTime, absl::nullopt, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param ordered Indicates if the message order must be preserved
     * @param maxPacketLifeTime Indicates the amount of time a message can be retransmitted (ms)
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration
        DataChannelConfiguration::createMaxPacketLifeTime(bool ordered, int maxPacketLifeTime)
    {
        return {ordered, maxPacketLifeTime, absl::nullopt, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param maxPacketLifeTime Indicates the amount of time a message can be retransmitted (ms)
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration
        DataChannelConfiguration::createMaxPacketLifeTime(int maxPacketLifeTime, std::string protocol)
    {
        return {true, maxPacketLifeTime, absl::nullopt, std::move(protocol)};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param ordered Indicates if the message order must be preserved
     * @param maxPacketLifeTime Indicates the amount of time a message can be retransmitted (ms)
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration
        DataChannelConfiguration::createMaxPacketLifeTime(bool ordered, int maxPacketLifeTime, std::string protocol)
    {
        return {ordered, maxPacketLifeTime, absl::nullopt, std::move(protocol)};
    }

    /**
     * @brief Creates a data channel configuration with the specified value.
     *
     * @param maxRetransmits Indicates the maximum number of time a message can be retransmitted
     * @return A data channel configuration with the specified value
     */
    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(int maxRetransmits)
    {
        return {true, absl::nullopt, maxRetransmits, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param ordered Indicates if the message order must be preserved
     * @param maxRetransmits Indicates the maximum number of time a message can be retransmitted
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(bool ordered, int maxRetransmits)
    {
        return {ordered, absl::nullopt, maxRetransmits, ""};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param maxRetransmits Indicates the maximum number of time a message can be retransmitted
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration
        DataChannelConfiguration::createMaxRetransmits(int maxRetransmits, std::string protocol)
    {
        return {true, absl::nullopt, maxRetransmits, std::move(protocol)};
    }

    /**
     * @brief Creates a data channel configuration with the specified values.
     *
     * @param ordered Indicates if the message order must be preserved
     * @param maxRetransmits Indicates the maximum number of time a message can be retransmitted
     * @param protocol The data channel protocol
     * @return A data channel configuration with the specified values
     */
    inline DataChannelConfiguration
        DataChannelConfiguration::createMaxRetransmits(bool ordered, int maxRetransmits, std::string protocol)
    {
        return {ordered, absl::nullopt, maxRetransmits, std::move(protocol)};
    }

    /**
     * @brief Indicates if the message order must be preserved.
     * @return true if the message order must be preserved
     */
    inline bool DataChannelConfiguration::ordered() const { return m_ordered; }

    /**
     * @brief Returns the maximum number of time a message can be retransmitted.
     * @return The maximum number of time a message can be retransmitted
     */
    inline const absl::optional<int>& DataChannelConfiguration::maxPacketLifeTime() const
    {
        return m_maxPacketLifeTime;
    }

    /**
     * @brief Returns the maximum number of time a message can be retransmitted.
     * @return The maximum number of time a message can be retransmitted
     */
    inline const absl::optional<int>& DataChannelConfiguration::maxRetransmits() const { return m_maxRetransmits; }

    /**
     * @brief Returns the data channel protocol.
     * @return The data channel protocol
     */
    inline const std::string& DataChannelConfiguration::protocol() const { return m_protocol; }
}

#endif
