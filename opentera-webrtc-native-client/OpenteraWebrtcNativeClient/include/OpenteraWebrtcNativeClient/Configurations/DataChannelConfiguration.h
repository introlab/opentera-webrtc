#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_DATA_CHANNEL_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_DATA_CHANNEL_CONFIGURATION_H

#include <sio_message.h>

#include <api/data_channel_interface.h>

#include <optional>
#include <string>

namespace introlab
{
    class DataChannelConfiguration
    {
        bool m_ordered;
        std::optional<int> m_maxPacketLifeTime; // It cannot be set with m_maxRetransmits
        std::optional<int> m_maxRetransmits; // It cannot be set with m_maxPacketLifeTime
        std::string m_protocol;

        DataChannelConfiguration(bool ordered, const std::optional<int>& maxPacketLifeTime,
                const std::optional<int>& maxRetransmits, const std::string& protocol);

    public:
        virtual ~DataChannelConfiguration() = default;

        static DataChannelConfiguration create();
        static DataChannelConfiguration create(bool ordered);
        static DataChannelConfiguration createProtocol(const std::string& protocol);
        static DataChannelConfiguration create(bool ordered, const std::string& protocol);

        static DataChannelConfiguration createMaxPacketLifeTime(const std::optional<int>& maxPacketLifeTime);
        static DataChannelConfiguration createMaxPacketLifeTime(bool ordered,
                const std::optional<int>& maxPacketLifeTime);
        static DataChannelConfiguration createMaxPacketLifeTime(const std::optional<int>& maxPacketLifeTime,
                const std::string& protocol);
        static DataChannelConfiguration createMaxPacketLifeTime(bool ordered,
                const std::optional<int>& maxPacketLifeTime, const std::string& protocol);

        static DataChannelConfiguration createMaxRetransmits(const std::optional<int>& maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, const std::optional<int>& maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(const std::optional<int>& maxRetransmits,
                const std::string& protocol);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, const std::optional<int>& maxRetransmits,
                const std::string& protocol);

        bool ordered() const;
        const std::optional<int>& maxPacketLifeTime() const;
        const std::optional<int>& maxRetransmits() const;
        const std::string& protocol() const;

        explicit operator webrtc::DataChannelInit() const;
    };

    inline DataChannelConfiguration DataChannelConfiguration::create()
    {
        return DataChannelConfiguration(true, std::nullopt, std::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered)
    {
        return DataChannelConfiguration(ordered, std::nullopt, std::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createProtocol(const std::string& protocol)
    {
        return DataChannelConfiguration(true, std::nullopt, std::nullopt, protocol);
    }

    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered, const std::string& protocol)
    {
        return DataChannelConfiguration(ordered, std::nullopt, std::nullopt, protocol);
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(
            const std::optional<int>& maxPacketLifeTime)
    {
        return DataChannelConfiguration(true, maxPacketLifeTime, std::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(bool ordered,
            const std::optional<int>& maxPacketLifeTime)
    {
        return DataChannelConfiguration(ordered, maxPacketLifeTime, std::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(
            const std::optional<int>& maxPacketLifeTime,
            const std::string& protocol)
    {
        return DataChannelConfiguration(true, maxPacketLifeTime, std::nullopt, protocol);
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(bool ordered,
            const std::optional<int>& maxPacketLifeTime, const std::string& protocol)
    {
        return DataChannelConfiguration(ordered, maxPacketLifeTime, std::nullopt, protocol);
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(
            const std::optional<int>& maxRetransmits)
    {
        return DataChannelConfiguration(true, std::nullopt, maxRetransmits, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(bool ordered,
            const std::optional<int>& maxRetransmits)
    {
        return DataChannelConfiguration(ordered, std::nullopt, maxRetransmits, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(
            const std::optional<int>& maxRetransmits, const std::string& protocol)
    {
        return DataChannelConfiguration(true, std::nullopt, maxRetransmits, protocol);
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(bool ordered,
            const std::optional<int>& maxRetransmits, const std::string& protocol)
    {
        return DataChannelConfiguration(ordered, std::nullopt, maxRetransmits, protocol);
    }

    inline bool DataChannelConfiguration::ordered() const
    {
        return m_ordered;
    }

    inline const std::optional<int>& DataChannelConfiguration::maxPacketLifeTime() const
    {
        return m_maxPacketLifeTime;
    }

    inline const std::optional<int>& DataChannelConfiguration::maxRetransmits() const
    {
        return m_maxRetransmits;
    }

    inline const std::string& DataChannelConfiguration::protocol() const
    {
        return m_protocol;
    }
}

#endif
