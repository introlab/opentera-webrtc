#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_DATA_CHANNEL_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_DATA_CHANNEL_CONFIGURATION_H

#include <sio_message.h>

#include <api/data_channel_interface.h>

#include <optional>
#include <string>

namespace introlab
{
    class DataChannelConfiguration
    {
        bool m_ordered;
        absl::optional<int> m_maxPacketLifeTime; // It cannot be set with m_maxRetransmits
        absl::optional<int> m_maxRetransmits; // It cannot be set with m_maxPacketLifeTime
        std::string m_protocol;

        DataChannelConfiguration(bool ordered, absl::optional<int> maxPacketLifeTime,
                absl::optional<int> maxRetransmits, std::string&& protocol);

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
        static DataChannelConfiguration createMaxPacketLifeTime(bool ordered, int maxPacketLifeTime,
                std::string protocol);

        static DataChannelConfiguration createMaxRetransmits(int maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, int maxRetransmits);
        static DataChannelConfiguration createMaxRetransmits(int maxRetransmits, std::string protocol);
        static DataChannelConfiguration createMaxRetransmits(bool ordered, int maxRetransmits, std::string protocol);

        bool ordered() const;
        const absl::optional<int>& maxPacketLifeTime() const;
        const absl::optional<int>& maxRetransmits() const;
        const std::string& protocol() const;

        explicit operator webrtc::DataChannelInit() const;

        DataChannelConfiguration& operator=(const DataChannelConfiguration& other) = default;
        DataChannelConfiguration& operator=(DataChannelConfiguration&& other) = default;
    };

    inline DataChannelConfiguration DataChannelConfiguration::create()
    {
        return DataChannelConfiguration(true, absl::nullopt, absl::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered)
    {
        return DataChannelConfiguration(ordered, absl::nullopt, absl::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createProtocol(std::string protocol)
    {
        return DataChannelConfiguration(true, absl::nullopt, absl::nullopt, std::move(protocol));
    }

    inline DataChannelConfiguration DataChannelConfiguration::create(bool ordered, std::string protocol)
    {
        return DataChannelConfiguration(ordered, absl::nullopt, absl::nullopt, std::move(protocol));
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(int maxPacketLifeTime)
    {
        return DataChannelConfiguration(true, maxPacketLifeTime, absl::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(bool ordered,
            int maxPacketLifeTime)
    {
        return DataChannelConfiguration(ordered, maxPacketLifeTime, absl::nullopt, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(int maxPacketLifeTime,
            std::string protocol)
    {
        return DataChannelConfiguration(true, maxPacketLifeTime, absl::nullopt, std::move(protocol));
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxPacketLifeTime(bool ordered,
            int maxPacketLifeTime, std::string protocol)
    {
        return DataChannelConfiguration(ordered, maxPacketLifeTime, absl::nullopt, std::move(protocol));
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(int maxRetransmits)
    {
        return DataChannelConfiguration(true, absl::nullopt, maxRetransmits, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(bool ordered, int maxRetransmits)
    {
        return DataChannelConfiguration(ordered, absl::nullopt, maxRetransmits, "");
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(int maxRetransmits,
            std::string protocol)
    {
        return DataChannelConfiguration(true, absl::nullopt, maxRetransmits, std::move(protocol));
    }

    inline DataChannelConfiguration DataChannelConfiguration::createMaxRetransmits(bool ordered, int maxRetransmits,
            std::string protocol)
    {
        return DataChannelConfiguration(ordered, absl::nullopt, maxRetransmits, std::move(protocol));
    }

    inline bool DataChannelConfiguration::ordered() const
    {
        return m_ordered;
    }

    inline const absl::optional<int>& DataChannelConfiguration::maxPacketLifeTime() const
    {
        return m_maxPacketLifeTime;
    }

    inline const absl::optional<int>& DataChannelConfiguration::maxRetransmits() const
    {
        return m_maxRetransmits;
    }

    inline const std::string& DataChannelConfiguration::protocol() const
    {
        return m_protocol;
    }
}

#endif
