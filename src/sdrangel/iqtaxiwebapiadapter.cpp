#include "iqtaxiwebapiadapter.h"

#include "util/simpleserializer.h"

IqtaxiWebAPIAdapter::IqtaxiWebAPIAdapter() = default;
IqtaxiWebAPIAdapter::~IqtaxiWebAPIAdapter() = default;

QByteArray IqtaxiWebAPIAdapter::serialize()
{
    SimpleSerializer s(1);
    s.writeString(1, QString::fromStdString(m_settings.device_addr));
    s.writeU32(2, m_settings.sample_rate_hz);
    s.writeU64(3, m_settings.center_freq_hz);
    s.writeU32(4, m_settings.rx_gain);
    s.writeU32(5, m_settings.channel);
    s.writeString(6, QString::fromStdString(m_settings.device_model));
    return s.final();
}

bool IqtaxiWebAPIAdapter::deserialize(const QByteArray &data)
{
    SimpleDeserializer d(data);
    if (!d.isValid())
    {
        return false;
    }

    QString addr;
    d.readString(1, &addr, "192.168.1.10");
    m_settings.device_addr = addr.toStdString();
    d.readU32(2, &m_settings.sample_rate_hz, 15360000u);
    quint64 centerFreq = static_cast<quint64>(m_settings.center_freq_hz);
    d.readU64(3, &centerFreq, 1000000000ull);
    m_settings.center_freq_hz = static_cast<uint64_t>(centerFreq);
    d.readU32(4, &m_settings.rx_gain, 30u);
    d.readU32(5, &m_settings.channel, 1u);
    QString model;
    d.readString(6, &model, "E206");
    m_settings.device_model = iqtaxiModelFromHardwareId(model.toStdString());
    m_settings.sample_rate_hz = iqtaxiNearestSampleRate(m_settings.device_model, m_settings.sample_rate_hz);
    m_settings.rx_gain = iqtaxiClampGain(m_settings.device_model, m_settings.rx_gain);
    return m_settings.is_valid();
}

int IqtaxiWebAPIAdapter::webapiSettingsGet(
    SWGSDRangel::SWGDeviceSettings &response,
    QString &errorMessage)
{
    (void)response;
    errorMessage = "IqtaxiWebAPIAdapter::webapiSettingsGet not implemented in skeleton";
    return 501;
}

int IqtaxiWebAPIAdapter::webapiSettingsPutPatch(
    bool force,
    const QStringList &deviceSettingsKeys,
    SWGSDRangel::SWGDeviceSettings &response,
    QString &errorMessage)
{
    (void)force;
    (void)deviceSettingsKeys;
    (void)response;
    errorMessage = "IqtaxiWebAPIAdapter::webapiSettingsPutPatch not implemented in skeleton";
    return 501;
}
