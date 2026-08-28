#include <QtPlugin>

#include "plugin/pluginapi.h"

#include "iqtaxiplugin.h"
#include "iqtaxiinput.h"
#include "iqtaxiwebapiadapter.h"
#include "iqtaxi_settings.h"
#include "include/sdr/api/DeviceProfile.hpp"
#include "include/sdr/api/UdpDiscover.hpp"
#ifndef SERVER_MODE
#include "iqtaxigui.h"
#endif

const PluginDescriptor IqtaxiPlugin::m_pluginDescriptor = {
    QStringLiteral("IQTAXI"),
    QStringLiteral("IQTAXI Input"),
    QStringLiteral("0.2.0"),
    QStringLiteral("(c) MicroPhase"),
    QStringLiteral("https://www.microphase.cn"),
    true,
    QStringLiteral("https://www.microphase.cn")
};

static constexpr const char *const kDeviceTypeID = IQTAXI_DEVICE_TYPE_ID;

IqtaxiPlugin::IqtaxiPlugin(QObject *parent)
    : QObject(parent)
{
}

const PluginDescriptor &IqtaxiPlugin::getPluginDescriptor() const
{
    return m_pluginDescriptor;
}

void IqtaxiPlugin::initPlugin(PluginAPI *pluginAPI)
{
    pluginAPI->registerSampleSource(kDeviceTypeID, this);
}

void IqtaxiPlugin::enumOriginDevices(QStringList &listedHwIds, OriginDevices &originDevices)
{
    const auto discovered = sdr::api::iqtaxi_udp_discover(250);

    for (std::size_t i = 0; i < kIqtaxiDeviceCapsCount; ++i)
    {
        const IqtaxiDeviceCaps &caps = kIqtaxiDeviceCaps[i];
        const QString hardwareId = QString::fromLatin1(caps.hardware_id);
        if (listedHwIds.contains(hardwareId))
        {
            continue;
        }

        QString serial = QString::fromLatin1(caps.model);
        QString display = QString::fromLatin1(caps.display_name);
        for (const auto &info : discovered)
        {
            if (info.name != caps.model)
            {
                continue;
            }
            const std::string modelLabel =
                sdr::api::iqtaxi_model_label(info.name, info.board_version);
            display = QString("IQTAXI %1").arg(QString::fromStdString(modelLabel));
            if (!info.serial.empty())
            {
                serial = QString::fromStdString(info.serial);
                display += QString(" %1").arg(serial);
            }
            if (!info.addr.empty())
            {
                display += QString(" @ %1").arg(QString::fromStdString(info.addr));
            }
            break;
        }

        originDevices.append(OriginDevice(
            display,
            hardwareId,
            serial,
            static_cast<int>(i),
            1, // nb Rx
            0  // nb Tx
            ));
        listedHwIds.append(hardwareId);
    }
}

PluginInterface::SamplingDevices IqtaxiPlugin::enumSampleSources(const OriginDevices &originDevices)
{
    SamplingDevices result;

    for (OriginDevices::const_iterator it = originDevices.begin(); it != originDevices.end(); ++it)
    {
        for (std::size_t i = 0; i < kIqtaxiDeviceCapsCount; ++i)
        {
            const IqtaxiDeviceCaps &caps = kIqtaxiDeviceCaps[i];
            if (it->hardwareId != QString::fromLatin1(caps.hardware_id))
            {
                continue;
            }

            result.append(SamplingDevice(
                it->displayableName,
                it->hardwareId,
                kDeviceTypeID,
                it->serial.isEmpty() ? QString::fromLatin1(caps.model) : it->serial,
                it->sequence,
                PluginInterface::SamplingDevice::BuiltInDevice,
                PluginInterface::SamplingDevice::StreamSingleRx,
                1,
                0));
            break;
        }
    }

    return result;
}

DeviceGUI *IqtaxiPlugin::createSampleSourcePluginInstanceGUI(
    const QString &sourceId,
    QWidget **widget,
    DeviceUISet *deviceUISet)
{
#ifdef SERVER_MODE
    (void)sourceId;
    (void)widget;
    (void)deviceUISet;
    return nullptr;
#else
    if (sourceId == kDeviceTypeID)
    {
        auto *gui = new IqtaxiGui(deviceUISet);
        *widget = gui;
        return gui;
    }
    return nullptr;
#endif
}

DeviceSampleSource *IqtaxiPlugin::createSampleSourcePluginInstance(
    const QString &sourceId,
    DeviceAPI *deviceAPI)
{
    if (sourceId == kDeviceTypeID)
    {
        return new IqtaxiInput(deviceAPI);
    }
    return nullptr;
}

DeviceWebAPIAdapter *IqtaxiPlugin::createDeviceWebAPIAdapter() const
{
    return new IqtaxiWebAPIAdapter();
}
