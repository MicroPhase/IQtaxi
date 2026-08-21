#include <QtPlugin>

#include "plugin/pluginapi.h"

#include "iqtaxiplugin.h"
#include "iqtaxiinput.h"
#include "iqtaxiwebapiadapter.h"
#include "iqtaxi_settings.h"
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
    for (std::size_t i = 0; i < kIqtaxiDeviceCapsCount; ++i)
    {
        const IqtaxiDeviceCaps &caps = kIqtaxiDeviceCaps[i];
        const QString hardwareId = QString::fromLatin1(caps.hardware_id);
        if (listedHwIds.contains(hardwareId))
        {
            continue;
        }

        originDevices.append(OriginDevice(
            QString::fromLatin1(caps.display_name),
            hardwareId,
            QString::fromLatin1(caps.model), // serial carries model name
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
