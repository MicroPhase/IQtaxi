#pragma once

#include <QObject>

#include "plugin/plugininterface.h"

class PluginAPI;

#define IQTAXI_DEVICE_TYPE_ID "sdrangel.samplesource.iqtaxi"

class IqtaxiPlugin : public QObject, public PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(PluginInterface)
    Q_PLUGIN_METADATA(IID IQTAXI_DEVICE_TYPE_ID)

public:
    explicit IqtaxiPlugin(QObject *parent = nullptr);

    const PluginDescriptor &getPluginDescriptor() const override;
    void initPlugin(PluginAPI *pluginAPI) override;

    void enumOriginDevices(QStringList &listedHwIds, OriginDevices &originDevices) override;
    SamplingDevices enumSampleSources(const OriginDevices &originDevices) override;

    DeviceGUI *createSampleSourcePluginInstanceGUI(
        const QString &sourceId,
        QWidget **widget,
        DeviceUISet *deviceUISet) override;

    DeviceSampleSource *createSampleSourcePluginInstance(
        const QString &sourceId,
        DeviceAPI *deviceAPI) override;

    DeviceWebAPIAdapter *createDeviceWebAPIAdapter() const override;

private:
    static const PluginDescriptor m_pluginDescriptor;
};
