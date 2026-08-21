#pragma once

#include "device/devicewebapiadapter.h"
#include "iqtaxi_settings.h"

class IqtaxiWebAPIAdapter : public DeviceWebAPIAdapter
{
public:
    IqtaxiWebAPIAdapter();
    ~IqtaxiWebAPIAdapter() override;

    QByteArray serialize() override;
    bool deserialize(const QByteArray &data) override;

    int webapiSettingsGet(
        SWGSDRangel::SWGDeviceSettings &response,
        QString &errorMessage) override;

    int webapiSettingsPutPatch(
        bool force,
        const QStringList &deviceSettingsKeys,
        SWGSDRangel::SWGDeviceSettings &response,
        QString &errorMessage) override;

private:
    IqtaxiSettings m_settings;
};
