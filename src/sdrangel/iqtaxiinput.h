#pragma once

#include <QByteArray>
#include <cstddef>
#include <QMutex>
#include <QString>
#include <string>

#include "dsp/devicesamplesource.h"
#include "util/message.h"

#include "iqtaxi_backend.h"

class DeviceAPI;

class IqtaxiInput : public DeviceSampleSource
{
    Q_OBJECT
public:
    class MsgConfigureIqtaxi : public Message
    {
        MESSAGE_CLASS_DECLARATION
    public:
        const IqtaxiSettings &getSettings() const { return m_settings; }
        bool getForce() const { return m_force; }

        static MsgConfigureIqtaxi *create(const IqtaxiSettings &settings, bool force)
        {
            return new MsgConfigureIqtaxi(settings, force);
        }

    private:
        IqtaxiSettings m_settings;
        bool m_force;

        MsgConfigureIqtaxi(const IqtaxiSettings &settings, bool force) :
            Message(),
            m_settings(settings),
            m_force(force)
        {}
    };

    class MsgStartStop : public Message
    {
        MESSAGE_CLASS_DECLARATION
    public:
        bool getStartStop() const { return m_startStop; }
        static MsgStartStop *create(bool startStop) { return new MsgStartStop(startStop); }

    private:
        bool m_startStop;
        explicit MsgStartStop(bool startStop) :
            Message(),
            m_startStop(startStop)
        {}
    };

    class MsgReportError : public Message
    {
        MESSAGE_CLASS_DECLARATION
    public:
        const QString &getError() const { return m_error; }
        static MsgReportError *create(const QString &error) { return new MsgReportError(error); }

    private:
        QString m_error;
        explicit MsgReportError(const QString &error) :
            Message(),
            m_error(error)
        {}
    };

    explicit IqtaxiInput(DeviceAPI *deviceAPI);
    ~IqtaxiInput() override;

    void destroy() override;

    void init() override;
    bool start() override;
    void stop() override;

    QByteArray serialize() const override;
    bool deserialize(const QByteArray &data) override;

    void setMessageQueueToGUI(MessageQueue *queue) override { m_guiMessageQueue = queue; }
    const QString &getDeviceDescription() const override;
    int getSampleRate() const override;
    void setSampleRate(int sampleRate) override;
    quint64 getCenterFrequency() const override;
    void setCenterFrequency(qint64 centerFrequency) override;

    bool handleMessage(const Message &message) override;

    // Required so Workspace title-bar start/stop (ChannelWebAPIUtils::run/stop) works
    // and keeps the device GUI Start button in sync.
    int webapiRunGet(
            SWGSDRangel::SWGDeviceState &response,
            QString &errorMessage) override;
    int webapiRun(
            bool run,
            SWGSDRangel::SWGDeviceState &response,
            QString &errorMessage) override;

private:
    bool applySettings(const IqtaxiSettings &settings, bool force);
    void onSamples(const int16_t *samples, size_t count, uint64_t timestamp);
    void onError(const std::string &error);

private:
    DeviceAPI *_deviceAPI;
    QMutex _mutex;
    QString _deviceDescription;
    bool _running;

    IqtaxiSettings _settings;
    IqtaxiBackend _backend;
};
